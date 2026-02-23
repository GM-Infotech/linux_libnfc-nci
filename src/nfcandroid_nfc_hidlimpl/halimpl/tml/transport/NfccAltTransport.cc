/******************************************************************************
 *  Copyright 2021 NXP
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 ******************************************************************************/

/*
 * NfccAltTransport.cc – userspace transport for NXP NFC controllers.
 *
 * GPIO handling has been migrated from the deprecated sysfs
 * /sys/class/gpio interface to libgpiod 2.x (gpiod_chip_open /
 * gpiod_chip_request_lines API).  The sysfs interface was removed from
 * mainline kernel 6.6 and is absent on the target Raspberry Pi 6.12 kernel.
 *
 * Build dependency:  link with -lgpiod  (libgpiod-dev >= 2.x)
 */

#include <errno.h>
#include <fcntl.h>
#ifdef ANDROID
#include <hardware/nfc.h>
#endif
#include <gpiod.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include <NfccAltTransport.h>
#include <NfccI2cTransport.h>
#include <phNfcStatus.h>
#include <phNxpConfig.h>
#include <phNxpLog.h>
#include "phNxpNciHal_utils.h"

extern phTmlNfc_i2cfragmentation_t fragmentation_enabled;
extern phTmlNfc_Context_t *gpphTmlNfc_Context;

/* ── Constructor / Destructor ────────────────────────────────────────────── */

NfccAltTransport::NfccAltTransport()
    : mpGpioChip(nullptr), mpIntRequest(nullptr), mpOutRequest(nullptr) {}

NfccAltTransport::~NfccAltTransport() { ReleaseGpio(); }

/*******************************************************************************
**
** Function         ReleaseGpio
**
** Description      Release all libgpiod resources acquired by ConfigurePin().
**                  Safe to call even if ConfigurePin() was never called or
**                  failed partway through.
**
*******************************************************************************/
void NfccAltTransport::ReleaseGpio()
{
  if (mpOutRequest)
  {
    gpiod_line_request_release(mpOutRequest);
    mpOutRequest = nullptr;
  }
  if (mpIntRequest)
  {
    gpiod_line_request_release(mpIntRequest);
    mpIntRequest = nullptr;
  }
  if (mpGpioChip)
  {
    gpiod_chip_close(mpGpioChip);
    mpGpioChip = nullptr;
  }
}

/*******************************************************************************
**
** Function         ConfigurePin
**
** Description      Open the GPIO chip and request the three NFC control lines.
**
**                  Pin numbers and the chip path are read at runtime from
**                  libnfc-nxp.conf so they can be changed without recompiling:
**
**                    NXP_GPIO_CHIP_PATH="/dev/gpiochip0"   # string
**                    NXP_GPIO_INT=23                       # BCM offset, input
**                    NXP_GPIO_VEN=24                       # BCM offset, output
**                    NXP_GPIO_FWDNLD=25                    # BCM offset, output
**
**                  Any key that is absent falls back to the compiled-in default
**                  (GPIO_CHIP_PATH / PIN_INT / PIN_ENABLE / PIN_FWDNLD).
**
** Parameters       none
**
** Returns          NFCSTATUS_SUCCESS on success, NFCSTATUS_INVALID_DEVICE on
**                  any failure.
**
*******************************************************************************/
int NfccAltTransport::ConfigurePin()
{

  /* ── Resolve chip path ──────────────────────────────────────────────────
   * GetNxpStrValue writes a null-terminated string and returns non-zero on
   * success.  PATH_MAX (4096) is overkill for a devnode but safe.           */
  char chipPath[64] = GPIO_CHIP_PATH; /* pre-filled with compiled default */
  if (GetNxpStrValue(NAME_NXP_GPIO_CHIP_PATH, chipPath, sizeof(chipPath)) <= 0)
  {
    NXPLOG_TML_D("%s NXP_GPIO_CHIP_PATH not in config, using default: %s",
                 __func__, chipPath);
  }

  /* ── Resolve pin offsets ────────────────────────────────────────────────
   * GetNxpNumValue writes into a 'long' when len == sizeof(long).
   * We read into long then range-check before trusting the value.           */
  unsigned int pinInt = PIN_INT;
  unsigned int pinEnable = PIN_ENABLE;
  unsigned int pinFwDnld = PIN_FWDNLD;

  long cfgVal = 0;
  if (GetNxpNumValue(NAME_NXP_GPIO_INT, &cfgVal, sizeof(cfgVal)) > 0 &&
      cfgVal > 0 && cfgVal < 512)
  {
    pinInt = (unsigned int)cfgVal;
  }
  else
  {
    NXPLOG_TML_D("%s NXP_GPIO_INT not in config, using default: %u",
                 __func__, pinInt);
  }

  cfgVal = 0;
  if (GetNxpNumValue(NAME_NXP_GPIO_VEN, &cfgVal, sizeof(cfgVal)) > 0 &&
      cfgVal > 0 && cfgVal < 512)
  {
    pinEnable = (unsigned int)cfgVal;
  }
  else
  {
    NXPLOG_TML_D("%s NXP_GPIO_VEN not in config, using default: %u",
                 __func__, pinEnable);
  }

  cfgVal = 0;
  if (GetNxpNumValue(NAME_NXP_GPIO_FWDNLD, &cfgVal, sizeof(cfgVal)) > 0 &&
      cfgVal > 0 && cfgVal < 512)
  {
    pinFwDnld = (unsigned int)cfgVal;
  }
  else
  {
    NXPLOG_TML_D("%s NXP_GPIO_FWDNLD not in config, using default: %u",
                 __func__, pinFwDnld);
  }

  NXPLOG_TML_D("%s chip=%s  INT=%u  VEN=%u  FWDL=%u",
               __func__, chipPath, pinInt, pinEnable, pinFwDnld);

  /* ── Open GPIO chip ─────────────────────────────────────────────────── */
  mpGpioChip = gpiod_chip_open(chipPath);
  if (!mpGpioChip)
  {
    NXPLOG_TML_E("%s gpiod_chip_open(%s) failed: %s",
                 __func__, chipPath, strerror(errno));
    return NFCSTATUS_INVALID_DEVICE;
  }

  /* ── Request 1: interrupt input with rising-edge detection ─────────── */
  {
    struct gpiod_line_settings *settings = gpiod_line_settings_new();
    struct gpiod_line_config *line_cfg = gpiod_line_config_new();
    struct gpiod_request_config *req_cfg = gpiod_request_config_new();

    if (!settings || !line_cfg || !req_cfg)
    {
      NXPLOG_TML_E("%s allocation failed for INT request", __func__);
      gpiod_line_settings_free(settings);
      gpiod_line_config_free(line_cfg);
      gpiod_request_config_free(req_cfg);
      ReleaseGpio();
      return NFCSTATUS_INVALID_DEVICE;
    }

    gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_INPUT);
    gpiod_line_settings_set_edge_detection(settings, GPIOD_LINE_EDGE_RISING);
    gpiod_line_settings_set_bias(settings, GPIOD_LINE_BIAS_PULL_DOWN);

    gpiod_line_config_add_line_settings(line_cfg, &pinInt, 1, settings);
    gpiod_request_config_set_consumer(req_cfg, "nfc-irq");

    mpIntRequest = gpiod_chip_request_lines(mpGpioChip, req_cfg, line_cfg);

    gpiod_line_settings_free(settings);
    gpiod_line_config_free(line_cfg);
    gpiod_request_config_free(req_cfg);

    if (!mpIntRequest)
    {
      NXPLOG_TML_E("%s request for INT pin %u failed: %s",
                   __func__, pinInt, strerror(errno));
      ReleaseGpio();
      return NFCSTATUS_INVALID_DEVICE;
    }

    /* Store for use by GetIrqState() / wait4interrupt() */
    mPinInt = pinInt;
  }

  /* ── Request 2: VEN + FWDL outputs, both initially LOW ─────────────── */
  {
    struct gpiod_line_settings *settings = gpiod_line_settings_new();
    struct gpiod_line_config *line_cfg = gpiod_line_config_new();
    struct gpiod_request_config *req_cfg = gpiod_request_config_new();

    if (!settings || !line_cfg || !req_cfg)
    {
      NXPLOG_TML_E("%s allocation failed for output request", __func__);
      gpiod_line_settings_free(settings);
      gpiod_line_config_free(line_cfg);
      gpiod_request_config_free(req_cfg);
      ReleaseGpio();
      return NFCSTATUS_INVALID_DEVICE;
    }

    gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_OUTPUT);
    gpiod_line_settings_set_output_value(settings, GPIOD_LINE_VALUE_INACTIVE);

    unsigned int out_offsets[] = {pinEnable, pinFwDnld};
    gpiod_line_config_add_line_settings(line_cfg, out_offsets, 2, settings);
    gpiod_request_config_set_consumer(req_cfg, "nfc-ctrl");

    mpOutRequest = gpiod_chip_request_lines(mpGpioChip, req_cfg, line_cfg);

    gpiod_line_settings_free(settings);
    gpiod_line_config_free(line_cfg);
    gpiod_request_config_free(req_cfg);

    if (!mpOutRequest)
    {
      NXPLOG_TML_E("%s request for output pins (%u,%u) failed: %s",
                   __func__, pinEnable, pinFwDnld, strerror(errno));
      ReleaseGpio();
      return NFCSTATUS_INVALID_DEVICE;
    }

    /* Store for use by gpio_set_ven() / gpio_set_fwdl() */
    mPinEnable = pinEnable;
    mPinFwDnld = pinFwDnld;
  }

  NXPLOG_TML_D("%s Success", __func__);
  return NFCSTATUS_SUCCESS;
}

/* ── GPIO output helpers ─────────────────────────────────────────────────── */

void NfccAltTransport::gpio_set_ven(int value)
{
  if (!mpOutRequest)
    return;

  enum gpiod_line_value v =
      (value ? GPIOD_LINE_VALUE_ACTIVE : GPIOD_LINE_VALUE_INACTIVE);

  if (gpiod_line_request_set_value(mpOutRequest, mPinEnable, v) < 0)
  {
    NXPLOG_TML_E("%s set VEN(pin %u)=%d failed: %s",
                 __func__, mPinEnable, value, strerror(errno));
  }
  usleep(10 * 1000);
}

void NfccAltTransport::gpio_set_fwdl(int value)
{
  if (!mpOutRequest)
    return;

  enum gpiod_line_value v =
      (value ? GPIOD_LINE_VALUE_ACTIVE : GPIOD_LINE_VALUE_INACTIVE);

  if (gpiod_line_request_set_value(mpOutRequest, mPinFwDnld, v) < 0)
  {
    NXPLOG_TML_E("%s set FWDL(pin %u)=%d failed: %s",
                 __func__, mPinFwDnld, value, strerror(errno));
  }
  usleep(10 * 1000);
}

/*******************************************************************************
**
** Function         GetIrqState
**
** Description      Return the current logical level of the IRQ / INT line.
**
** Parameters       pDevHandle – unused (kept for API compatibility)
**
** Returns          1  if IRQ is asserted (ACTIVE / high)
**                  0  if IRQ is de-asserted
**                 -1  on error
**
*******************************************************************************/
int NfccAltTransport::GetIrqState(void *pDevHandle)
{
  (void)pDevHandle;

  if (!mpIntRequest)
  {
    NXPLOG_TML_E("%s INT request not initialised", __func__);
    return -1;
  }

  enum gpiod_line_value val =
      gpiod_line_request_get_value(mpIntRequest, mPinInt);

  if (val == GPIOD_LINE_VALUE_ERROR)
  {
    NXPLOG_TML_E("%s gpiod_line_request_get_value failed: %s", __func__,
                 strerror(errno));
    return -1;
  }

  NXPLOG_TML_D("%s state=%d", __func__, (val == GPIOD_LINE_VALUE_ACTIVE));
  return (val == GPIOD_LINE_VALUE_ACTIVE) ? 1 : 0;
}

/*******************************************************************************
**
** Function         wait4interrupt
**
** Description      Block until the IRQ line goes high (ACTIVE).
**
**                  Strategy:
**                    1. If already asserted, return immediately.
**                    2. Otherwise poll(POLLIN) on the libgpiod request fd.
**                       The kernel queues an edge event each time a rising
**                       edge is detected; we must drain the event buffer
**                       after waking so the fd goes non-readable again.
**
*******************************************************************************/
void NfccAltTransport::wait4interrupt(void)
{
  if (!mpIntRequest)
  {
    NXPLOG_TML_E("%s INT request not initialised", __func__);
    return;
  }

  /* Fast path – IRQ already high */
  if (GetIrqState(nullptr) > 0)
    return;

  int irq_fd = gpiod_line_request_get_fd(mpIntRequest);
  if (irq_fd < 0)
  {
    NXPLOG_TML_E("%s gpiod_line_request_get_fd failed: %s", __func__,
                 strerror(errno));
    return;
  }

  struct pollfd fds = {.fd = irq_fd, .events = POLLIN};

  while (GetIrqState(nullptr) <= 0)
  {
    int ret = poll(&fds, 1, -1 /* block indefinitely */);
    if (ret < 0)
    {
      if (errno == EINTR)
        continue; /* signal – retry */
      NXPLOG_TML_E("%s poll() error: %s", __func__, strerror(errno));
      break;
    }

    if (ret > 0 && (fds.revents & POLLIN))
    {
      /*
       * Drain the event(s) that woke us up.  We must read them or the fd
       * stays readable and the next poll() returns immediately.
       */
      struct gpiod_edge_event_buffer *evbuf = gpiod_edge_event_buffer_new(8);
      if (evbuf)
      {
        /* Return value is the number of events read; errors are < 0 */
        gpiod_line_request_read_edge_events(mpIntRequest, evbuf, 8);
        gpiod_edge_event_buffer_free(evbuf);
      }
    }
  }
}

/* ── NfccReset ───────────────────────────────────────────────────────────── */

/*******************************************************************************
**
** Function         NfccReset
**
** Description      Reset NFCC device via VEN / FWDL GPIO lines.
**
** Parameters       pDevHandle – valid device handle
**                  eType      – reset mode
**
** Returns          0 on success, -1 on failure
**
*******************************************************************************/
int NfccAltTransport::NfccReset(void *pDevHandle, NfccResetType eType)
{
  NXPLOG_TML_D("%s VEN eType %ld", __func__, (long)eType);

  if (NULL == pDevHandle)
    return -1;

  switch (eType)
  {
  case MODE_POWER_OFF:
    gpio_set_fwdl(0);
    gpio_set_ven(0);
    break;
  case MODE_POWER_ON:
    gpio_set_fwdl(0);
    gpio_set_ven(1);
    break;
  case MODE_FW_DWNLD_WITH_VEN:
    gpio_set_fwdl(1);
    gpio_set_ven(0);
    gpio_set_ven(1);
    break;
  case MODE_FW_DWND_HIGH:
    gpio_set_fwdl(1);
    break;
  case MODE_POWER_RESET:
    gpio_set_ven(0);
    gpio_set_ven(1);
    break;
  case MODE_FW_GPIO_LOW:
    gpio_set_fwdl(0);
    break;
  default:
    NXPLOG_TML_E("%s unknown eType %ld", __func__, (long)eType);
    return -1;
  }

  if ((eType != MODE_FW_DWNLD_WITH_VEN) && (eType != MODE_FW_DWND_HIGH))
  {
    EnableFwDnldMode(false);
  }
  if ((eType == MODE_FW_DWNLD_WITH_VEN) || (eType == MODE_FW_DWND_HIGH))
  {
    EnableFwDnldMode(true);
  }

  return 0;
}

/* ── GetNfcState ─────────────────────────────────────────────────────────── */

/*******************************************************************************
**
** Function         GetNfcState
**
** Description      Query NFC controller state via IOCTL on the transport fd.
**
** Returns          NFC_STATE_UNKNOWN / NFC_STATE_FW_DWL / NFC_STATE_NCI
**
*******************************************************************************/
int NfccAltTransport::GetNfcState(void *pDevHandle)
{
  int ret = NFC_STATE_UNKNOWN;
  NXPLOG_TML_D("%s", __func__);
  if (NULL == pDevHandle)
    return ret;
  ret = ioctl((intptr_t)pDevHandle, NFC_GET_NFC_STATE);
  NXPLOG_TML_D("%s nfc state=%d", __func__, ret);
  return ret;
}

/* ── FW download mode flag ───────────────────────────────────────────────── */

void NfccAltTransport::EnableFwDnldMode(bool mode) { bFwDnldFlag = mode; }

bool_t NfccAltTransport::IsFwDnldModeEnabled(void) { return bFwDnldFlag; }

/* ── Semaphore helpers ───────────────────────────────────────────────────── */

/*******************************************************************************
**
** Function         SemPost
**
** Description      Post the TX/RX semaphore (only if count is 0).
**
*******************************************************************************/
void NfccAltTransport::SemPost()
{
  int sem_val = 0;
  sem_getvalue(&mTxRxSemaphore, &sem_val);
  if (sem_val == 0)
  {
    sem_post(&mTxRxSemaphore);
  }
}

/*******************************************************************************
**
** Function         SemTimedWait
**
** Description      Wait on the TX/RX semaphore with a 500 ms timeout.
**
** Returns          NFCSTATUS_SUCCESS or NFCSTATUS_FAILED
**
*******************************************************************************/
int NfccAltTransport::SemTimedWait()
{
  long sem_timedout_ns = 500L * 1000L * 1000L;
  int s;
  struct timespec ts;

  clock_gettime(CLOCK_REALTIME, &ts);
  ts.tv_nsec += sem_timedout_ns;
  if (ts.tv_nsec >= 1000000000L)
  {
    ts.tv_sec += 1;
    ts.tv_nsec -= 1000000000L;
  }

  while ((s = sem_timedwait(&mTxRxSemaphore, &ts)) == -1 && errno == EINTR)
  {
    continue; /* restart on signal */
  }

  if (s == 0)
    return NFCSTATUS_SUCCESS;

  if (errno == ETIMEDOUT)
  {
    NXPLOG_TML_E("%s timed out errno=0x%x", __func__, errno);
  }
  return NFCSTATUS_FAILED;
}

/* ── Flushdata ───────────────────────────────────────────────────────────── */

/*******************************************************************************
**
** Function         Flushdata
**
** Description      Read the payload of a FW response from the NFCC device.
**
** Parameters       pDevHandle – valid device handle
**                  pBuffer    – buffer (header already placed at offset 0)
**                  numRead    – bytes already read by the calling function
**
** Returns          Always -1 (caller uses SemPost side-effect).
**
*******************************************************************************/
int NfccAltTransport::Flushdata(void *pDevHandle, uint8_t *pBuffer,
                                int numRead)
{
  int retRead = 0;
  uint16_t totalBytesToRead =
      pBuffer[FW_DNLD_LEN_OFFSET] + FW_DNLD_HEADER_LEN + CRC_LEN;

  /* One byte already consumed by the caller, so read the remainder. */
  retRead =
      read((intptr_t)pDevHandle, pBuffer + numRead, totalBytesToRead - 1);

  if (retRead > 0)
  {
    numRead += retRead;
    phNxpNciHal_print_packet("RECV", pBuffer, numRead);
  }
  else if (retRead == 0)
  {
    NXPLOG_TML_E("%s read() [pyld] EOF", __func__);
  }
  else
  {
    if (!bFwDnldFlag)
    {
      NXPLOG_TML_D("%s read() [hdr] received", __func__);
      phNxpNciHal_print_packet("RECV", pBuffer - numRead,
                               NORMAL_MODE_HEADER_LEN);
    }
    NXPLOG_TML_E("%s read() [pyld] errno=0x%x", __func__, errno);
  }

  SemPost();
  return -1;
}