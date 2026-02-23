/******************************************************************************
 *
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

#pragma once
#include <NfccTransport.h>
#include <gpiod.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <poll.h>

/* ── I2C / SPI bus configuration ─────────────────────────────────────────── */
#define I2C_ADDRESS 0x28
#define I2C_BUS "/dev/i2c-1"
#define SPI_BUS "/dev/spidev0.0"

/*
 * GPIO chip device.
 * On Raspberry Pi 1-4 all BCM GPIOs live on gpiochip0.
 * RPi 5 moved the main bank to gpiochip4 – override at build time if needed:
 *   -DGPIO_CHIP_PATH=\"/dev/gpiochip4\"
 */
#ifndef GPIO_CHIP_PATH
#define GPIO_CHIP_PATH "/dev/gpiochip0"
#endif

/* ── BCM GPIO pin numbers (= offsets on gpiochip0) ───────────────────────── */
#define PIN_INT 23    /* IRQ  – input, rising-edge */
#define PIN_ENABLE 24 /* VEN  – output             */
#define PIN_FWDNLD 25 /* FWDL – output             */

/* Edge constants kept for source-level compatibility with any call sites */
#define EDGE_NONE 0
#define EDGE_RISING 1
#define EDGE_FALLING 2
#define EDGE_BOTH 3

/* ── Protocol framing constants ──────────────────────────────────────────── */
#define CRC_LEN 2
#define NORMAL_MODE_HEADER_LEN 3
#define FW_DNLD_HEADER_LEN 2
#define FW_DNLD_LEN_OFFSET 1
#define NORMAL_MODE_LEN_OFFSET 2
#define FRAGMENTSIZE_MAX PHNFC_I2C_FRAGMENT_SIZE

extern phTmlNfc_i2cfragmentation_t fragmentation_enabled;

/* ══════════════════════════════════════════════════════════════════════════ */

class NfccAltTransport : public NfccTransport
{
public:
  NfccAltTransport();
  ~NfccAltTransport();

  bool_t bFwDnldFlag = false;
  sem_t mTxRxSemaphore;

private:
  /* ── libgpiod v2 handles ───────────────────────────────────────────────── */
  struct gpiod_chip *mpGpioChip;

  /*
   * Two separate requests:
   *   mpIntRequest – PIN_INT only (input + rising-edge detection)
   *   mpOutRequest – PIN_ENABLE + PIN_FWDNLD (outputs, combined so both values
   *                  can be driven atomically if ever required)
   */
  struct gpiod_line_request *mpIntRequest;
  struct gpiod_line_request *mpOutRequest;

public:
  /*
   * Release all libgpiod resources.
   * Must be called from subclass Close() implementations instead of
   * close(iEnableFd) / close(iInterruptFd) / close(iFwDnldFd).
   * Also called automatically by ~NfccAltTransport().
   */
  void ReleaseGpio();

  /* ── GPIO output helpers ──────────────────────────────────────────────── */
  void gpio_set_ven(int value);
  void gpio_set_fwdl(int value);
  void wait4interrupt(void);

  /* ── Semaphore helpers ───────────────────────────────────────────────── */
  int SemTimedWait();
  void SemPost();

  /* ── NfccTransport overrides ─────────────────────────────────────────── */
  int Flushdata(void *pDevHandle, uint8_t *pBuffer, int numRead);
  int NfccReset(void *pDevHandle, NfccResetType eType);
  void EnableFwDnldMode(bool mode);
  bool_t IsFwDnldModeEnabled(void);
  int GetIrqState(void *pDevHandle);
  int GetNfcState(void *pDevHandle);
  int ConfigurePin();
};