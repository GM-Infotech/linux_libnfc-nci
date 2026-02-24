# libnfc-nci Changelog

## Userspace GPIO Migration — sysfs → libgpiod

**Date:** 2026-02-23  
**Target:** Raspberry Pi (kernel 6.12, armv7l)  
**Dependency added:** `libgpiod >= 2.x` (`libgpiod-dev` package)

---

### Background

The alt transport option in libnfc-nci (`NXP_TRANSPORT=0x02` / `0x03`) uses
userspace GPIO drivers instead of the `nxpnfc` kernel driver. The original
implementation drove GPIO via the `/sys/class/gpio` pseudo-filesystem, which
was deprecated in kernel 5.x and removed from mainline in kernel 6.6. It is
absent from the target Raspberry Pi 6.12 kernel.

All GPIO handling has been migrated to the libgpiod 2.x API
(`gpiod_chip_open` / `gpiod_chip_request_lines`).

---

### Files Modified

#### `src/nfcandroid_nfc_hidlimpl/halimpl/tml/transport/NfccAltTransport.h`

- Removed three raw POSIX file descriptor members that pointed at sysfs GPIO
  value files:
  - `int iEnableFd`
  - `int iInterruptFd`
  - `int iFwDnldFd`
- Added libgpiod 2.x handle members:
  - `struct gpiod_chip *mpGpioChip`
  - `struct gpiod_line_request *mpIntRequest` — PIN_INT only (input + rising-edge detection)
  - `struct gpiod_line_request *mpOutRequest` — PIN_ENABLE + PIN_FWDNLD (outputs)
- Added runtime pin offset members, initialised to compiled-in defaults:
  - `unsigned int mPinInt`
  - `unsigned int mPinEnable`
  - `unsigned int mPinFwDnld`
- Added destructor `~NfccAltTransport()` to ensure GPIO resources are always
  released on teardown.
- Added `void ReleaseGpio()` as a **public** method (previously would have been
  private) so that subclasses `NfccAltI2cTransport` and `NfccAltSpiTransport`
  can call it from their `Close()` implementations.
- Removed `verifyPin()` declaration — replaced entirely by `ConfigurePin()`.
- Added `#include <gpiod.h>` and `#include <phNxpConfig.h>`.

---

#### `src/nfcandroid_nfc_hidlimpl/halimpl/tml/transport/NfccAltTransport.cc`

**`ConfigurePin()`** — complete rewrite:
- Removed the `verifyPin()` helper function, which was a manual
  reimplementation of GPIO export/direction/edge setup via sysfs file I/O.
- Now opens the GPIO chip via `gpiod_chip_open()`.
- Requests the interrupt line (PIN_INT) and output lines (PIN_ENABLE,
  PIN_FWDNLD) as two separate `gpiod_line_request` objects. Keeping them
  separate ensures the fd returned by `gpiod_line_request_get_fd()` on the
  interrupt request carries only edge events and can be `poll()`'d cleanly.
- Pin numbers and chip path are now resolved at runtime from `libnfc-nxp.conf`
  via `GetNxpStrValue()` / `GetNxpNumValue()` before any GPIO calls are made.
  Falls back to compiled-in defaults if a key is absent from the config.
- Resolved values are stored into `mPinInt`, `mPinEnable`, `mPinFwDnld`.

**`gpio_set_ven()` / `gpio_set_fwdl()`:**
- Replaced `write(fd, "0"/"1", 1)` on sysfs value files with
  `gpiod_line_request_set_value()`.
- Now reference `mPinEnable` / `mPinFwDnld` instead of the `PIN_ENABLE` /
  `PIN_FWDNLD` compile-time constants.

**`GetIrqState()`:**
- Replaced `lseek()` + `read()` + ASCII char comparison on the sysfs value
  file with `gpiod_line_request_get_value()`.
- Returns `GPIOD_LINE_VALUE_ERROR` (-1) on failure instead of relying on a
  read-length check.
- References `mPinInt` instead of `PIN_INT`.

**`wait4interrupt()`:**
- Replaced `POLLPRI` on the sysfs value file fd with `POLLIN` on the fd
  returned by `gpiod_line_request_get_fd()`. `POLLPRI` is specific to sysfs
  GPIO value files and has no effect on a character device.
- After `poll()` wakes up, `gpiod_line_request_read_edge_events()` is called
  to drain the kernel edge event queue. Without this drain the fd remains
  readable and subsequent `poll()` calls return immediately.

**`ReleaseGpio()`** — new method:
- Calls `gpiod_line_request_release()` on both requests and
  `gpiod_chip_close()` on the chip handle.
- Safe to call if `ConfigurePin()` was never called or failed partway through
  (all pointers are checked before use and reset to `nullptr`).
- Called automatically by `~NfccAltTransport()` and explicitly by subclass
  `Close()` implementations.

**`SemTimedWait()`:**
- Fixed a latent bug where `ts.tv_nsec` could exceed `999999999` after adding
  the 500ms timeout, resulting in `sem_timedwait()` returning `EINVAL`.
  Nanosecond overflow is now carried into `ts.tv_sec`.

---

#### `src/nfcandroid_nfc_hidlimpl/halimpl/tml/transport/NfccAltI2cTransport.cc`

**`Close()`:**
- Removed `close(iEnableFd)`, `close(iInterruptFd)`, `close(iFwDnldFd)`.
- Replaced with a single call to `ReleaseGpio()` inherited from
  `NfccAltTransport`.

---

#### `src/nfcandroid_nfc_hidlimpl/halimpl/tml/transport/NfccAltSpiTransport.cc`

**`Close()`:**
- Same change as `NfccAltI2cTransport.cc` above.

---

#### `src/nfcandroid_nfc_hidlimpl/halimpl/utils/phNxpConfig.h`

Added four new config key name defines inside the existing
`#if(NXP_EXTNS == TRUE)` block:

```c
#define NAME_NXP_GPIO_CHIP_PATH  "NXP_GPIO_CHIP_PATH"
#define NAME_NXP_GPIO_INT        "NXP_GPIO_INT"
#define NAME_NXP_GPIO_VEN        "NXP_GPIO_VEN"
#define NAME_NXP_GPIO_FWDNLD     "NXP_GPIO_FWDNLD"
```

---

#### `conf/libnfc-nxp.conf`

Added a new GPIO configuration section at the end of the file:

```properties
###############################################################################
# GPIO configuration for the alt (userspace) transport (NXP_TRANSPORT=0x02/0x03)
#
# NXP_GPIO_CHIP_PATH  - gpiochip device node.
#                       Raspberry Pi 1-4: /dev/gpiochip0
#                       Raspberry Pi 5:   /dev/gpiochip4
# NXP_GPIO_INT        - BCM GPIO number for the NFCC interrupt (IRQ) line.
# NXP_GPIO_VEN        - BCM GPIO number for the VEN (enable) line.
# NXP_GPIO_FWDNLD     - BCM GPIO number for the firmware-download mode line.
#
# If any of these keys are absent the compiled-in defaults are used
# (gpiochip0, INT=23, VEN=24, FWDNLD=25).
###############################################################################
NXP_GPIO_CHIP_PATH="/dev/gpiochip0"
NXP_GPIO_INT=23
NXP_GPIO_VEN=24
NXP_GPIO_FWDNLD=25
```

All four keys are optional. If absent, the compiled-in defaults from
`NfccAltTransport.h` (`GPIO_CHIP_PATH`, `PIN_INT`, `PIN_ENABLE`, `PIN_FWDNLD`)
are used. Config changes take effect on the next launch of `nfcDemoApp` — there
is no daemon to restart.

---

#### `Makefile.am`

- Removed `libnfc_nci_linux_la_FLAGS += -lgpiod` (line 256). This variable
  feeds into `AM_CPPFLAGS` (the compiler), not the linker. Passing `-lgpiod`
  to the compiler is a no-op at best.
- `-lgpiod` is correctly retained in:
  - `libnfc_nci_linux_la_LDFLAGS` — links the shared library
  - `nfcDemoApp_LDFLAGS` — links the demo application

No source file additions were required — `NfccAltTransport.cc` was already
listed in `HALIMPL_SOURCE` and `<gpiod.h>` installs to the standard system
include path from the `libgpiod-dev` package.

---

### Runtime Behaviour Notes

- GPIO chip path and pin numbers are resolved once at startup inside
  `ConfigurePin()` and cached. Editing `libnfc-nxp.conf` while `nfcDemoApp`
  is running has no effect until the application is restarted.
- libgpiod enforces **exclusive** ownership of each GPIO line. If another
  process has already requested a line, `gpiod_chip_request_lines()` will fail.
- On RPi 1–4 the main GPIO bank is `/dev/gpiochip0`. On RPi 5 it moved to
  `/dev/gpiochip4`. Use `gpiodetect` to confirm on the target board.
- The GPIO lines are released automatically when the process exits via
  `~NfccAltTransport()` → `ReleaseGpio()`. Unlike sysfs, libgpiod does not
  leave pin directions persisted across process lifetimes beyond what the
  kernel's internal state retains — do not rely on pin state surviving a
  restart of `nfcDemoApp`.