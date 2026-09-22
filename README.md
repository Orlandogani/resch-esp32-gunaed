# ESP32-S3 Platform SDK

A product-independent firmware platform for the ESP32-S3 on ESP-IDF v6.1: USB composite
device (UAC2 microphone + HID at 1 ms), I2S/PDM audio capture into a multi-reader ring,
Bluetooth LE peripheral (Bluedroid), Wi-Fi station, HTTPS OTA with rollback, deep-sleep
arbitration, diagnostics, and NVS-backed configuration. Laid out Zephyr-style:

```
applications/<product>/   product firmware (policy) — applications/gunshot-aed is the reference
subsys/                   services and protocols
drivers/                  hardware-facing code
lib/                      pure logic, host-testable
third_party/              vendored upstream (TinyUSB) — see each VENDORED.md
tests/{lib,drivers,subsys}/<name>/   one on-target Unity project per component
docs/                     requirements → architecture → design, with traceability
```

Start with [docs/README.md](docs/README.md). Every public header documents its contract
(thread-safety, ISR-safety, blocking, every return code).

## Build

Requires ESP-IDF v6.1. On this machine the environment is activated with:

```powershell
. "C:\Espressif\tools\Microsoft.v6.1.PowerShell_profile.ps1"
```

```powershell
cd applications/gunshot-aed
idf.py set-target esp32s3        # first time only
idf.py build
idf.py -p COM4 flash monitor     # the board enumerates as COM4 (USB-Serial/JTAG)
```

Configuration lives in `sdkconfig.defaults` (version-controlled); the generated `sdkconfig`
is ignored. To apply a defaults change: `idf.py fullclean && idf.py build`.

`CONFIG_APP_ENABLE_USB` is **off by default**. On the ESP32-S3 the USB-OTG device and the
built-in USB-Serial/JTAG share the same pins: starting the USB stack removes the console
until the board is physically re-plugged. Turn it on only with a separate UART console or
for production images. The same applies to light and deep sleep — see `CON-09` in the SyRS.

## Test

Each component has its own project. Build, flash, and read the Unity summary:

```powershell
cd tests/subsys/pm_policy
idf.py build
idf.py -p COM4 flash monitor
```

Projects: `tests/lib/ringbuf`, `tests/drivers/{audio_capture, audio_playback}`, `tests/subsys/{power,
pm_policy, cfg, diag, usb_device, wifi_link, ota, ble}`. Tests that would sever a
USB-Serial/JTAG console are tagged `[needs_uart_console]` / `[needs_usb_host]` and skip
themselves or are excluded by the project's `app_main`.

CI (`.github/workflows/build.yml`) builds every project on every push and enforces the
release gate from ADR-016.

## Adding a component

1. Create `subsys/<name>/` (or `drivers/`, `lib/`) with `CMakeLists.txt`, `Kconfig`,
   `include/<name>.h`, `<name>.c`; end the CMakeLists with `sdk_component_strict()`.
2. Write the `FW-*` requirements and `DES-*` elements first (docs/README.md §6).
3. Add `tests/<tree>/<name>/` and list it in the CI matrix.
4. Check the SAD §10 conformance checklist before merging.
