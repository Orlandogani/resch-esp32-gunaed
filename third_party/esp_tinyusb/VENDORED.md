# Vendored: `espressif/esp_tinyusb`

| Field | Value |
| --- | --- |
| Upstream | https://github.com/espressif/esp-usb/tree/master/device/esp_tinyusb |
| Registry | https://components.espressif.com/components/espressif/esp_tinyusb |
| Version vendored | **2.3.0** (upstream commit `0a1e5fd0f7f9d3c58a0d4949b5d1bfbf23af8d96`) |
| Vendored on | 2026-09-20 |
| Decision | `ADR-017` in [docs/architecture/SAD.md](../../docs/architecture/SAD.md) |
| License | MIT (see `LICENSE`, unchanged) |

## Why this is vendored

Upstream `esp_tinyusb` generates TinyUSB's `tusb_config.h` from Kconfig and exposes CDC,
MSC, MTP, HID, MIDI, DFU, BTH, NET and VENDOR classes — **but not the audio class**.
`CFG_TUD_AUDIO` is never defined, so TinyUSB's UAC driver is never compiled, and there is
no supported hook to add it from outside the component.

The SDK requires a composite UAC + HID device on one port (`SYS-USB-002`), and for the
headset product a *duplex* UAC2 function with an explicit feedback endpoint (`ADR-021`).
TinyUSB supports all of that; upstream `esp_tinyusb` simply never exposes it. Rather than
depend on ESP-IDF's private `esp_private/usb_phy.h` API to drive raw TinyUSB ourselves,
this copy of the thin wrapper is carried in-tree with a minimal patch. The TinyUSB stack
itself is vendored alongside at `third_party/tinyusb` — see that directory's `VENDORED.md`
for why a registry dependency was not viable.

## Patch summary

Every change is marked with a `PATCHED (ADR-017)` comment in the file. Nothing else differs
from upstream 2.3.0 except the removal of `test_apps/` and `idf_component.yml`.

| File | Change |
| --- | --- |
| `idf_component.yml` | **Removed.** TinyUSB is vendored at `third_party/tinyusb` (0.21.0~2); the SDK has no registry dependencies. |
| `CMakeLists.txt` | `tinyusb` is always appended to `REQUIRES` (upstream only did so when the component manager was disabled). |
| `Kconfig` | New menu `USB Audio Class (UAC) [SDK patch]` with `TINYUSB_AUDIO_ENABLED`, `TINYUSB_AUDIO_EP_IN_SZ_MAX`, `TINYUSB_AUDIO_EP_IN_SW_BUF_MULT`, `TINYUSB_AUDIO_CTRL_BUF_SZ`, and (ADR-021) `TINYUSB_AUDIO_SPEAKER_ENABLED`, `TINYUSB_AUDIO_EP_OUT_SZ_MAX`, `TINYUSB_AUDIO_EP_OUT_SW_BUF_MULT`. |
| `include/tusb_config.h` | `CFG_TUD_AUDIO` and the `CFG_TUD_AUDIO_FUNC_1_*` / `CFG_TUD_AUDIO_ENABLE_EP_IN` macros, derived from the new Kconfig symbols; IN flow control on. Under `TINYUSB_AUDIO_SPEAKER_ENABLED` also `CFG_TUD_AUDIO_ENABLE_EP_OUT`, `CFG_TUD_AUDIO_ENABLE_FEEDBACK_EP` and the OUT sizes (ADR-021). |
| `test_apps/` | Removed. |

## How to upgrade

1. Download the new upstream version from the registry (or clone `esp-usb` at the tag).
2. Diff it against this directory **ignoring** lines marked `PATCHED`, to see upstream changes.
3. Replace this directory with the new upstream copy, delete `test_apps/`.
4. Re-apply the patches above by hand (they are small and localised), or check
   whether upstream has since added audio support — if so, drop the patch and consider
   returning to the registry dependency, superseding `ADR-017`.
5. Update the version table at the top of this file.
6. Rebuild `applications/gunshot-aed` and run `tests/subsys/usb_device` on hardware, in
   **both** configurations - the default microphone one and the duplex-headset variant
   (`sdkconfig.headset.defaults`), which is the only build that exercises the OUT path
   and the feedback endpoint.

## What depends on this

- `subsys/usb_device` — the only SDK component that calls `tinyusb_driver_install()`.
- `subsys/usb_hid`, `subsys/usb_audio` — implement TinyUSB class callbacks; they never call
  `esp_tinyusb` directly (`ADR-007`).
