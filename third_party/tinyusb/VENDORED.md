# Vendored: `espressif/tinyusb` (TinyUSB device stack)

| Field | Value |
| --- | --- |
| Upstream | https://github.com/hathach/tinyusb — packaged by Espressif as `espressif/tinyusb` |
| Registry | https://components.espressif.com/components/espressif/tinyusb |
| Version vendored | **0.21.0~2** — this table is authoritative; upstream `version.yml` is a Mynewt artefact that always reads 0.0.0 |
| Vendored on | 2026-09-20 |
| Decision | `ADR-017` in [docs/architecture/SAD.md](../../docs/architecture/SAD.md) |
| License | MIT (see `LICENSE`, unchanged) |

## Why this is vendored

`third_party/esp_tinyusb` is vendored and patched (see its `VENDORED.md`). If TinyUSB
itself stayed a registry dependency, the IDF component manager would download it and
**force it into every build that merely discovers the SDK trees** — including test projects
and applications that do not use USB — where it fails to compile because nothing supplies
`tusb_config.h`. Vendoring the stack makes it an ordinary component that `MINIMAL_BUILD`
includes only when something `REQUIRES` it, and removes the SDK's last network dependency
at build time (`SYS-BLD-002`).

## What was kept

Only what the ESP-IDF component build needs:

| Path | Why |
| --- | --- |
| `src/` | The stack. All of it; the CMakeLists selects sources by target. |
| `lib/networking/` | `rndis_reports.c` is in the unconditional source list. |
| `CMakeLists.txt`, `LICENSE`, `version.yml` | Unchanged from upstream. |

Dropped: `hw/` (14 MB of other vendors' BSPs), `examples/`, `test/`, docs, CI metadata.

## Patches

**None.** This directory is byte-identical to upstream for the files kept.

## How to upgrade

1. Download the new `espressif/tinyusb` version from the registry.
2. Replace `src/`, `lib/networking/`, `CMakeLists.txt`, `LICENSE`, `version.yml`.
3. Update the version table above and the pin comment in `third_party/esp_tinyusb/VENDORED.md`.
4. Rebuild and run `tests/subsys/usb_device` on hardware — the audio class API has changed
   between minor versions before (e.g. `tud_audio_tx_done_pre_load_cb` → `tud_audio_tx_done_isr`).
