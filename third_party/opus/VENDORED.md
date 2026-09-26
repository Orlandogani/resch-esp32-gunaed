# Vendored: libopus (Opus audio codec)

| Field | Value |
| --- | --- |
| Upstream | https://github.com/xiph/opus |
| Version vendored | **1.5.2** — tag `v1.5.2`, commit `ddbe48383984d56acd9e1ab6a090c54ca6b735a6` (2024-04-10) |
| Vendored on | 2026-09-26 |
| Decision | `ADR-024` in [docs/architecture/SAD.md](../../docs/architecture/SAD.md) |
| License | BSD 3-clause (see `COPYING`, unchanged), with the royalty-free patent grants referenced there |
| Used by | `subsys/audio_link` only, privately (`PRIV_REQUIRES opus`) |

## Why Opus, and why vendored

`subsys/audio_link` needs a codec: 48 kHz stereo PCM is 1.5 Mbit/s, which BLE cannot
carry, and the product line is commercial. Opus is standardised (RFC 6716), royalty-free
under its patent grants, has a 2.5 ms low-delay mode, and has packet-loss concealment
built in. LC3's royalty-free grant is tied to Bluetooth LE Audio use; using it over a
proprietary link needs a licensing check first (`ADR-024`).

It is vendored rather than taken from the component registry for the same reason as
TinyUSB (`ADR-017`): no network at build time (`SYS-BLD-002`), and `MINIMAL_BUILD`
compiles it only into images that use the link. It is **source**, not Espressif's
prebuilt `esp_audio_codec`, so the whole codec is auditable and carries only its own
license.

## Build configuration

Upstream builds through autotools/CMake/meson and a generated `config.h`. This
directory's `CMakeLists.txt` replaces all three with what the sources actually test:

| Definition | Why |
| --- | --- |
| `FIXED_POINT=1` | Integer DSP throughout; PCM in and out is `int16_t`. |
| `DISABLE_FLOAT_API` | Only `opus_encode()` / `opus_decode()`; no `*_float` entry points. |
| `VAR_ARRAYS` | Scratch on the caller's stack (C99 VLAs), not a global pseudo-stack, so encoder and decoder may run on different tasks. The link task's stack is sized for it (`CONFIG_AUDIO_LINK_TASK_STACK`). |
| `HAVE_LRINT`, `HAVE_LRINTF` | newlib provides both. |
| `-O2` | The codec is the per-frame hot path, whatever the project's optimisation level. |

Suppressed warnings, and nothing else: `-Wno-unused-variable` (`src/repacketizer.c`, a
variable read only under an assert) and `-Wno-maybe-uninitialized` (a GCC 15 false
positive through `celt/celt_lpc.c`'s variable-length arrays).

Not enabled: `ENABLE_DEEP_PLC`, `ENABLE_DRED`, `ENABLE_OSCE` (the 1.5 neural
extensions — they need the `dnn/` tree and model weights, and the RAM they want does
not exist here), `CUSTOM_MODES`, and every architecture-specific path (x86, ARM, MIPS);
the Xtensa LX7 has none.

## What was kept

| Path | Why |
| --- | --- |
| `include/` | Public headers. |
| `celt/*.c`, `celt/*.h` | Top level only. `celt/arm`, `celt/x86`, `celt/mips`, `celt/tests`, `celt/dump_modes`, `celt/opus_custom_demo.c` dropped. |
| `silk/*.c`, `silk/*.h`, `silk/fixed/*.c`, `silk/fixed/*.h` | SILK is linked even in CELT-only use: `opus_encoder_init()` initialises it. `silk/float`, `silk/arm`, `silk/x86`, `silk/mips`, `silk/tests` dropped. |
| `src/opus.c`, `opus_decoder.c`, `opus_encoder.c`, `extensions.c`, `repacketizer.c`, `src/*.h` | The single-stream API. Multistream, projection, analysis (float only) and the demos dropped. |
| `COPYING`, `AUTHORS` | License and attribution, unchanged. |

Dropped entirely: `dnn/` (11 MB), `doc/`, `tests/`, `training/`, `m4/`, `meson/`, `cmake/`,
and the autotools files.

## Cost (measured from the `tests/subsys/audio_link` build, 2026-09-26)

`libopus.a`: 158,660 B flash code + 22,462 B flash rodata, **0 B static RAM**. The
encoder and decoder state are heap allocations made by `audio_link_init()`; scratch is the
link task's stack.

Measured on target (ESP32-S3, 240 MHz, 32 KiB I-cache / 64 KiB D-cache, 10 ms frames,
2026-09-26):

| | Complexity 5 | Complexity 1 (default) |
| --- | --- | --- |
| 48 kHz stereo encode, 96 kbit/s | 7.1 ms | 4.86 ms |
| 48 kHz stereo decode | 3.2 ms | 3.17 ms |
| 16 kHz mono encode / decode, 24 kbit/s | — | 2.40 / 1.41 ms |
| Peak scratch (stack), stereo encode + decode | 26.3 KiB | 21.9 KiB |
| Codec state, encoder + decoder | 55,812 B stereo · 42,348 B mono | same |

At the IDF default 16 KiB I-cache and 160 MHz, complexity 5 stereo encode was 10.4 ms — not
real time. The code runs from flash, so cache size matters as much as clock.

## Patches

**None.** Every file kept is byte-identical to the tag.

## How to upgrade

1. `git clone --depth 1 --branch vX.Y.Z https://github.com/xiph/opus.git`.
2. Replace the kept paths above; keep the drop list.
3. Regenerate the source lists in `CMakeLists.txt` from upstream's `celt_sources.mk`,
   `silk_sources.mk` (`SILK_SOURCES` + `SILK_SOURCES_FIXED`) and `opus_sources.mk`
   (without multistream/projection), leaving out architecture-specific entries.
4. Build `tests/subsys/audio_link` warning-free and run it on target: the round-trip,
   loss and benchmark cases are the regression suite for the codec.
5. Update the version, commit and date in this file.
