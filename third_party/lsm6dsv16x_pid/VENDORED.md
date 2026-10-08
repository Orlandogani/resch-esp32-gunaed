# Vendored: ST LSM6DSV16X platform-independent driver

| Field | Value |
| --- | --- |
| Upstream | https://github.com/STMicroelectronics/lsm6dsv16x-pid |
| Version vendored | `main`, commit `2808e5cd6b85f91b66758e1dd0faab5f043aba07` (2026-08-25) |
| Vendored on | 2026-10-02 |
| Decision | `ADR-027` in [docs/architecture/SAD.md](../../docs/architecture/SAD.md) |
| License | BSD 3-clause (see `LICENSE`, unchanged) |
| Used by | `drivers/lsm6dsv16x` only, privately (`PRIV_REQUIRES lsm6dsv16x_pid`) |

## What and why

`lsm6dsv16x_reg.c` / `lsm6dsv16x_reg.h` are ST's register-level driver for the
LSM6DSV16X: every register as a bit-field struct, and one function per feature
(`*_set` / `*_get`). It has no platform dependency — the caller supplies read, write and
delay functions in a `stmdev_ctx_t`. The register map is about 8 000 lines and the
embedded functions (SFLP sensor fusion, activity/inactivity) are configured through
multi-register sequences with page switching; re-deriving that from the datasheet would
be a worse source of truth than the manufacturer's own code. `drivers/lsm6dsv16x` binds it
to ESP-IDF's I2C master driver and adds the SDK lifecycle.

Vendored rather than fetched for the reasons of `ADR-017`: no network at build time
(`SYS-BLD-002`), and `MINIMAL_BUILD` compiles it only into images that use the IMU.

## What was kept

| File | Why |
| --- | --- |
| `lsm6dsv16x_reg.c`, `lsm6dsv16x_reg.h` | The driver. |
| `LICENSE` | Required by the license. |

Dropped: `README.md`, `Release_Notes.*`, `_htmresc/`, the GitHub community files.

## Local changes

None. The directory is `lsm6dsv16x_pid` (ST's repository name) rather than `lsm6dsv16x`
because ESP-IDF names a component after its directory, and `drivers/lsm6dsv16x` already
has that name.

## Updating

Copy the two source files and `LICENSE` from a new upstream commit, update the commit in
this file, build `tests/drivers/lsm6dsv16x`, and re-run its `[needs_orga]` cases on a board.
