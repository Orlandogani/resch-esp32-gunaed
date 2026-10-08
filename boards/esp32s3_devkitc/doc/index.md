# Board: ESP32-S3-DevKitC-1 (N16R8) — development stand-in

The default board (`ADR-025`). Whatever is wired to the headers is provisional, so the pins are
Kconfig (`BOARD_DEVKIT_*` in [`board/Kconfig`](../board/Kconfig)), with the defaults the headset
used before boards existed:

| Function | GPIO (default) |
| --- | --- |
| Speaker I2S BCLK / WS / DOUT / MCLK | 15 / 16 / 17 / none |
| Microphone PDM CLK / DIN (I2S WS for a standard mic) | 4 / 6 (5) |
| Buttons: volume down / up / action (BOOT, deep-sleep wake) | 10 / 11 / 0 |

No codec, IMU, encoder, LED, battery, jack or power latch: the headset's modules see them as
absent and carry on (always worn, always on external power, host-side volume only).

**The only host link is the USB-Serial/JTAG on the USB connector** (CLAUDE.md hard rule 1):
light sleep, deep sleep and starting TinyUSB sever it. Images for other boards — in particular
`orga_v1`, which starts TinyUSB and drives GPIO21/10 — must never be flashed here.
