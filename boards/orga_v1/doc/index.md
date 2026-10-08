# Board: ORGA v1 — headset main board (left cup)

ESP32-S3-WROOM-1-N16R8 headset board. The hardware design (KiCad) is in
`C:\ORLANDO\Lab\kicad\headset-orga\orga-v1\`; its netlist source is
`generator/design.py`, its datasheets are in `datasheets/`, and its status page is mirrored at
[`docs/orga_v1_kicad_status.md`](../../../docs/orga_v1_kicad_status.md).

Firmware: [`board/board.c`](../board/board.c) (the description), [`sdkconfig.defaults`](../sdkconfig.defaults)
(board Kconfig), [`bootloader_components/orga_boot_hooks`](../bootloader_components/orga_boot_hooks/)
(PWR_HOLD at power-on). Select it with `-D BOARD=orga_v1` (`ADR-025`).

**State, 2026-10-02:** the PCB is placed but unrouted. Every line of firmware for it is
build-verified only.

---

## 1. Pin map

Module pin → GPIO from the WROOM-1 datasheet pin table; net from `design.py`.

| GPIO | Module pin | Net | Part / function | Firmware |
| --- | --- | --- | --- | --- |
| 1 | 39 | BAT_ADC | 1 MΩ / 1 MΩ divider from +BATT, 100 nF (ADC1_CH0) | `drivers/battery` |
| 2 | 38 | DBG_TIMING | Test point | `board_init()` drives low |
| 4 | 4 | IO4_MCLK → CODEC_MCLK | 22 Ω, 0 Ω option; 12.288 MHz | `audio_playback` (MCLK) |
| 5 | 5 | I2S_BCLK | 22 Ω | `audio_playback` (TX), `audio_capture` (RX, slave) |
| 6 | 6 | I2S_WS | 22 Ω | as BCLK |
| 7 | 7 | I2S_DOUT → codec DIN | 22 Ω | `audio_playback` |
| 15 | 8 | I2S_DIN ← codec DOUT | 22 Ω at the codec | `audio_capture` (boom mic) |
| 16 | 9 | PDM_CLK | 22 Ω; MK1–MK3 | `audio_capture` (PDM, I2S0) |
| 17 | 10 | PDM_DIN0 | MK1 voice (LR = GND → left), MK2 reference (LR = VDD → right) | `audio_capture` |
| 18 | 11 | PDM_DIN1 | MK3 inner cup (not fitted) | — |
| 8 | 12 | I2C_SDA | 4.7 kΩ pull-up; codec 0x18, IMU 0x6A | `board_i2c_bus()` |
| 9 | 17 | I2C_SCL | 4.7 kΩ pull-up | `board_i2c_bus()` |
| 10 | 18 | CODEC_RST | 100 kΩ pull-down (codec held in reset until released) | `drivers/tlv320aic3104` |
| 11 | 19 | IMU_INT | LSM6DSV16X INT1 | `drivers/lsm6dsv16x` |
| 12 | 20 | JACK_DET | 100 kΩ pull-up; ring switch to GND opens on insertion → 1 = plugged | `drivers/buttons` (as an input) |
| 13 | 21 | ENC_A | 10 kΩ pull-up, 10 nF | `drivers/encoder` |
| 14 | 22 | ENC_B | 10 kΩ pull-up, 10 nF | `drivers/encoder` |
| 19 / 20 | 13 / 14 | USB_DN / USB_DP | USB-C via USBLC6-2 | TinyUSB (`usound`) or USB-Serial/JTAG |
| 21 | 23 | PWR_HOLD | 100 kΩ pull-down, BAT54C into TPS63001 EN | `drivers/power_latch`, bootloader hook |
| 38 / 39 / 40 | 31 / 32 / 33 | LED_R / LED_G / LED_B | Common-anode RGB at 3V3; 470 Ω (R), 68 Ω (G, B) | `drivers/rgb_led` (active low) |
| 41 | 34 | BTN_PWR | Q1 (SI2302) pulls low while POWER pressed; 10 kΩ pull-up | `drivers/buttons` (role POWER) |
| 42 | 35 | BTN_MFB | MFB switch to GND; 10 kΩ pull-up, 100 nF | `drivers/buttons` (role ACTION) |
| 47 | 24 | CHG_PG | BQ24074 PGOOD, open drain, 10 kΩ to 3V3; low = input power good | `drivers/battery` |
| 48 | 25 | SPARE_IO48 | Test point | — |
| 43 / 44 | 37 / 36 | UART_TX / UART_RX | Programming pads | Console (UART0) |
| 0 | 27 | ESP_IO0 | BOOT button, 10 kΩ pull-up | ROM download |
| 3, 45, 46 | 15, 26, 16 | — | Strapping pins, not connected (defaults) | — |
| 35–37 | 28–30 | — | Octal PSRAM | — |

## 2. Parts and how firmware drives them

| Ref | Part | Firmware |
| --- | --- | --- |
| U7 | TLV320AIC3104 | I2S slave; MCLK 12.288 MHz → fS 48 kHz (Q = 2, PLL off); DAC → HPLOUT/HPROUT with HPxCOM inverted (speakers floating between them — **never ground a COM or a speaker wire**); MIC1LP single-ended + MICBIAS 2.5 V for the boom; DAC volume = host volume with a hearing-safety floor |
| MK1/MK2 (MK3 DNP) | IM73D122V01 PDM | I2S0 PDM RX, mono left (voice). Clock bands 0.45–0.85, 1.2–1.65, 2.0–2.6, 2.9–3.3 MHz → 16 kHz needs ×128 (2.048 MHz); 48 kHz would use ×64 (3.072 MHz). `board_mic_pdm_oversample()` chooses |
| U8 | LSM6DSV16X | I2C 0x6A (SA0 = GND), INT1 latched; motion engine always on, SFLP game rotation on demand |
| U2 | BQ24074 | Autonomous charger (USB500, 0.49 A). Firmware sees PGOOD only (`TBD-015`) |
| U3 + latch | TPS63001 + BAT54C + SI2302 | `power_latch` holds; POWER long press (2.5 s) powers off after release |
| D6 | Würth 150141M173100 | LEDC PWM, active low; red gain 600 ‰ to start (`TBD-017`) |
| SW3 | PEC09 placeholder | PCNT ×4, 4 counts per detent; sense set by `HS_UI_WHEEL_REVERSE` until the part is chosen |
| J3 | SJ2-3593D boom jack | Jack-detect switches the headset's microphone to the codec ADC live |

## 3. Review findings (2026-10-02)

Checked against the netlist and the datasheets in `datasheets/`.

1. **PDM clock band** — a firmware bug in the SDK as it stood: ESP-IDF's default is 64 × fs,
   i.e. 1.024 MHz at 16 kHz, outside every IM73D122 band. Fixed: `audio_capture` gained
   `pdm_oversample`, and this board selects ×128 (`FW-AUD-071`).
2. **Any reset cuts power** unless PWR_HOLD is latched (100 kΩ pull-down). Mitigated by the pad
   hold and the bootloader hook; whether the hold survives an **esptool reset over USB** is a
   first-board check (`R14`). Fallback: hold POWER while flashing, or feed 3V3 at the pad.
3. **USB plug-in cannot power the device on** — see §5.
4. **Charging vs. full is invisible** to the firmware (`TBD-015`); inferred from PGOOD + voltage.
5. **TS must not float**: fit the not-fitted 10 kΩ if the cell has no NTC (`TBD-016`).
6. **POWER and MFB are not RTC pins** (GPIO41/42): no deep-sleep wake from them. Not needed:
   "off" is the latch.
7. **Codec ADC and DAC share WCLK** (no GPIO1 on the RHB package; SLAS510G register 2 note:
   ADC fs must equal DAC fs). The boom microphone therefore arrives at 48 kHz and is decimated
   to 16 kHz in `audio_capture` (`lib/decimator`).
8. **MCLK from the S3's fractional divider** (no APLL): 160 MHz / 12.288 MHz, so some jitter.
   The not-fitted 12.288 MHz oscillator (Y1, with the 0 Ω moved) is the escape. The codec stays
   an I2S slave on BCLK/WCLK either way; the only firmware change is `spk.mclk = BOARD_PIN_NONE`
   in `board.c`, so the S3 stops driving MCLK into the oscillator's output.
9. **Codec RESET has no capacitor.** SLAS510G §10.3.1 recommends ≥ 1 nF from RESET to DVSS
   against ESD-induced resets. Worth adding in the layout.
10. Every IC pin checked matches (BQ24074 pins 14/15 are TMR/ITERM and may float; IMU pins
    10/11 may float; mic L/R wiring matches stereo mode).

## 4. Bring-up

1. Power from a bench supply on +BATT (current-limited, 3.8 V), USB unplugged. Hold POWER:
   3V3 must come up and **stay up after release** once the bootloader runs (PWR_HOLD on GPIO21).
2. USB-UART adapter on the programming pads (TX, RX, GND; EN/IO0 for auto-reset).
   `idf.py -C tests/boards/orga_v1 -p <uart> flash monitor`.
3. In the console, in this order:
   - `i2c_scan` → 0x18 and 0x6A.
   - `codec init`, `codec regs` → r3 = 0x10, r7 = 0x0A, r101 = 0x01.
   - `tone 1000 3` with a driver connected → `r94` should read `0xC6`, `r95` = 0.
   - `mic pdm 3` (speak: levels rise above the floor), `mic boom 3` with the boom plugged.
   - `imu accel 2` (\|g\| ≈ 1000 mg), `imu pose 3` (unit quaternions, ~30 Hz).
   - `led 255 0 0`, `led 0 255 0`, `led 0 0 255` — tune gains if green/blue are dim (`TBD-017`).
   - `enc 10` (turn), `btn 10` (POWER, MFB, jack).
   - `bat` against a meter on +BATT.
   - `latch off` with POWER released → the board switches off.
4. Then the Unity suites with `CONFIG_TEST_ON_ORGA=y` (`tests/drivers/tlv320aic3104`,
   `tests/drivers/lsm6dsv16x`), and the headset: `idf.py -C applications/headset -B build_orga
   -D BOARD=orga_v1 -p <port> flash`.

**Never flash an ORGA image to the devkit**: it drives GPIO21/10, and the headset image starts
TinyUSB, which severs the devkit's only host link (CLAUDE.md hard rule 1).

## 5. Option: power on when USB is plugged in (`TBD-013`)

**Decision for v1 (2026-10-02): button-only.** Recorded here so it can be changed for v2.

Today the TPS63001 enable (BB_EN) is a diode-OR of the POWER button (through BAT54C pin 1) and
PWR_HOLD (pin 2), with a 1 MΩ pull-down. Plugging in USB powers the BQ24074 and charges the
cell, but nothing raises BB_EN: the ESP stays off and the PC sees no device.

To make a USB plug power the headset on, add a **third input to the OR** from the USB side:

- **From PGOOD (recommended):** PGOOD is open-drain, active low, so it needs inverting — a
  small P-FET or an NPN/NMOS inverter from VSYS, whose output feeds a third diode (a second
  BAT54C, or BAT54A/S variant) into BB_EN. PGOOD only asserts for a valid input (4.35–10.5 V),
  so a bad cable does not wake the board.
- **From VBUS directly:** a resistor divider from VBUS (5 V → ≤ 3.6 V at BB_EN, respecting the
  TPS63001 EN limit) through a Schottky into BB_EN. Simplest; wakes on any 5 V.

Firmware for it already exists in shape: the headset enters USOUND when external power appears
(`handle_battery()`, "wired wins"). What would change is the *off* side — on unplug, decide
whether to stay on (wireless) or power off if no button session started it; that needs a
"power-on cause" (read BTN_PWR at boot: pressed = button, else USB).

Cost: one inverter + one dual diode (+ two resistors), and the standby current of whatever
stays powered from VSYS. Benefit: a switched-off headset appears on the PC when plugged in, like
most commercial wired-capable headsets.
