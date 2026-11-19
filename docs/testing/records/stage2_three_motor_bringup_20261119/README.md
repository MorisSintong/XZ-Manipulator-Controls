# Stage 2: Three-Motor 24 V Diagnostic, StallGuard4 Homing & Rotation Angle Precision Record (`2026-11-19`)

**Session Timestamp**: `2026-11-19 21:11:59` (`20261119_211159`)
**Hardware Target**: `BIGTREETECH SKR Octopus Pro V1.1` (`STM32H723ZET6` @ 400 MHz) + 3x `TMC2240` SPI Drivers (`MOTOR1` = Z, `MOTOR0` = X, `MOTOR2_1` = Rotation $\theta$)
**Firmware Binary**: `MotionFirmware_StallGuardCLI` (`MotorControls/MotionFirmware/Apps/stallguard_cli`)
**Host Connection**: USB-C CDC Virtual COM Port (`COM11`)
**Power Supply**: 24 V DC Main Bus (`~23.8 V` at TMC2240 `ADC_VSUPPLY`)

---

## 1. Record Contents

| File | Description |
| :--- | :--- |
| [`diag_20261119_211159.log`](diag_20261119_211159.log) | Raw timestamped USB-CDC telemetry stream (`353` status frames, `2,559` 50 Hz SG4 frames, `17` `[ROT]` precision reports, `110` state/homing/diag events) |
| [`diag_20261119_211159.json`](diag_20261119_211159.json) | Structured JSON report containing initial/final 3-driver registers, 2x Z/X homing summaries, soft-limit move checks, 17-point rotation precision dataset, and full telemetry arrays |
| [`movement_overview_20261119_211159.png`](movement_overview_20261119_211159.png) | 5-panel overview plot (Z position/SG4, X position/SG4, Rotation $\theta$ target vs. actual angle, Rotation quantization & `MSCNT` phase error, and 3-driver temperatures/current scales) |
| [`rotation_precision_20261119_211159.png`](rotation_precision_20261119_211159.png) | Dedicated 3-panel Rotation ($\theta$) precision plot (target vs. actual angle, microstep quantization error vs. $\pm 0.05625^\circ$ half-step band, and TMC2240 10-bit `MSCNT` electrical phase verification) |
| [`movement_rerun_20261119_211159.rrd`](movement_rerun_20261119_211159.rrd) | Interactive Rerun (`.rrd`) recording (`130.98 s` timeline) with live 2D X-Z gantry + rotation pointer arrow schematic, Z/X/$\theta$ time-series plots, 3-driver thermal/CS telemetry, and event log |

### Replaying the Rerun Recording

```powershell
# Launch the Rerun viewer directly on the saved .rrd file:
uv run --with rerun-sdk rerun docs/testing/records/stage2_three_motor_bringup_20261119/movement_rerun_20261119_211159.rrd

# Or regenerate and spawn from the JSON capture:
uv run --with rerun-sdk python MotorControls/MotionFirmware/tools/stallguard_to_rerun.py `
    docs/testing/records/stage2_three_motor_bringup_20261119/diag_20261119_211159.json --spawn
```

---

## 2. Three-Driver 24 V Health Summary (`TMC2240` SPI)

| Axis | Socket (Driver) | Mechanics & Scale | Current (`Run / Hold / Home`) | Initial -> Final `GSTAT` | Initial -> Final `DRV_STATUS` | Initial -> Final `VM` | Initial -> Final Temp | Faults / Drops |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **Z** | `MOTOR1` (`#0`) | T8x8 lead screw (`400 usteps/mm`) | `1000 / 500 / 594 mA` | `0x00` -> `0x00` | `0x81004000` -> `0x800F40C9` | `23804` -> `23794 mV` | `33.7` -> `46.4 °C` | `0` / `0` |
| **X** | `MOTOR0` (`#1`) | GT2 20T belt (`80 usteps/mm`) | `801 / 401 / 501 mA` | `0x00` -> `0x00` | `0x81004000` -> `0x800F40C6` | `23853` -> `23814 mV` | `33.2` -> `42.3 °C` | `0` / `0` |
| **Rot ($\theta$)** | `MOTOR2_1` (`#2`) | 1.8° NEMA 17 (`8.888889 usteps/deg`, `0.1125°/ustep`) | `801 / 401 mA` | `0x00` -> `0x00` | `0x81004017` -> `0x811D4010` | `23775` -> `23765 mV` | `32.7` -> `43.3 °C` | `0` / `0` |

---

## 3. StallGuard4 Sensorless Homing Repeatability (2x Runs per Axis)

| Axis | Run | Homing Sequence | First Endstop Stall (`SG4 / Base`) | Second Endstop Stall (`SG4 / Base`) | Measured Total Travel | Calibrated Soft Limits | Travel Repeatability ($\Delta$) |
| :--- | :---: | :--- | :--- | :--- | :---: | :---: | :---: |
| **Z (`MOTOR1`)** | **#1** | `MAX (+1)` then `MIN (-1)` | `+71.90 mm` (`221 / 311`) | `-43.85 mm` (`221 / 344`) | **`115.75 mm`** | `5.00 .. 110.75 mm` | **`0.00 mm`** |
| **Z (`MOTOR1`)** | **#2** | `MAX (+1)` then `MIN (-1)` | `+115.60 mm` (`218 / 315`) | `-0.15 mm` (`240 / 344`) | **`115.75 mm`** | `5.00 .. 110.75 mm` | (`0 usteps`) |
| **X (`MOTOR0`)** | **#1** | `MIN (-1)` then `MAX (+1)` | `-214.25 mm` (`208 / 329`) | `+8.44 mm` (`235 / 340`) | **`222.69 mm`** | `10.00 .. 212.69 mm` | **`0.06 mm`** |
| **X (`MOTOR0`)** | **#2** | `MIN (-1)` then `MAX (+1)` | `+3.99 mm` (`221 / 332`) | `+226.74 mm` (`171 / 342`) | **`222.75 mm`** | `10.00 .. 212.75 mm` | (`5 usteps`) |

---

## 4. Linear Soft-Limit Positioning & Simultaneous 2-Axis Soft-Cycle

- **Point-to-Point Positioning Accuracy**:
  - `move z 5.00` -> settled at **`5.00 mm`** (`0.00 mm` error)
  - `move z 110.75` -> settled at **`110.75 mm`** (`0.00 mm` error)
  - `move z 57.88` (midpoint) -> settled at **`57.88 mm`** (`0.00 mm` error)
  - `move x 10.00` -> settled at **`10.00 mm`** (`0.00 mm` error)
  - `move x 212.75` -> settled at **`212.75 mm`** (`0.00 mm` error)
  - `move x 111.38` (midpoint) -> settled at **`111.38 mm`** (`0.00 mm` error)
- **Simultaneous Dual-Axis Soft-Limit Cycling (`mode soft`, 12 s)**:
  - Both Z and X axes cycled concurrently between their calibrated soft limits without false StallGuard trips (`0` unexpected stalls; Z completed `1` full stroke cycle, X completed `4` full stroke cycles) and returned cleanly to `READY` on `stop`.

---

## 5. Rotation ($\theta$) Motor 17-Point Angle Precision & `MSCNT` Phase Verification

The rotation motor (`MOTOR2_1`, `TMC2240 #2`) operates at `16` microsteps/fullstep with `1/256` MicroPlyer interpolation (`3200 usteps/rev` = `8.888889 usteps/deg`, step quantum = `0.1125°/ustep`). Each microstep advances the TMC2240 internal 10-bit sine-table phase counter (`MSCNT`, `0x6A`, modulo `1024`) by `16` counts (`1024 counts = 4 fullsteps = 7.2°`).

| Step # | Commanded Angle (`cmd_deg`) | Actual Quantized Angle (`actual_deg`) | Microstep Index (`ustep`) | $\Delta$ Microsteps (`delta_ustep`) | Quantization Residual (`quant_err_deg`) | `MSCNT` Start -> End (Expected) | `MSCNT` Count Error | Electrical Phase Error (`phase_err_deg`) |
| :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **Init** | `0.0000°` (`zero`) | `0.0000°` | `0` | `0` | `0.0000°` | `8` -> `8` (`8`) | `0` | `0.0000°` |
| **1** | `+0.1125°` | `+0.1125°` | `+1` | `+1` | `0.0000°` | `8` -> `11`* (`24`) | `-13`* | `-0.0914°`* |
| **2** | `+0.4500°` | `+0.4500°` | `+4` | `+3` | `0.0000°` | `24` -> `72` (`72`) | **`0`** | **`0.0000°`** |
| **3** | `+1.8000°` | `+1.8000°` | `+16` | `+12` | `0.0000°` | `72` -> `264` (`264`) | **`0`** | **`0.0000°`** |
| **4** | `+5.0000°` | `+4.9500°` | `+44` | `+28` | `-0.0500°` | `264` -> `712` (`712`) | **`0`** | **`0.0000°`** |
| **5** | `+11.2500°` | `+11.2500°` | `+100` | `+56` | `0.0000°` | `712` -> `584` (`584`) | **`0`** | **`0.0000°`** |
| **6** | `+45.0000°` | `+45.0000°` | `+400` | `+300` | `0.0000°` | `584` -> `264` (`264`) | **`0`** | **`0.0000°`** |
| **7** | `+90.0000°` | `+90.0000°` | `+800` | `+400` | `0.0000°` | `264` -> `520` (`520`) | **`0`** | **`0.0000°`** |
| **8** | `+180.0000°` | `+180.0000°` | `+1600` | `+800` | `0.0000°` | `520` -> `8` (`8`) | **`0`** | **`0.0000°`** |
| **9** | `+360.0000°` | `+360.0000°` | `+3200` | `+1600` | `0.0000°` | `8` -> `8` (`8`) | **`0`** | **`0.0000°`** |
| **10** | `+180.0000°` | `+180.0000°` | `+1600` | `-1600` | `0.0000°` | `8` -> `520` (`520`) | **`0`** | **`0.0000°`** |
| **11** | `+90.0000°` | `+90.0000°` | `+800` | `-800` | `0.0000°` | `520` -> `264` (`264`) | **`0`** | **`0.0000°`** |
| **12** | `0.0000°` | `0.0000°` | `0` | `-800` | `0.0000°` | `264` -> `8` (`8`) | **`0`** | **`0.0000°`** |
| **13** | `-45.0000°` | `-45.0000°` | `-400` | `-400` | `0.0000°` | `8` -> `776` (`776`) | **`0`** | **`0.0000°`** |
| **14** | `-90.0000°` | `-90.0000°` | `-800` | `-400` | `0.0000°` | `776` -> `520` (`520`) | **`0`** | **`0.0000°`** |
| **15** | `-180.0000°` | `-180.0000°` | `-1600` | `-800` | `0.0000°` | `520` -> `8` (`8`) | **`0`** | **`0.0000°`** |
| **16** | `-360.0000°` | `-360.0000°` | `-3200` | `-1600` | `0.0000°` | `8` -> `8` (`8`) | **`0`** | **`0.0000°`** |
| **17** | `0.0000°` | `0.0000°` | `0` | `+3200` | `0.0000°` | `8` -> `8` (`8`) | **`0`** | **`0.0000°`** |

*\*Note on Step 1 (`+0.1125°`, single `1 ustep` pulse from standstill): Because only one `STEP` rising edge is emitted without a preceding step interval, TMC2240's `1/256` MicroPlyer interpolator steps slowly using its standby clock reference (`MSCNT` observed at `11` at the `15 ms` readback window) before completing at `MSCNT = 24`, as confirmed by Step 2's starting readback (`mscnt_start = 24`) and **`0` count error across all 16 multi-step moves (`12,800` total microsteps traversed across $\pm 360^\circ$)**.*
