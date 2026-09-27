# MotionFirmware – XZ manipulator motion controller

STM32F446RE (Nucleo-64) firmware that drives the two NEMA 17 axes of the XZ
Cartesian manipulator through **TMC2240** drivers (SPI + STEP/DIR) and reads the
two **AS5600** magnetic encoders (I2C). The current application homes every axis
with **StallGuard4** (sensorless), measures its travel and then moves the axes
continuously back and forth. The motion layer already accepts absolute targets
in millimetres, so the vision/PC positioning protocol can be added on top later.

## Hardware mapping

Wiring: [`wiring.md`](wiring.md). Axis mapping (edit `App/Inc/app_config.h` and
`App/Src/app_config.c` if yours differs):

| Axis | Driver | CS | STEP | DIR | ENN | Step timer | AS5600 bus |
|---|---|---|---|---|---|---|---|
| 0 = **Z** (T8x8 lead screw, 400 µsteps/mm) | TMC2240 #0 | PC0 | PC1 | PC2 | PC6 | TIM2 | I2C3 (PA8 SCL, PC9 SDA) |
| 1 = **X** (GT2 20T belt, 80 µsteps/mm) | TMC2240 #1 | PC3 | PC4 | PC5 | PC7 | TIM5 | I2C1 (PB8 SCL, PB9 SDA) |

Shared SPI2 (mode 3, 2.625 MHz): PB13 SCK, PB14 MISO, PB15 MOSI. Console:
USART2 (ST-LINK/J-Link VCOM, 115200 8N1) **and** SEGGER RTT terminal 0.

> The thesis tables (TA 2026, tables 3.8/3.9 and section 3.6.2) assign motor 0
> to X and swap the encoder buses. This firmware follows `wiring.md`. If your
> harness follows the thesis, swap the two entries of `g_app_axis_cfg[]` and the
> `APP_AXIS*_ENCODER_I2C` macros. A swapped encoder is detected during homing
> and reported (`[ENC] ... swap the encoder buses`).

## Build, flash, test

```powershell
cmake --preset Debug            # or Release
cmake --build --preset Debug    # -> build/Debug/MotionFirmware.elf/.hex/.bin
flash.cmd                       # J-Link (flash.jlink loads build/Debug/MotionFirmware.hex)
```

Host unit tests (planner, StallGuard4 detector, encoder unwrap, TMC2240 math,
console parser and a closed-loop simulation of homing/cycling/fault handling):

```powershell
cmake -S tests -B build-host -G Ninja -DCMAKE_C_COMPILER=clang
cmake --build build-host
ctest --test-dir build-host --output-on-failure
```

## Operation

1. Power the Nucleo and the 24 V motor supply (order does not matter: drivers
   that do not answer are probed every 0.5 s; LD2 blinks at 1 Hz until one does).
2. Press **B1** (or type `start`): Z homes, then X homes, then both axes cycle.
3. Press **B1** again to stop (decelerate and hold), again to resume. Hold B1
   for 1.5 s to switch the drivers off (motors free, axes must be re-homed).
   In a fault, a short press clears it.

| LD2 | Meaning |
|---|---|
| 1 Hz blink | waiting for a TMC2240 (VM off / SPI wiring) |
| short flash every 2 s | idle |
| 4 Hz blink | homing |
| on | cycling |
| 0.5 Hz blink | paused, homed and holding |
| 10 Hz blink | fault (see console) |

### Console commands

| Command | Action |
|---|---|
| `help` | list commands |
| `status` | state, position, encoder, SG4, CS_ACTUAL, temperature of both axes |
| `start` / `stop` | home if needed and cycle / decelerate and hold |
| `home` | re-home all axes (no cycling afterwards) |
| `off` | disable both drivers (ENN high) |
| `clear` | clear faults; drivers are re-initialised automatically |
| `mode soft\|bounce` | cycle by step counting between soft limits / StallGuard4 stop-to-stop |
| `move <axis> <mm>` | move a homed, idle axis (clamped to the soft limits) |
| `set <axis> <param> <value>` | tune at runtime (`set` alone lists all parameters) |
| `get [axis]` | show the parameters |
| `diag [axis]` | TMC2240 register dump, StallGuard/encoder details |
| `log 0\|1\|2` | events only / + status every 250 ms / + SG4 trace while seeking |

Axis names are `z` and `x`. Example: `set x hspeed 80`, `move z 42.5`.

Telemetry (log level 1) prints one line per axis, e.g.

```
[   38.250] RUNNING, mode soft
  Z CYCLE          pos    71.35 mm  v   +20.0 mm/s  enc    71.34 (err -0.01)  SG 212 CS 31 34.5C  cyc 3
  X DWELL          pos   290.05 mm  v    +0.0 mm/s  enc   290.06 (err +0.01)  SG 288 CS 15 31.0C  cyc 9
```

## How it works

### StallGuard4 homing (per axis, Z first, then X)

1. **Prep** – HOMING driver profile: StealthChop2 at every speed (`TPWMTHRS=0`,
   SG4 only works in StealthChop), reduced homing current with `IHOLD=IRUN`,
   CoolStep off, SG4 enabled above half the seek speed via `TCOOLTHRS`,
   `SG4_THRS` = static value, SG4 filter and angle-offset compensation on.
   ENN is released and the motor stands still for 250 ms (StealthChop AT#1).
2. **Back-off** a few mm away from the first end (running start, AT#2).
3. **Seek 1** at constant speed towards the first end (Z: up/MAX, X: MIN).
   After reaching speed and a blanking time, a stall is confirmed from:
   the TMC2240 comparator (the `sg2` bit returned in *every* SPI status byte),
   an absolute SG4 threshold, or an SG4 drop below `sgratio` × the free-running
   baseline measured during this seek. Once the baseline is known, `SG4_THRS`
   is re-programmed so the chip comparator (and DIAG0) uses the calibrated
   threshold too. If the AS5600 is available, "steps advance but the shaft does
   not" is used as a backup detector.
4. **Retract**, **seek 2** across the whole axis to the other end, retract.
   On this traverse the encoder ratio (counts per microstep and sign) is
   calibrated; the travel is then taken from the encoder, so microsteps lost
   while pressing against the stops do not bias the coordinate frame.
5. MIN end = 0 mm, soft limits = `[margin, travel - margin]`, RUN profile
   (run current, optional SpreadCycle above `stealth_max_mm_s`, optional CoolStep).

Seeks are bounded by `max_travel_mm` (hard software limit in the step ISR) and
a timeout, so a missed stall ends in a fault, never in a runaway.

### Cycle modes

* **soft** (default) – trapezoidal moves between the soft limits, counted in
  steps, never touching the stops. The AS5600 following error is checked
  continuously (lag-compensated, 2 fullsteps); a lost step faults the axis.
* **bounce** – every stroke is a StallGuard4 seek into the hard stop. Each stop
  is reported with its deviation from the homed travel (encoder based if
  available) and the frame is re-referenced at the MIN stop: this directly
  produces the data for the sensorless homing repeatability test (TA 3.7.3).

### TMC2240 features used

| Feature | Registers | Where |
|---|---|---|
| Checked SPI configuration with read-back | all written registers | `tmc_axis.c` |
| Current: range + `GLOBAL_SCALER` so IRUN = 31, standstill reduction, homing current | `DRV_CONF`, `GLOBAL_SCALER`, `IHOLD_IRUN`, `TPOWERDOWN` | `tmc_calc.c` |
| 1/16 microsteps + MicroPlyer 1/256 interpolation, double-edge STEP | `CHOPCONF.MRES/intpol/dedge` | |
| StealthChop2 with auto-tuning, optional SpreadCycle hybrid | `GCONF.en_pwm_mode`, `PWMCONF`, `TPWMTHRS` | |
| StallGuard4 sensorless homing / crash detection | `SG4_THRS`, `SG4_RESULT`, `TCOOLTHRS`, `THIGH`, SPI status | `axis_ctrl.c`, `stall_detect.c` |
| CoolStep (off by default, tune first) | `COOLCONF` | `set`/`app_config.c` |
| Direction inversion | `GCONF.shaft` | `tmc.invert_dir` |
| Diagnostics: reset/UV/driver error, over-temperature, short, open load, CS_ACTUAL | `GSTAT`, `DRV_STATUS` | health poll every 50 ms |
| Chip temperature and motor supply voltage | `ADC_TEMP`, `ADC_VSUPPLY_AIN` | every 1 s |
| DIAG0 stall output (for an optional EXTI wire) | `GCONF.diag0_stall` | |

## Tuning StallGuard4 on the machine

1. `log 2`, then `home`. While seeking, lines `sg,<axis>,<ms>,<pos mm>,<SG4>,<baseline>,<SG4_THRS>,<flag>`
   show the load signal. Free-running SG4 should be clearly above `sgmin`
   (default 40) and drop towards 0 at the stop.
2. **Stops too early / "travel too short"** (false stall): lower `sgratio`
   (e.g. `set x sgratio 0.3`), raise `confirm` or `blank`, or seek faster
   (`hspeed`) – more back-EMF gives a cleaner SG4 signal.
3. **Grinds at the stop / "no stall detected"**: raise `sgratio` (0.6) or `sg`,
   lower `hcurrent`. The encoder backup detector also catches this if the
   AS5600 is working.
4. **Wrong direction** (MIN/MAX swapped): set `tmc.invert_dir = true` for that
   axis in `app_config.c` (uses `GCONF.shaft`, no rewiring needed).
5. Make a good setting permanent in `App/Src/app_config.c`.

Currents (`current`, `hcurrent`) re-initialise the driver; the defaults are
1000/600 mA (Z) and 800/500 mA (X) RMS – match them to your motors' rating.

## Code structure

| Module | Role | Host tested |
|---|---|---|
| `App/Src/motion_profile.c` | exact integer trapezoidal planner + step sequencer used by the ISR | yes |
| `App/Src/stepgen.c` | TIM2/TIM5 update ISR: one STEP edge per interrupt, DIR, travel limits | – |
| `App/Src/stall_detect.c` | StallGuard4 stall decision (blanking, baseline, confirm) | yes |
| `App/Src/axis_ctrl.c` | homing / cycling / move / fault state machine per axis | yes (simulation) |
| `App/Src/tmc_calc.c`, `tmc_axis.c` | TMC2240 register values, profiles, health, dump | math: yes |
| `App/Src/enc_track.c`, `encoder.c` | AS5600 multi-turn tracking, magnet check, I2C bus recovery | unwrap: yes |
| `App/Src/console.c`, `cmd_parse.c` | DMA UART + RTT console, command parser | parser: yes |
| `App/Src/app.c`, `app_config.c` | sequencing, button, LED, telemetry, commands, defaults | config: yes |
| `Core/`, `MotionFirmware.ioc` | CubeMX peripheral set-up (all user code in `USER CODE` blocks / `App/`) | – |
| `../Drivers/TMC2240-Driver`, `../Drivers/AS5600` | checked chip drivers (unchanged) | own suites |

`MotionFirmware.ioc` was updated for I2C1/I2C3, TIM2/TIM5, USART2 + DMA and
the motor-1 GPIOs without CubeMX available; open it in CubeMX and compare with
`Core/Src/main.c` before regenerating code.

### Adding the PC / vision positioning protocol

`console_getc()` delivers the USART2 byte stream (DMA, idle-line) that today
feeds the ASCII command parser. The framed packets described in
`VisionSystem/REMEDIATION_PLAN_AI.md` (`0xAA 0x55`, target X/Y in 0.1 mm,
CRC16-CCITT, `\r\n`) can be decoded in the same place in `app_run()` and turned
into `axis_ctrl_move_mm()` calls: moves are clamped to the homed soft limits,
supervised by the encoders and can be re-targeted while running.

## Safety notes

* ENN is held high (power stage off) from reset until an axis is homed or
  moved, on every fault, in `Error_Handler()` and in the fault exception
  handlers. An external pull-up on ENN keeps the drivers off while the MCU is
  in reset.
* Homing drives the carriages into the mechanical end stops at reduced current;
  make sure both ends of each axis are solid and clear of the conveyor.
* Never plug or unplug a motor while VM is on (`wiring.md`).
