# Motion Characterization Bench Runbook

Isolated, passive-by-default metrology firmware (`MotionFirmware_Characterization`) for characterizing the installed AS5600 encoders, TMC2240 drivers, and X / Z / $\theta$ stepper axes on the BIGTREETECH SKR Octopus Pro V1.1 (STM32H723ZET6).

## Safety & Passive-by-Default Invariants

- Boots with **all ENN pins HIGH** (power stages disabled), zero STEP edges, no homing, no auto-recovery, and no motion on USB connect.
- Uses **native USB-C CDC exclusively** (does not touch USART2 or the vision protocol).
- **B1** is a software stop (EXTI + held-level check), **not** a safety-rated E-stop. Keep an independently tested physical VM power cut within reach.
- **Z-axis gravity protection:** mechanically restrain or provide a physical catch/brake for Z that protects on loss of holding torque without bearing the test load through the measured stroke.
- **One active axis per trial.** Mechanically secure inactive axes.
- **Separate sessions:**
  - `xz`: Electrical reads on X/Z/$\theta$ allowed; X and Z may move only after live commissioning and arming; $\theta$ motion is forbidden (`no_shaft_feedback`).
  - `theta_fixture`: Park and secure X and Z. With power OFF, mount a shaft-mounted AS5600 on $\theta$ and connect it to the I2C1 header (in place of X). Only $\theta$ may move, and only after its own live feedback check. Recommission X after restoring the harness.

## Build (Software-Only Workstation)

```powershell
Push-Location MotorControls/MotionFirmware
cmake --preset Release -DCMAKE_C_COMPILER_LAUNCHER=sccache -DCMAKE_CXX_COMPILER_LAUNCHER=sccache
cmake --build --preset Release --target MotionFirmware_Characterization MotionFirmware_StallGuardCLI
cmake -S Apps/motion_characterization/tests -B build/host-characterization -G Ninja -DCMAKE_C_COMPILER=clang
cmake --build build/host-characterization
ctest --test-dir build/host-characterization --output-on-failure
Pop-Location
```

## Flashing (Hardware Attached & Authorized Only)

1. Follow the BTT bootloader backup precaution and J-Link SWD connection in `MotorControls/MotionFirmware/README.md`.
2. Confirm `build/Release/MotionFirmware_Characterization.hex` is up to date and VM is isolated.
3. From `MotorControls/MotionFirmware`:
   ```powershell
   .\flash.cmd characterization
   ```
4. Verify all ENN outputs remain HIGH after reset before applying VM.

## Bench Campaign Sequence

Run from the repository root with `uv` and the actual USB-C CDC port:

```powershell
$tool = 'MotorControls/MotionFirmware/tools/diag'
$port = Read-Host 'Actual USB-C CDC port'
$id = [DateTime]::UtcNow.ToString('yyyyMMddTHHmmssZ')
$base = "docs/testing/records/motion_characterization_${id}_xz"

uv run --project $tool motion-characterize setup --out "${base}_setup.json"
uv run --project $tool motion-characterize run --stage passive --port $port --manifest "${base}_setup.json" --out "${base}_passive"
uv run --project $tool motion-characterize run --stage motion --port $port --manifest "${base}_passive/manifest.json" --out "${base}_motion"
uv run --project $tool motion-characterize report "${base}_motion"
```

1. **Passive stage** (`--stage passive`): verifies `HELLO`, commissions encoders with power stages OFF (manual positive/negative shaft turn $\ge 64$ counts, partner quiet), runs I2C rate windows (`raw` and `sample`, single and alternating), static precision (4 quarter-turn positions $\times$ 1024 samples), and TMC2240 SPI rate sweep (`/256` to `/16`, restoring `/32`).
2. **Motion stage** (`--stage motion`): repeats live encoder commissioning, imports passive qualification by capture ID, verifies safety interlocks (`STOP`, heartbeat lease timeout, USB unplug, B1) on a gentle restrained trial, then runs sign jog, slow reference returns, minimum detectable displacement, speed/acceleration sweeps, and 60 s loaded stress test.
3. **Reference stage** (`--stage reference`): optional manually indexed absolute-angle capture (12 positions at $30^\circ$ intervals, 5 approaches each direction, 32 settled samples per point) when an independent reference instrument is available.
4. **Replay report** (`motion-characterize report <run-dir>`): regenerates `trials.csv`, `summary.json`, and `report.md` deterministically without opening a serial port.
