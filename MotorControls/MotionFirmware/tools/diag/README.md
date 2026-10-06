# Motion diagnostics host tools

Run from repository root (Windows PowerShell). One process owns the serial port;
stop live vision before running a bench sequence. No hardware qualification or
ISO certification is implied by a successful test. UART abort is software-only.

```powershell
uv sync --project MotorControls\MotionFirmware\tools\diag
uv run --project MotorControls\MotionFirmware\tools\diag motion-diag --help
uv run --project MotorControls\MotionFirmware\tools\diag motion-diag capture --port COM9 --csv results\capture.csv --config results\config.json
uv run --project MotorControls\MotionFirmware\tools\diag motion-diag heartbeat --port COM9 --csv results\status.csv --config results\config.json
uv run --project MotorControls\MotionFirmware\tools\diag motion-diag send-sequence --port COM9 --csv results\trials.csv --config results\config.json --targets-mm 25,100,200 --approach-mm 5 --reverse-approach-mm 250 --direction both --repetitions 30 --pacing 0.1
uv run --project MotorControls\MotionFirmware\tools\diag motion-diag analyze --csv results\trials.csv
uv run --project MotorControls\MotionFirmware\tools\diag motion-diag abort --port COM9 --csv results\abort.csv --config results\config.json
uv run --project MotorControls\MotionFirmware\tools\diag motion-diag rehome --port COM9 --csv results\home.csv --config results\config.json
```

COM9 is illustrative; select the actual board VCOM. `python -m diag` is equivalent.
Capture runs until Ctrl+C (or `--duration` seconds). Results include raw `.bin`,
long-form CSV, `.metadata.json` run settings and `.health.json` transport counts.
`analyze` produces a console summary and `_report.json` / `_report.csv`.
JSON and metric-labelled CSV preserve endpoint, delta, tracking and unwrapped angular statistics.
Missing completions are taken from the health sidecar; without it missing is
unavailable, not zero. CRC-corrupted data remains in the raw capture.

## Configuration and safety

Supply a JSON run configuration, matching the actual immutable firmware config.
Minimal dry-run **example only**, not an authorization to move:

```json
{
  "config_id": 1,
  "qualified": false,
  "usable_x_mm": 0,
  "safe_z_01mm": 0,
  "pick_depth_01mm": 0,
  "x_lead_um": 40000,
  "z_lead_um": 8000,
  "steps_per_rev": 3200,
  "firmware_revision": "record the built revision",
  "encoder_signs": "record physically qualified signs",
  "load": "record tool/load",
  "temperature": "record conditions",
  "speeds_acceleration": "record actual settings",
  "timing_sg_encoder_conf_clamp_policy": "record actual settings"
}
```

Only set `qualified=true`, nonzero usable travel and fixed depth after physical
sign/scale, bounds, driver phase, home, interlock and Z support checks. Scored
sequences require config ID 1 and explicit qualified 0 ≤ safe Z ≤ fixed pick depth.
Host metadata cannot change firmware depth. Y remains conveyor Y.

The sender requires identity-matched homed/READY heartbeat before picks.
Each scored arrival has its own non-scored preposition pick (a complete
X/Z-down/Z-up cycle, so use qualified dry-run depth for preposition testing).
Use lower `--approach-mm` for positive approach; for negative use higher
`--approach-mm`. `both` additionally requires higher `--reverse-approach-mm`.
`--rehome-each-trial` produces separate home epochs, not pooled repeatability.
Every command gets a fresh ID; timeouts send abort and **never retry** an uncertain
command. Faulted results stop the sequence. Rejected re-home STATUS is terminal
and reported as rejected, without waiting for a nonexistent HOME_RESULT.
SUCCESS plus informational/event bits is still a successful trial; clamped
results remain excluded from nominal metric populations.
Readiness requires both home bits and no v1 fault bits (`0x0003E7E2`);
command success requires SUCCESS and no fault bits. RX_OVERFLOW events and
MSCNT_UNQUALIFIED do not invalidate healthy encoder populations; active
TX_BACKPRESSURE is readiness-blocking.

## Schema and statistics

The requested host CSV uses **one row per axis-phase**, rather than the design
document's wide record row. Header/host identity repeats across three command
rows; home produces two axis rows, status one row. `csv_io.COLUMNS` is the stable
union schema; blank means unavailable, never a synthetic zero. Trial direction
and phase direction are retained separately. All wire fields remain available.

Errors follow design §4 using counts/steps, not rounded telemetry. Endpoint
error is separate from move-delta error. Groups include axis, phase, applied
target, direction, pose, trial direction, config, home epoch and session.
Invalid, faulted, clamped, non-scored, late and duplicate rows are excluded and
counted. Unqualified config/sign evidence is not nominal acceptance. Healthy
encoder metrics do not depend on MSCNT qualification or snapshot validity;
the independent `mscnt_check`, `driver_mode` and flags remain in CSV.
SD and radius RP are unavailable for n<2. AP/RP are labelled **ISO-style 1D
analogues**, shaft-equivalent only; external calibrated carriage metrology and
licensed ISO text are required for formal qualification.

## Tests

```powershell
Set-Location MotorControls\MotionFirmware\tools\diag
uv run pytest
uv run ruff check src tests
uv run ruff format --check src tests
```

Tests emulate STM32 binary replies on fake serial, including fragmented replies,
timeouts/abort, scored preposition separation and offline reports. No COM port
or network share is accessed by tests.
