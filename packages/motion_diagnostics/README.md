# `motion-diagnostics` Python Package

Shared binary framing, CRC16-CCITT validation, command serialization, telemetry correlation, long-form CSV logging, and shaft-equivalent metrology package used by both [`VisionSystem`](../../VisionSystem/README.md) and [`motion-diag`](../../MotorControls/MotionFirmware/tools/diag/README.md).

---

## 1. Modules (`src/motion_diagnostics/`)

| Module | Responsibility |
| :--- | :--- |
| `crc.py` | `crc16_ccitt(data)` using polynomial `0x1021`, initial value `0xFFFF`, unreflected (`CCITT-FALSE`) |
| `status_bits.py` | `StatusBits` `IntFlag` definitions (`0..17`), `FAULT_BITS` mask (`0x0003E7E2`), `has_fault`, `command_succeeded`, `host_ready` |
| `commands.py` | Deployed 18-byte vision/control command codec (`Command`, `decode_command`, `encode_detection_packet`, `control_packet`; types `1=Pick`, `2=Heartbeat`, `3=Abort`, `4=Rehome`) |
| `frames.py` | Version-1 little-endian binary record parser (`0xD3 0x7E` preamble, records `0x81` `COMMAND_RESULT` 312 B, `0x82` `HOME_RESULT` 80 B, `0x83` `STATUS` 72 B) |
| `correlation.py` | Host-side `Correlator` joining TX `Command` entries (`host_send_seq`) with STM32 diagnostic records (`command_seq`, `record_seq`, `home_epoch`) and tracking missing/late/duplicate/reset counts |
| `csv_io.py` | Long-form CSV schema (`COLUMNS`, `derive`) producing one axis-phase row per record without zero-filling missing fields |
| `metrics.py` | Shaft-equivalent statistical summary (`summarize`, `analyze`: bias, MAE, max abs error, RMSE, sample SD, and ISO-style 1D `AP`/`RP` analogues — excludes disqualified rows and carriage-accuracy claims) |

---

## 2. Fault & Readiness Policy (`v1`)

- Active fault mask (`FAULT_BITS`): `0x0003E7E2` (includes `TX_BACKPRESSURE`, bit 13).
- `RX_OVERFLOW` (bit 12) is treated as a per-record diagnostic event bit and does not latch host readiness off.
- `MSCNT_UNQUALIFIED` (bit 11) is informational and recorded without discarding healthy AS5600 encoder samples.
- `host_ready(homed_mask, status_bits)` requires both X and Z axes homed (`homed_mask & 3 == 3`) and zero active bits in `FAULT_BITS`.

---

## 3. Installation & Tests

The package is declared as a workspace editable dependency by both `VisionSystem/pyproject.toml` and `MotorControls/MotionFirmware/tools/diag/pyproject.toml`, or can be installed directly with `uv`:

```powershell
# Sync package environment and run unit tests
uv sync --project packages\motion_diagnostics
uv run --project packages\motion_diagnostics pytest packages\motion_diagnostics\tests
```
