# Virtual Hardware Emulation Infrastructure

This directory (`MotorControls/Emulation/`) provides automated TCP socket bridging and an in-process protocol mock server (`MockFirmwareServer`) for headless UART testing of `MotionFirmware` (`z_bounce_emulation`), alongside a legacy Renode script for the historical STM32F446RE platform.

---

## 1. Directory Structure

```text
MotorControls/Emulation/
├── renode/
│   └── stm32f446_uart.resc     # Legacy STM32F446RE Renode script (does not emulate STM32H723)
├── renode_runner.py            # Python RenodeController class, MockFirmwareServer & CLI harness
├── test_renode_harness.py      # Unittest verification suite for legacy Renode lifecycle
└── README.md                   # Infrastructure documentation
```

---

## 2. Mock Server vs. Legacy Renode Script

- **`MockFirmwareServer` (Active H723 Protocol Harness):** Emulates the `z_bounce_emulation` startup banner (`STM32H723 SKR Octopus Pro V1.1`), `PING`/`STATUS`/legacy ASCII commands, and 18-byte `VisionUart` binary detection/heartbeat frames with CRC16-CCITT verification on `127.0.0.1:12345`. Use `--mode mock` when testing against the active H723 workspace.
- **Legacy Renode Script (`renode/stm32f446_uart.resc`):** Bridges `sysbus.usart2` on `@platforms/cpus/stm32f4.repl` (Cortex-M4 @ 84 MHz) to TCP port `12345`. Because `MotionFirmware` now targets **STM32H723ZET6 (Cortex-M7)** on BIGTREETECH SKR Octopus Pro V1.1, running the current `.elf` under `stm32f4.repl` is unsupported; `--mode renode` only applies when supplying a legacy STM32F446RE ELF via `-e $bin=@...`.

If Renode is not on `PATH` or in a standard install location, set `RENODE_PATH` to the full path of `renode.exe` before running legacy Renode tests.

---

## 3. Python Controller (`renode_runner.py`)

The `RenodeController` class provides process management, in-process `MockFirmwareServer` execution (`mode="mock"` or fallback), and deterministic socket connectivity:

### Key Features
- **Mock Server & Headless Renode Execution**: Supports `mode="mock"` (active H723 UART protocol verification), `mode="renode"`, and `mode="auto"`.
- **Socket Bridging**: Automatically polls and validates TCP port readiness on `127.0.0.1:12345`.
- **Zero Loss Telemetry**: Captures early boot telemetry and welcome banners without data dropping.
- **RFC 854 IAC Filter**: Built-in `strip_telnet(bytes)` helper to filter out terminal negotiation codes.
- **Process Lifecycle & Cleanup**:
  - Windows `taskkill /F /IM renode.exe` process cleanup.
  - `RenodeController.kill_all_zombies()` class method for sweeping orphaned instances.
- **Context Manager**: Supports standard Python `with RenodeController(...) as ctrl:` idioms.

### Quick Usage Example
```python
import sys
from pathlib import Path

sys.path.insert(0, str(Path("MotorControls").resolve()))
from Emulation.renode_runner import RenodeController

with RenodeController(mode="mock") as ctrl:
    # Connect standard Python TCP socket to the emulated UART server
    sock = ctrl.connect_socket()

    # Read serial banner from firmware mock
    telemetry = ctrl.read_available(sock, timeout=2.0)
    print(telemetry.decode("utf-8", errors="replace"))
```

---

## 4. Verification & Testing

Run the full 11-test STM32 UART protocol emulation suite (recommended `--mode mock` on the H723 codebase):
```powershell
uv run --project VisionSystem python MotorControls\tests\emulation\run_tests.py --mode mock
```

Legacy Renode harness unit test suite (requires Renode installed and a legacy STM32F446RE ELF matching `stm32f446_uart.resc`):
```powershell
uv run python MotorControls\Emulation\test_renode_harness.py
```
