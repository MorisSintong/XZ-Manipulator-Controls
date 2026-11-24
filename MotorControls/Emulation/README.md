# Mock UART Emulation Infrastructure

This directory (`MotorControls/Emulation/`) provides an in-process STM32H723 mock UART firmware server (`MockFirmwareServer`) for headless protocol testing of `MotionFirmware` (`z_bounce_emulation`) without attached hardware.

---

## 1. Directory Structure

```text
MotorControls/Emulation/
├── mock_firmware_server.py     # In-process MockFirmwareServer & standalone TCP CLI
└── README.md                   # Infrastructure documentation
```

---

## 2. `MockFirmwareServer` (`mock_firmware_server.py`)

`MockFirmwareServer` emulates the `z_bounce_emulation` startup banner (`STM32H723 SKR Octopus Pro V1.1`), legacy ASCII commands (`PING`, `STATUS`, `K<cls>,S<angle>,R<corr>`), and 18-byte `VisionUart` binary detection/heartbeat frames (`0x01`/`0x02`) with CRC16-CCITT verification over `127.0.0.1:12345`.

### Quick Usage Example
```python
import socket
import sys
from pathlib import Path

sys.path.insert(0, str(Path("MotorControls").resolve()))
from Emulation.mock_firmware_server import MockFirmwareServer

with MockFirmwareServer(host="127.0.0.1", port=12345):
    with socket.create_connection(("127.0.0.1", 12345), timeout=2.0) as sock:
        banner = sock.recv(4096)
        print(banner.decode("latin1", errors="replace"))
```

Or run standalone as a local TCP mock server:
```powershell
uv run python MotorControls\Emulation\mock_firmware_server.py --port 12345
```

---

## 3. Verification & Testing

Run the 11-test STM32H723 mock UART protocol suite:
```powershell
uv run --project VisionSystem python MotorControls\tests\emulation\run_tests.py
```
