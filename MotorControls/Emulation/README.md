# Virtual Hardware Emulation Infrastructure (Renode)

This directory provides automated headless emulation and deterministic hardware-in-the-loop (HIL) socket bridging for the **STM32F446RE** MCU running `MotionFirmware`.

---

## 1. Directory Structure

```text
Emulation/
├── renode/
│   └── stm32f446_uart.resc     # Renode emulation script for STM32F446RE
├── renode_runner.py            # Python RenodeController class and CLI harness
├── test_renode_harness.py      # Unittest verification suite
├── renode.log                  # Emulation output and diagnostic log
└── README.md                   # Infrastructure documentation
```

---

## 2. Emulation Script (`stm32f446_uart.resc`)

The script loads the virtual platform, flashes the target ELF, and bridges `sysbus.usart2` to a TCP server socket:

- **Platform**: `@platforms/cpus/stm32f4.repl` (Cortex-M4 @ 84 MHz / 125 MIPS)
- **Firmware Binary**: `$bin?=@MotorControls/MotionFirmware/build/Debug/MotionFirmware.elf`
- **UART Bridge**: `emulation CreateServerSocketTerminal $port "term"` connected to `sysbus.usart2`
- **Default Port**: `12345` (TCP localhost)

Variables `$bin` and `$port` can be dynamically overridden by pre-defining them before including the script.

---

## 3. Python Controller (`renode_runner.py`)

The `RenodeController` class provides headless process management and deterministic socket connectivity:

### Key Features
- **Headless Execution**: Runs `renode.exe` with `--plain`, `--disable-gui`, and `-P -1` (disabling telnet monitor to prevent port 1234 collisions).
- **Socket Bridging**: Automatically polls and validates TCP port readiness on `127.0.0.1:12345`.
- **Zero Loss Telemetry**: Captures early boot telemetry and welcome banners without data dropping.
- **RFC 854 IAC Filter**: Built-in `strip_telnet(bytes)` helper to filter out terminal negotiation codes.
- **Process Lifecycle & Cleanup**:
  - Windows `taskkill /F /T /PID` process-tree termination on shutdown.
  - `atexit` safety handlers.
  - `RenodeController.kill_all_zombies()` class method for sweeping orphaned instances.
- **Context Manager**: Supports standard Python `with RenodeController(...) as ctrl:` idioms.

### Quick Usage Example
```python
from renode_runner import RenodeController

with RenodeController() as ctrl:
    # Connect standard Python TCP socket to sysbus.usart2
    sock = ctrl.connect_socket()

    # Read serial banner from STM32 firmware
    telemetry = ctrl.read_available(sock, timeout=2.0)
    print(telemetry.decode("utf-8", errors="replace"))
```

---

## 4. Verification & Testing

Run the built-in self-test:
```powershell
python Emulation/renode_runner.py
```

Run the unit test suite:
```powershell
python Emulation/test_renode_harness.py
```
