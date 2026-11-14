# MotionFirmware — STM32H723 (SKR Octopus Pro V1.1) XZ Manipulator Firmware Suite

STM32H723ZET6 (`BIGTREETECH SKR Octopus Pro V1.1`, 400 MHz CPU / 200 MHz HCLK / 100 MHz APB1 & APB2 from 25 MHz HSE) firmware suite for the 2-Axis (XZ) Cartesian manipulator, driving **Trinamic TMC2240** stepper drivers (shared `SPI1` + per-socket `STEP`/`DIR`/`ENN`/`CS`) and reading dual **AS5600** magnetic encoders (`I2C1` on `J73` `PB8`/`PB9` for X-axis, `I2C4` on `J43` BLTouch `PB6`/`PB7` for Z-axis).

This project uses a **single shared CubeMX `Core/` and `Drivers/` layer** ([MotionFirmware.ioc](MotionFirmware.ioc)) with selectable application profiles under [`Apps/`](Apps/):

| Application Profile | Folder | Output Binary (`.elf` / `.hex` / `.bin`) | Description |
| :--- | :--- | :--- | :--- |
| **`motion_diagnostics`** *(default)* | [`Apps/motion_diagnostics/`](Apps/motion_diagnostics/README.md) | `MotionFirmware_Diagnostics.*` *(and default `MotionFirmware.*`)* | Binary-only `USART2` (`PD5`/`PD6`) DMA vision pick executor (`0x01..0x04`), single-end StallGuard4 homing (Z then X), 1 MHz timer compare stepping (`TIM2`/`TIM5`), and 356/124/116-byte binary evidence records (`MSCNT` + unwrapped AS5600 + step counts) for the Python [`tools/diag`](tools/diag/README.md) analyzer and live `VisionSystem`. |
| **`stallguard_cli`** | [`Apps/stallguard_cli/`](Apps/stallguard_cli/README.md) | `MotionFirmware_StallGuardCLI.*` | Interactive ASCII CLI (`USART2` + SEGGER RTT) with two-end StallGuard4 sensorless homing, full travel & encoder-ratio calibration, continuous **`soft`** and **`bounce`** cycling modes, and live closed-loop step-loss supervision. |
| **`z_bounce_emulation`** | [`Apps/z_bounce_emulation/`](Apps/z_bounce_emulation/README.md) | `MotionFirmware_ZBounceEmulation.*` | Single-axis Z StallGuard4 bounce demo + `VisionUart` binary (`0x01`/`0x02`) & legacy ASCII responder. |
| **`usb_cdc`** *(portable module + test app)* | [`Apps/usb_cdc/`](Apps/usb_cdc/README.md) | `MotionFirmware_UsbCdc.*` | Portable, non-blocking **TinyUSB `USB_OTG_HS` CDC** module (`PA11`/`PA12`, `HSI48+CRS`, VBUS override) ready to drop into any motion profile via `add_usb_cdc_module()`, plus a standalone Virtual COM verification app (`PING`/`INFO`/`STAT`/`BINTEST`). |

---

## 1. SKR Octopus Pro V1.1 (STM32H723) Pin Mapping

| Function | Firmware Symbol / Peripheral | MCU Pin(s) | Octopus Pro V1.1 Connector |
| :--- | :--- | :--- | :--- |
| **X-Axis TMC2240** (`motor=1`, `TIM5`) | `TMC_STEP1` / `TMC_DIR1` / `TMC_ENN1` / `TMC_CS1` / `TMC_DIAG1` | `PF13` / `PF12` / `PF14` / `PC4` / `PG6` | **Driver 0 (`MOTOR0`)** socket |
| **Z-Axis TMC2240** (`motor=0`, `TIM2`) | `TMC_STEP0` / `TMC_DIR0` / `TMC_ENN0` / `TMC_CS0` / `TMC_DIAG0` | `PG0` / `PG1` / `PF15` / `PD11` / `PG9` | **Driver 1 (`MOTOR1`)** socket |
| **Rotation ($\theta$) TMC2240** (`motor=2`, provisioned) | `TMC_STEP2` / `TMC_DIR2` / `TMC_ENN2` / `TMC_CS2` / `TMC_DIAG2` | `PF11` / `PG3` / `PG5` / `PC6` / `PG10` | **Driver 2 (`MOTOR2_1`)** socket (initialized disabled; open-loop placeholder) |
| **Shared TMC2240 SPI** | `SPI1` (Mode 3, 8-bit, 3.125 MHz) | `PA5` (SCK), `PA6` (MISO), `PA7` (MOSI) | Internal driver SPI bus |
| **X-Axis AS5600 Encoder** | `I2C1` (400 kHz Fast Mode) | `PB8` (SCL), `PB9` (SDA) | **`J73` (`I2C`)** 4-pin header (3.3V, GND, SCL, SDA) |
| **Z-Axis AS5600 Encoder** | `I2C4` (400 kHz Fast Mode) | `PB6` (SCL), `PB7` (SDA) | **`J43` (`BL_Touch`)** header (power AS5600 from `J73` 3.3V) |
| **Vision / Console UART** | `USART2` (`115200 8N1`, `DMA1_Stream0` RX / `DMA1_Stream1` TX) | `PD5` (TX), `PD6` (RX) | **`J26` (`UART2`)** 2x5 Raspberry Pi header |
| **Native USB-C CDC** (`usb_cdc` module) | `USB1_OTG_HS` (Full-Speed PHY, `HSI48+CRS`, no VBUS sense) | `PA11` (DM), `PA12` (DP) | **On-board USB-C** port (`J68` fitted for bench 5 V; removed under 24 V) |
| **E-Stop / Button** | `B1` (`EXTI15_10_IRQn`, active-low, on-board 10k pull-up) | `PG15` | **`J34` (`STOP7`)** endstop header |
| **Status / Heartbeat LED** | `LD2` | `PD15` | **`FAN5`** indicator LED `D27` |
| **SWD + SEGGER RTT** | `SYS_JTMS-SWDIO` / `SYS_JTCK-SWCLK` | `PA13` (SWDIO), `PA14` (SWCLK) | **`J72` (`SWD`)** 5-pin header (`3V3`, `SWDIO`, `GND`, `SWCLK`, `RST`) |

> [!NOTE]
> The Octopus Pro `PROBE` input (`PC5`) described in [`docs/hardware/wiring.md`](../../docs/hardware/wiring.md) is **not** configured or read in `MotionFirmware` yet. `TMC_*2` (`MOTOR2_1`) is configured at reset with `ENN2` (`PG5`) and `CS2` (`PC6`) HIGH (disabled) and no active motion profile.

---

## 2. Building All Firmware Targets

A single CMake build compiles the shared STM32H7 HAL once and produces `.elf`, `.hex`, and `.bin` images for **all four profiles** side-by-side in `build/<Preset>/` (`MotionFirmware_Diagnostics.*`, `MotionFirmware_StallGuardCLI.*`, `MotionFirmware_ZBounceEmulation.*`, and `MotionFirmware_UsbCdc.*`), while copying the selected preset's default profile to `MotionFirmware.*`:

```powershell
# Default preset (MotionFirmware.hex = motion_diagnostics; Release preset also available):
cmake --preset Debug
cmake --build --preset Debug

# Interactive StallGuard CLI as default MotionFirmware.hex (Release-StallGuard also available):
cmake --preset Debug-StallGuard
cmake --build --preset Debug-StallGuard

# Z-bounce responder as default MotionFirmware.hex:
cmake --preset Debug-ZBounce
cmake --build --preset Debug-ZBounce

# Standalone TinyUSB USB-C CDC verification app as default MotionFirmware.hex:
cmake --preset Debug-UsbCdc
cmake --build --preset Debug-UsbCdc
```

---

## 3. Bootloader Backup/Restore, Flashing & Real-Time Telemetry

The firmware links directly at `0x08000000` for zero-offset SWD flashing and SEGGER RTT debugging. Before your first SWD flash on a new Octopus Pro board, back up the factory 128 KiB BTT bootloader so you can restore it at any time (a copy of the official 128 KiB BTT H723 bootloader and guide is also archived in `tools/btt_bootloader/`):

```powershell
# 1. Back up the factory 128 KiB BTT bootloader (0x08000000..0x0801FFFF) before first flash:
.\backup_bootloader.cmd     # Saves bootloader_btt_octopus_pro_128k.bin and flash_full_backup_512k.bin

# 2. Flash default MotionFirmware.hex or select a named profile explicitly (SEGGER J-Link SWD):
.\flash.cmd                 # Flashes default MotionFirmware.hex (via flash.jlink)
.\flash.cmd diagnostics     # Flashes MotionFirmware_Diagnostics.hex
.\flash.cmd stallguard      # Flashes MotionFirmware_StallGuardCLI.hex
.\flash.cmd zbounce         # Flashes MotionFirmware_ZBounceEmulation.hex
.\flash.cmd usb_cdc         # Flashes MotionFirmware_UsbCdc.hex
# Or via USB-C DFU (STM32CubeProgrammer CLI, BOOT0 jumper fitted, flashes build\Release or build\Debug MotionFirmware.bin):
.\flash_dfu.cmd
# Or standalone J-Link batch script (expects C:\Program Files\SEGGER\JLink_V978\JLink.exe):
.\flash_jlink.bat

# 3. Stream non-blocking SEGGER RTT Terminal 0 logs (scans RAM_D1 0x24000000), debug, or monitor serial:
.\rtt.cmd                   # Or .\open_rtt_viewer.bat
.\gdb_server.cmd            # Starts JLinkGDBServerCL for STM32H723ZE on SWD 4000 kHz
.\monitor_com9.ps1          # Lightweight PowerShell COM port viewer (default COM9 @ 115200)
.\reset.cmd
.\erase.cmd

# 4. Restore the factory 128 KiB BTT bootloader if needed later:
.\restore_bootloader.cmd
```

> [!NOTE]
> `flash.cmd`, `rtt.cmd`, `reset.cmd`, `erase.cmd`, `gdb_server.cmd`, `backup_bootloader.cmd`, and `restore_bootloader.cmd` invoke `C:\tools\jlink\` targeting `-device STM32H723ZE`, or you can flash/stream RTT from any `.elf` directly with `probe-rs run --chip STM32H723ZETx build\Debug\MotionFirmware.elf`. See [`docs/hardware/wiring.md` Section 6](../../docs/hardware/wiring.md#6-swd-debug-probe-connection-j72) for wiring a NUCLEO-F446RE (`CN2`/`CN4`) J-Link OB probe to Octopus Pro `J72` (`SWD`).

---

## 4. Running Host Unit Tests & Verification Suites

### A. `motion_diagnostics` Host CTest + Cortex-M7 Compile Check (21 tests)
```powershell
.\Apps\motion_diagnostics\Tests\run_tests.ps1
```

### B. `stallguard_cli` Host Closed-Loop Simulation CTest (3 suites; supports `gcc` or `clang`)
```powershell
cmake -S Apps\stallguard_cli\tests -B Apps\stallguard_cli\tests\build-host -G Ninja -DCMAKE_C_COMPILER=clang
cmake --build Apps\stallguard_cli\tests\build-host
ctest --test-dir Apps\stallguard_cli\tests\build-host --output-on-failure
```

### C. Host Python Diagnostics Library & CLI Tests (run from repository root)
```powershell
uv run --project packages\motion_diagnostics pytest packages\motion_diagnostics\tests
uv run --project MotorControls\MotionFirmware\tools\diag pytest MotorControls\MotionFirmware\tools\diag\tests
uv run --project VisionSystem pytest VisionSystem\tests\test_diag_transport.py
```

---

## 5. Portable Native USB-C CDC Transport (`Apps/usb_cdc/`)

Currently, the three motion profiles (`motion_diagnostics`, `stallguard_cli`, `z_bounce_emulation`) communicate over physical `USART2` (`PD5`/`PD6` on `J26`). To let future motion-firmware iterations switch to direct **USB-C Virtual COM (`PA11`/`PA12`)** without touching hardware wiring or CubeMX `.ioc` clocks, the standalone portable module in **[`Apps/usb_cdc/`](Apps/usb_cdc/README.md)** provides:

1. **Vendored TinyUSB v0.18.0 (`MotorControls/Drivers/tinyusb/`) + CMake helper (`usb_cdc_module.cmake`):**
   - Call `add_usb_cdc_module(<target>)` from [`CMakeLists.txt`](CMakeLists.txt) to link only the portable CDC transport (`usb_cdc_port.c`, `usb_descriptors.c`, `tusb_config.h`, and TinyUSB DWC2/CDC sources) without `usb_cdc_app.c`.
2. **Self-Contained `HSI48 + CRS` Clock & `PA9` VBUS Bypass:**
   - Enables `HSI48` + USB SOF `CRS` (`RCC_USBCLKSOURCE_HSI48`) without touching `PLL1` (`400 MHz` CPU / `TIM2` / `TIM5`), and bypasses `PA9` hardware VBUS sensing (`VBDEN=0`, `BVALOEN|BVALOVAL=1`) so USB-C enumerates under both USB-only bench power (`J68` fitted) and 24 V operation (`J68` removed).
3. **ISR Priority & Cooperative Non-Blocking API (`usb_cdc_port.h`):**
   - `OTG_HS_IRQn` runs at priority `5` (never preempting `TIM2`/`TIM5` step ISRs at priority `0`), and `usb_cdc_port_write()`/`read()`/`poll()` never block on host DTR.
   - See **[`Apps/usb_cdc/README.md`](Apps/usb_cdc/README.md)** for full API docs and copy-paste integration adapters for [`uart_transport.c`](Apps/motion_diagnostics/Src/uart_transport.c), [`console.c`](Apps/stallguard_cli/Src/console.c), and [`z_bounce_app.c`](Apps/z_bounce_emulation/Src/z_bounce_app.c).
