# STM32F446RE Motion Control Firmware

Firmware application for the 2-Axis (XZ) Cartesian Manipulator running on the **STM32F446RE Nucleo-64** board.

---

## 1. Overview & Peripherals

The firmware integrates:
- **TMC2240 Stepper Motor Drivers:** Interfaced via high-speed SPI (`SPI2` @ 2.625 MHz, SPI Mode 3) with dedicated Chip Select lines (`PC0` for Z-Axis, `PC3` for X-Axis).
- **StallGuard4 Sensorless Bouncing/Homing:** Automatic mechanical limit detection on the lead screw and belt axes without requiring physical limit switches.
- **AS5600 12-Bit Encoders:** Dual hardware I2C channels (`I2C1` on PB8/PB9, `I2C3` on PA8/PC9) for independent rotary feedback.
- **Dual Telemetry Broadcast:**
  - **USART2:** CDC Virtual COM Port (`COM9` @ 115200 baud, 8-N-1).
  - **SEGGER RTT:** High-speed real-time terminal output (Buffer 0) via SWD.

---

## 2. Hardware Wiring

Complete pin assignments, SPI/I2C connections, and power distribution diagrams are located in:  
👉 **[`docs/hardware/wiring.md`](../../docs/hardware/wiring.md)**

---

## 3. Toolchain & Prerequisites

Make sure the following tools are installed and present in your system `PATH`:
- **Arm GNU Toolchain** (`arm-none-eabi-gcc` 15.3+)
- **CMake** (v3.22+)
- **Ninja** build system
- **SEGGER J-Link** or **ST-Link** utility / `probe-rs`

---

## 4. Building the Firmware

From this directory (`MotorControls/MotionFirmware/`):

```powershell
# 1. Configure the build with CMake and Ninja using arm-none-eabi toolchain
cmake -B build -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake -DCMAKE_BUILD_TYPE=Debug

# 2. Compile the ELF and HEX firmware binaries
ninja -C build
```

The output artifacts will be generated in `build/`:
- `MotionFirmware.elf`
- `MotionFirmware.hex`
- `MotionFirmware.bin`

---

## 5. Flashing & Debugging

Automated batch and command scripts are provided in this directory for rapid flashing and debugging:

### Flashing via SEGGER J-Link
```powershell
.\flash_jlink.bat
# Or using the direct command wrapper:
.\flash.cmd
```

### Full Chip Erase
```powershell
.\erase.cmd
```

### Reset Target MCU
```powershell
.\reset.cmd
```

### Real-Time Telemetry & Monitoring

1. **SEGGER RTT Terminal:**
   ```powershell
   .\open_rtt_viewer.bat
   # Or run direct RTT logger:
   .\rtt.cmd
   ```

2. **USART2 COM9 Serial Monitor:**
   ```powershell
   powershell -ExecutionPolicy Bypass -File .\monitor_com9.ps1
   ```

3. **GDB Server:**
   ```powershell
   .\gdb_server.cmd
   ```
