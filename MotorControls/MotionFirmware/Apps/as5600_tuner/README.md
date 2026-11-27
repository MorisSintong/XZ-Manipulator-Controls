# AS5600 Tuner & Filter Evaluation Firmware (`MotionFirmware_AS5600Tuner`)

Temporary standalone passive firmware profile for aligning mounted AS5600 magnets (X on `I2C1`, Z on `I2C4`), sweeping volatile on-chip `CONF` filter settings (`SF`, `HYST`, `FTH`), and capturing uniform-cadence NDJSON streams for FFT / digital-filter analysis in MATLAB (`MotorControls/matlab/scripts/analyze_as5600_noise.m`).

## Safety Invariants
- Boots with **all TMC2240 `ENN` pins HIGH** (motors unpowered) and zero `STEP` edges.
- Never writes AS5600 OTP (`0xFF`) or range registers (`ZPOS`/`MPOS`/`MANG`).
- Uses native **USB-C CDC** exclusively (`USART2` untouched).

## Interactive / CLI Commands (over USB-C CDC)
| Command | Purpose |
| :--- | :--- |
| `HELLO` / `HELP` | Print profile banner and command list |
| `STATUS` | Read `RAW`, `CONF`, `SF`, `STATUS` (`MD`/`ML`/`MH`), `AGC` (aim ~64 at 3.3 V), `MAG` for both X and Z |
| `CONF <x\|z> <sf:2\|4\|8\|16> [hyst:0..3] [fth:0..7]` | Set volatile AS5600 filter parameters and read back `CONF` |
| `CAPTURE <x\|z> [rate_hz:50..1000] [n:64..4096]` | Capture `n` uniform-cadence NDJSON samples + summary result (directly readable by `analyze_as5600_noise.m`) |
| `STREAM <x\|z> [rate_hz:10..500]` | Continuous live stream with periodic `AGC`/`MAG`/`STATUS` for magnet airgap alignment |
| `STOP` | Stop active capture or stream |


## Scrutiny Debugger over SEGGER RTT (Channel 0)
`MotionFirmware_AS5600Tuner` integrates `scrutiny-embedded` (`v0.7.2`) over SEGGER RTT channel 0 (dedicated binary channel; no text logs are written to RTT in this profile):
- **Live aliases (`as5600_aliases.json`):** `as5600/{x,z}/{raw,angle,unwrapped,deg,mag,agc,status,conf}` and writable RPVs `as5600/{x,z}/{sf_code,hyst_code,fth_code}` (volatile `CONF` only; `memory_write_enable = false`).
- **Embedded datalogger:** 500 Hz `FixedFrequencyLoopHandler` (`as5600_500hz`, 16 KiB RAM buffer).
- **Post-build SFD:** Building `MotionFirmware_AS5600Tuner` automatically tags the ELF firmware ID (before `.bin`/`.hex` generation) and builds + installs `MotionFirmware_AS5600Tuner.sfd`. This requires `scrutiny` and `arm-none-eabi-c++filt` on `PATH`; if either is missing, CMake emits a warning and falls back to plain `.bin`/`.hex` output. For offline builds, configure with `-DENABLE_SCRUTINY=OFF`.

### Host Connection (J-Link SWD + `rtt-bridge` + Scrutiny)
```powershell
# Terminal 1: Bridge RTT channel 0 <-> UDP 127.0.0.1:19030 (or connect Scrutiny directly to J-Link RTT)
rtt-bridge scrutiny-bridge --device STM32H723ZE --channel 0 --udp-port 19030

# Terminal 2: Start Scrutiny server (UDP link to rtt-bridge)
scrutiny server --options device_link.type=udp device_link.config.host=127.0.0.1 device_link.config.port=19030

# Terminal 3: Open Scrutiny GUI
scrutiny gui
```
## Build & Flash
```powershell
Push-Location MotorControls/MotionFirmware
cmake --preset Release -DCMAKE_C_COMPILER_LAUNCHER=sccache -DCMAKE_CXX_COMPILER_LAUNCHER=sccache
cmake --build --preset Release --target MotionFirmware_AS5600Tuner
.\flash.cmd as5600_tuner
Pop-Location
```
