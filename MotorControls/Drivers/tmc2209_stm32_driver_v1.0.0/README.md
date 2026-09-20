# TMC2209-UART STM32 Driver

A **C HAL driver** for the **TMC2209 stepper motor driver**, with bare-metal
and FreeRTOS backends. **STM32F446RE is the software-validation target**;
other STM32 families are unverified.

The C11 implementation is audited against the bundled **TMC2209 Datasheet
Rev. 1.08**. Host tests do not establish physical UART timing, electrical
margins, motor-current accuracy, or hardware readiness. Release claims require
the complete recorded build, test, analysis, and coverage gates.

---

## Key Features

- **Pure C Driver (No Arduino dependencies)**: Native STM32CubeIDE & CubeMX HAL support (`stm32f4xx_hal.h`, `main.h`, etc.).
- **Half-Duplex Single-Wire UART**: Bounded F446 TX-complete/RX polling, error-flag recovery, interframe idle, and immediate reply reception. Physical timing still requires bench measurements.
- **Dual OS Backend Support**:
  - `tmc2209_os_none.c`: Super-loop bare-metal (zero dynamic allocation, zero OS overhead).
  - `tmc2209_os_freertos.c`: FreeRTOS multi-tasking with priority-inheritance bus mutexes and microsecond/millisecond scheduling.
- **Datasheet-Verified CRC-8 (Poly 0x07)**: Precomputed start CRCs for slave addresses `0..3` and master reply `0xFF`.
- **Typed Register Helpers**: Convenience access to `GCONF`, `SLAVECONF`, `IHOLD_IRUN`, `CHOPCONF`, `PWMCONF`, `COOLCONF`, `TPWMTHRS`, `TCOOLTHRS`, `SGTHRS`, `TPOWERDOWN`, `IOIN`, `SG_RESULT`, `IFCNT`, `GSTAT`, and `DRV_STATUS`. This is not a claim that every chip feature has a typed helper.
- **Desired Mirrors**: `UINT32_MAX` retries an existing desired register mirror; before a first request it returns `ERR_STATE`. A desired mirror is not confirmation of hardware state.
- **Multi-Motor Bus Sharing**: Connect up to 4 TMC2209 ICs (`addr 0..3`) on a single UART wire; bus mutex ensures atomic datagram transactions.
- **Production-linked Host Tests**: Protocol/core, real HAL adapter, and real OS-adapter logic are tested separately against controlled dependencies. Read actual runner results rather than a hard-coded assertion total.

---

## Directory Structure

```
TMC2209-UART/
├── Inc/                           ← C Header files (Public & Abstraction API)
│   ├── tmc2209.h                  ← High-level lifecycle & typed helpers
│   ├── tmc2209_ll.h               ← Low-level CRC & datagram packing/unpacking
│   ├── tmc2209_reg.h              ← Datasheet register map, bitfield unions & defaults
│   ├── tmc2209_unit.h             ← Bus (UART+lock) and Unit (IC mirror) definitions
│   ├── tmc2209_port.h             ← UART transport port abstraction interface
│   ├── tmc2209_os.h               ← OS abstraction interface (mutex, sem, delay, tick)
│   └── tmc2209_types.h            ← Status codes, sentinels & config inclusion
├── Src/                           ← C Source files
│   ├── tmc2209.c                  ← Checked lifecycle, explicit configuration and activation
│   ├── tmc2209_ll.c               ← Mutex-protected raw R/W, CRC & datagrams
│   ├── tmc2209_port_hal.c         ← STM32 HAL half-duplex UART driver (DWT µs timer)
│   ├── tmc2209_os_none.c          ← Bare-metal OS backend (super-loop no-ops)
│   └── tmc2209_os_freertos.c      ← FreeRTOS OS backend (priority inheritance mutex)
├── Config/
│   └── tmc2209_conf_template.h    ← User project configuration template
├── Examples/
│   ├── baremetal_f446re/          ← Standalone polling example
│   │   └── app_tmc2209_example.c
│   └── freertos_f446re/           ← Multi-task concurrent bus sharing example
│       └── app_tmc2209_rtos_example.c
├── Tests/                        ← Production-linked tests, HAL/RTOS fakes, and CMake targets
├── docs/
│   └── TMC2209-Datasheet.pdf      ← Official Trinamic TMC2209 datasheet
├── run_tests.bat                  ← 1-click test runner (GCC host tests)
├── instruction.md                 ← Step-by-step STM32CubeIDE & CubeMX setup guide
└── LICENSE                        ← GPLv3 License
```

---

## Quick Start (Bare-Metal Diagnostics)

Configure the real UART and external ENN inhibit first. Copy the bare-metal
example's `.c` and `.h` into the application and check its returned statuses:

```c
#include "main.h"
#include "app_tmc2209_example.h"

void App_Init(void)
{
    if (TMC2209_AppInit() != TMC2209_OK) {
        Error_Handler();
        return;
    }
}

void App_Loop(void)
{
    if (TMC2209_AppPoll() != TMC2209_OK) {
        Error_Handler();
        return;
    }
    HAL_Delay(100);
}
```

This initializes/probes communication only. It does not configure current or
activate a motor. Supply an explicit `tmc2209_motor_config_t` through
`TMC2209_AppConfigureMotor`, handle startup status deliberately, and use a
separate checked `TMC2209_AppActivate` action before releasing ENN.
The application's error handler must inhibit the motor and stop its STEP
source; disabling interrupts alone does not stop a running hardware timer.

`TMC2209_SetupDefault` and universal motor presets have been removed. Read
`instruction.md` for the complete checked lifecycle, profile fields, raw-access
restrictions, and migration requirements.

---

## Running Host Tests

A package-local CMake/CTest runner is available on Windows. It requires CMake,
Ninja, and Clang and resolves source paths relative to the script:

```cmd
run_tests.bat
```

An optional first argument selects its out-of-source build directory. A passing
host run is not an ARM build or hardware qualification.

From the enclosing Drivers workspace, run the cross-package strict host gate:

```powershell
.\run_tests.ps1 -Strict
.\run_tests.ps1 -Strict -Configuration Release
```

This also checks sanitizers and per-layer coverage. A required coverage failure
produces a failing exit code even when individual tests pass. Static analysis
uses `run_static_analysis.ps1` with that build directory. ARM integration and
hardware qualification are separate gates.

---

## License

GNU General Public License v3 (GPLv3).  
Original C++ library by Anton Khrustalev (2023). Ported, hardened, and expanded for STM32 C environments.
