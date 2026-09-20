# Actual CMSIS-FreeRTOS integration test

This optional, standalone test executes the **official FreeRTOS 11.2.0 kernel,
official CMSIS-RTOS2 wrapper, and real Cortex-M4 exception/task switching** in
QEMU's `mps2-an386` machine. Only the AS5600 register bus is simulated.
It does **not** validate STM32F446RE peripherals, HAL I2C, electrical behavior,
interrupt priorities in an STM32 application, DMA, or hardware timing.

## Build and run on Windows

Prerequisites: Clang/LLD with an `arm-none-eabi` target, CMake 3.24+, Ninja,
PowerShell 7, and a QEMU executable supporting `mps2-an386`. Arm GCC/newlib
is not required. From the AS5600 workspace:

```powershell
$qemu = 'C:\path\to\xpack-qemu-arm-9.2.4-1\bin\qemu-system-arm.exe'
cmake -S tests\rtos -B tests\rtos\build -G Ninja `
  '-DCMAKE_TOOLCHAIN_FILE=clang-arm.cmake' '-DCMAKE_BUILD_TYPE=Release' `
  "-DAS5600_QEMU=$qemu"
cmake --build tests\rtos\build --parallel 4
ctest --test-dir tests\rtos\build --output-on-failure --verbose
```

For a repeatability check, add `--repeat until-fail:10` to the CTest command.
Alternatively, invoke `tests\rtos\run.ps1 -Qemu $qemu` directly after building.
The runner requires both exit code zero and `RESULT: PASS`; failures cannot
silently pass. It captures output in `tests\rtos\build\qemu.log`.
Firmware exits through Arm semihosting. A 5,000-tick in-kernel watchdog and a
30-second external watchdog bound execution; CTest adds a 40-second limit.
The runner terminates only its own QEMU PID if the external watchdog expires.
QEMU instruction-count timing makes the schedule repeatable, not a hardware
performance measurement. The disconnected `lan9118.0` warning is harmless;
the runner disables network peers and does not use networking.

### Pinned dependencies

CMake downloads unmodified upstream archives into the ignored `build\_deps`
directory, verifies their SHA-256 hashes, and retains their licenses:

| Dependency | Release | Commit |
| --- | --- | --- |
| [CMSIS_6](https://github.com/ARM-software/CMSIS_6) | v6.1.0 | `b0bbb0423b278ca632cfe1474eb227961d835fd2` |
| [CMSIS-FreeRTOS](https://github.com/ARM-software/CMSIS-FreeRTOS) | v11.2.0 | `c7e7294e78a80be862340d8d50edd515ac3c41fd` |

Archive hashes are recorded in `CMakeLists.txt`. No vendor implementation is
copied into the test sources. For an offline build, supply the usual
`FETCHCONTENT_SOURCE_DIR_CMSIS6` and `FETCHCONTENT_SOURCE_DIR_CMSIS_FREERTOS`
CMake overrides pointing to previously verified checkouts of these commits.

Validated emulator:
[xPack QEMU Arm 9.2.4-1 Windows x64 ZIP](https://github.com/xpack-dev-tools/qemu-arm-xpack/releases/download/v9.2.4-1/xpack-qemu-arm-9.2.4-1-win32-x64.zip),
SHA-256 `f029d6549fabe5b0ddce07921832bb97a20f56d54b63c8f4d3d5e82c3c8eae33`.
Extract it to an isolated tools directory and pass the executable's full path;
no installer, registry change, global installation, or PATH edit is needed.

## What the assertions prove

- A low-priority task owns the bus **inside `as5600_read_sample`**. A real
  higher-priority contender blocks on that mutex. The owner inherits the
  higher priority, continues despite a ready medium-priority task, and
  disinherits on release. The waiting task then completes successfully.
- Another contender times out after five ticks while the owner legitimately
  sleeps with the mutex held. Output is byte-for-byte unchanged, there is no
  I/O or unlock after failed acquisition, ownership remains intact, the owner's
  priority drops after the waiter times out, and subsequent operations succeed.
- NACK, I/O error, and transfer timeout are injected independently at every one
  of the six sample reads. Partial callback output never escapes a failed API
  call. All 18 cases release the lock and recover on the next call.
- Configuration failures at initial read, write, and readback recover without
  reinitialization. A failed readback is not assumed to undo a completed write.
- Three genuine kernel tasks perform 48 sample reads and 48 configuration
  updates at different priorities. Callbacks assert actual kernel mutex
  ownership, task identity, transfer sequence, decreasing timeout budget, and
  ownership across configuration settling and readback. Reserved bits survive.
- Exact aggregate counts are checked: 70 successful samples, 51 successful
  configuration operations, 22 expected failures, 642 transfers, and 142
  balanced acquisitions/releases. Stack guards and stack headroom are checked.
  Pre-scheduler calls are also rejected with unchanged output and no I/O.

Every RTOS object, including the priority-inheriting nonrecursive mutex, tasks,
stacks, and idle task, uses static storage. `configSUPPORT_DYNAMIC_ALLOCATION`
is zero; no heap implementation is linked. The local freestanding runtime
provides only memory functions, semihosting, and an unsigned-division ABI shim
(checked against quotient/remainder vectors before kernel start).

The test configuration uses 56 CMSIS priorities, 32-bit ticks at 1 kHz, no
optimized priority selection, and the upstream GCC `ARM_CM4F` port compiled
with Clang. It is a **test configuration**, not an STM32 deployment template.

## Recorded execution

On Windows with Clang/LLD 23.1.0, CMake 4.4.3, Ninja 1.13.2, and xPack QEMU
9.2.4, the Release test passed in approximately 1.5 seconds, reporting 316
scheduler switches and 135 ticks. Ten consecutive Release runs passed; a
separate unoptimized Debug build also passed (316 switches, 138 ticks).
See `validated-run.log` for the captured firmware output.
Real STM32F446RE board testing remains required.
