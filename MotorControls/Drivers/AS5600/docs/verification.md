# Verification evidence and reproduction

## Recorded results

Validation performed on Windows on 2026-09-19 with Clang/LLD 23.1.0,
CMake 4.4.3, Ninja 1.13.2 and Cppcheck 2.21.0. Tests use explicit checks;
they are not disabled by `NDEBUG`.

| Check | Recorded outcome |
| --- | --- |
| Strict C99 Release host build | Passed, warnings treated as errors |
| Portable core suite | 1,492,503 API invocations; 106,840,683 assertions; zero failures |
| HAL/CMSIS adapter suite | 1,333 checks; zero failures |
| Actual host-thread contention | Four workers/four handles; 426 operations; 393 balanced acquisitions/releases |
| Address + undefined-behavior sanitizers | All three host suites passed |
| Clang path-sensitive analyzer | No findings in the three production translation units |
| Cppcheck + MISRA C:2012 add-on | No undispositioned findings under the scoped settings in `standards.md` |
| Real-header target compile | All three modules and both example modules compiled as Cortex-M4 objects |
| Public embedded CMake targets | Core, HAL and CMSIS archives built against real upstream headers |
| Actual FreeRTOS/CMSIS-RTOS2 execution | Passed on Cortex-M4 QEMU, including inheritance, timeout and task switching |
| STM32F446RE + physical AS5600 | **Not executed; hardware acceptance remains required** |

Thread contention counts/timings can vary with host scheduling; the
ownership and completion invariants are asserted independently.

### Production source coverage

Clang source-based coverage from fresh host test profiles:

| Source | Executable lines | Functions | Branch outcomes |
| --- | --- | --- | --- |
| `src\as5600.c` | 533/533 (100%) | 28/28 (100%) | 193/194 (99.48%) |
| `ports\stm32_hal\as5600_stm32_hal.c` | 178/178 (100%) | 11/11 (100%) | 99/100 (99%) |
| `ports\cmsis_rtos2\as5600_cmsis_rtos2.c` | 132/132 (100%) | 7/7 (100%) | 66/66 (100%) |
| **Total** | **843/843 (100%)** | **46/46 (100%)** | **358/360 (99.44%)** |

The two unexercised branch outcomes are defensive internal invariants:
an otherwise successful transaction cannot be inactive, and the HAL
initializer's privately constructed, validated bus cannot fail core
binding. No public error path is excluded from the line-coverage result.
This is ordinary line/function/branch coverage, **not MC/DC or a proof**.

`tools\coverage.ps1` requires 100% production line/function coverage and
at least 99% branch coverage. It uses a separate overwritten profile for
each test and rejects profiles older than their executable, rather than
merging unrelated historical test runs.

### Actual RTOS evidence

The optional `tests\rtos` project links the official CMSIS-FreeRTOS
11.2.0 wrapper/kernel and CMSIS 6.1.0, uses static RTOS allocation, and runs
real SysTick/SVC/PendSV task switching on QEMU's Cortex-M4 `mps2-an386`.
It does not substitute host-thread mocks for the kernel.

The executed scenarios prove a low-priority bus owner's priority
inheritance and disinheritance with a high-priority waiter and ready
medium-priority task; a five-tick acquisition timeout; ownership throughout
settling/readback; unchanged failed outputs; and recovery from injected
NACK/I/O/timeout faults.

The run asserted 70 successful samples, 51 successful configuration
operations, 22 expected failures, **642 transfers and 142 balanced
acquisitions/releases**. Release passed ten consecutive runs and Debug
also passed. See `tests\rtos\validated-run.log` and the
[RTOS test guide](../tests/rtos/README.md).

Only the register bus is simulated in that suite. It does **not** execute
STM32 HAL I2C against an emulated STM32 peripheral or validate physical
sensor timing/electrical behavior. CPU context guards in the STM32 adapter
are covered by the host adapter suite and compiled with real Cortex-M4
intrinsics in the target build.

## What the host tests exercise

- All 65,536 raw word patterns for 12-bit masking, all raw-count conversion
  inputs, valid CONF encodings and reserved-bit combinations.
- Invalid arguments, enum values, reserved output mode, settings/range
  boundaries and wrapping position spans.
- Exact 7-bit/HAL-shifted addresses, register selection, two-byte
  big-endian transfers and the special AS5600 pointer-wrap constraint.
- Read-modify-write preservation, no-op writes, settling, readback mismatch,
  partial write behavior and the absence of OTP commands.
- Magnet errors before/after sample acquisition, contradictory flags and
  intentionally diagnostic-only reads.
- Failure injection at operation phases, partial read buffers, lock failure,
  unlock failure, unchanged outputs and recovery on subsequent operations.
- A single decreasing timeout budget, expired acquisition, late callbacks,
  callback errors, clock rollover and cleanup at deadline boundaries.
- ISR/interrupt-mask/unprivileged rejection, privileged PSP/FP-context
  acceptance, kernel-state restrictions, owner checks and HAL error mapping.
- CMSIS tick rounding at multiple frequencies, unrepresentable waits,
  finite 16-bit-backend caps, and minimum settling across tick phase.
- Actual host concurrency across four handles sharing one bus, with
  contention and injected ordinary transport failures.

Host tests do not determine analog accuracy, magnetic alignment, electrical
noise immunity, hardware WCET or full application safety.

## Reproduce host verification

Run from the workspace root in PowerShell. Compiler and tool executables
must be installed and available through PATH, or supplied by full path.

```powershell
cmake -S . -B .\build -G Ninja -DCMAKE_C_COMPILER=clang -DCMAKE_CXX_COMPILER=clang++ -DCMAKE_BUILD_TYPE=Release
cmake --build .\build
ctest --test-dir .\build --output-on-failure

cmake -S . -B .\build-sanitize -G Ninja -DCMAKE_C_COMPILER=clang -DCMAKE_CXX_COMPILER=clang++ -DCMAKE_BUILD_TYPE=RelWithDebInfo -DAS5600_ENABLE_SANITIZERS=ON
cmake --build .\build-sanitize
ctest --test-dir .\build-sanitize --output-on-failure

cmake -S . -B .\build-coverage -G Ninja -DCMAKE_C_COMPILER=clang -DCMAKE_CXX_COMPILER=clang++ -DCMAKE_BUILD_TYPE=Debug -DAS5600_ENABLE_COVERAGE=ON
cmake --build .\build-coverage
ctest --test-dir .\build-coverage --output-on-failure
.\tools\coverage.ps1
```

The portable library can also be built without C++ or tests by setting
`AS5600_BUILD_TESTS=OFF`. Sanitizer/coverage builds are host-only.

### Static analysis

Cppcheck must include its standard platform files and Python MISRA add-on.
Use an actual Python interpreter, not an uninstalled Windows Store alias.

```powershell
.\tools\analyze.ps1 -Cppcheck cppcheck -Python python
.\tools\analyze.ps1 -Cppcheck cppcheck -Python python -RawFindings
```

The second command **intentionally returns failure** while the documented
advisory/library-ABI findings are present, and saves them to
`build-analysis\raw-findings.txt`. The first applies only the reviewed
scopes in `tools\cppcheck-suppressions.txt` and saves
`build-analysis\analysis.txt`. Project approval of the proposed deviations
is still necessary; see [Standards](standards.md).

The supplementary Clang check is:

```powershell
clang --analyze -std=c99 -Wall -Wextra -Wpedantic -Xanalyzer -analyzer-output=text -I .\include -I .\ports\stm32_hal -I .\ports\cmsis_rtos2 -I .\tests\fakes .\src\as5600.c .\ports\stm32_hal\as5600_stm32_hal.c .\ports\cmsis_rtos2\as5600_cmsis_rtos2.c
```

Cppcheck/Python were installed only in session-local tooling storage for
this execution; no global installation or PATH change was made.

## Real Cortex-M4 compilation

This standalone build downloads immutable, hash-checked official dependency
archives under `build-target\_deps`, retaining their license files. It
compiles the production sources and both board examples, **not a complete
Nucleo firmware image**:

```powershell
$toolchain = (Resolve-Path .\cmake\arm-clang.cmake).Path
cmake -S .\tests\vendor_compile -B .\build-target -G Ninja "-DCMAKE_TOOLCHAIN_FILE=$toolchain"
cmake --build .\build-target
```

The objects were inspected as **32-bit little-endian ARM ELF**, built with
`-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard -ffreestanding`.
Real CMSIS intrinsics were used, not the host stubs.

At Clang `-Os`, the production object totals were 3,611 bytes of text/rodata
and no mutable library `.data`/`.bss`. This excludes caller-owned contexts,
HAL/kernel code, compiler runtime helpers, stacks and the final link.
The generated `.su` files report individual static stack frames; they are
not a whole-system stack bound.

| Dependency | Version | Pinned commit |
| --- | --- | --- |
| [STM32F4 HAL](https://github.com/STMicroelectronics/stm32f4xx-hal-driver) | v1.8.5 | `b6f0ed3829f3829eb358a2e7417d80bba1a42db7` |
| [STM32F4 CMSIS device](https://github.com/STMicroelectronics/cmsis-device-f4) | v2.6.11 | `0fa0e489e053fa1ca7790bb40b4d76458f64c55d` |
| [CMSIS Core/RTOS2](https://github.com/ARM-software/CMSIS_6) | v6.1.0 | `b0bbb0423b278ca632cfe1474eb227961d835fd2` |
| [CMSIS-FreeRTOS](https://github.com/ARM-software/CMSIS-FreeRTOS) | v11.2.0 | `c7e7294e78a80be862340d8d50edd515ac3c41fd` |

The HAL uses BSD-3-Clause, the STM32 CMSIS device and Arm CMSIS components
use Apache-2.0, and the bundled FreeRTOS kernel uses MIT; preserve the
upstream notices. The source archives are build dependencies, not copied
into the authored driver.

CMSIS-FreeRTOS 11.2.0 expects CMSIS 6; the test pins match. A CubeMX project
may ship different versions. Keep the generated HAL/kernel/header set
internally consistent and rerun these checks when changing versions.

For actual FreeRTOS execution, follow `tests\rtos\README.md`; it documents
the portable QEMU tool, its checksum, build flags and bounded runner.
The already configured local run can be repeated with:

```powershell
ctest --test-dir .\tests\rtos\build --output-on-failure
```

## Physical-board acceptance: not yet performed

Do not treat software-only evidence as a board release. With the actual
Nucleo, sensor, magnet and intended firmware, record at least:

| ID | Test and acceptance evidence |
| --- | --- |
| H-01 | Measure supply/pull-up voltages, rise times and I2C timing. Confirm address 0x36, repeated START and separate two-byte angle reads on a logic analyzer at 100 kHz and the intended final speed. |
| H-02 | Check known mechanical positions, direction, full-turn wrap and raw/ANGLE differences against the datasheet and fixture accuracy. Check AGC/magnitude and magnet placement. |
| H-03 | Remove/misalign the magnet and exercise weak/strong conditions. Checked samples must fail without publishing fresh data; diagnostics must remain available when transport works. |
| H-04 | Exercise absent device/NACK, interrupted transfer and safely fixture-held SCL/SDA. Check reported errors, released locks, no implicit GPIO/controller reset and controlled application recovery. |
| H-05 | Run multiple priorities plus another I2C1 device with the same mutex. Verify no bus interleaving, priority inheritance, finite contention waits and healthy acquisition after an ordinary fault. |
| H-06 | Verify rejected ISR, masked, unprivileged and scheduler-locked calls leave peripheral/interrupt state untouched. Confirm privileged PSP tasks work normally. |
| H-07 | Apply volatile settings/range changes, readback mismatch and partial-pair fault cases. Suspend calibrated output use after failure; verify restoration before resuming. Confirm power-cycle behavior and unchanged burn count. |
| H-08 | Confirm the HAL tick stays live during preemption and RTOS delays. Review tickless/low-power behavior, run a controlled rollover test, measure worst-case latency and size an independent watchdog. |
| H-09 | Run sustained loaded operation with stack-overflow checks/high-water marks, bus faults and the intended filter/power settings. Establish application-specific latency, freshness and error-response limits. |

Define numeric timing and accuracy acceptance limits from the application's
requirements and the datasheet before those tests. This driver cannot
establish universal control-loop or functional-safety limits for an
unspecified application.
