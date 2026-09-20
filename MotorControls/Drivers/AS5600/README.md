# AS5600 driver for STM32F446RE

A portable, allocation-free C99 driver with an STM32CubeF4 polling-I2C port
and a CMSIS-RTOS2 synchronization adapter for FreeRTOS. The board integration
targets the **NUCLEO-F446RE**, I2C1 on **PB8 / D15 (SCL)** and
**PB9 / D14 (SDA)**.

The implementation follows a MISRA C:2012-oriented coding and verification
process. It is **not a certified or unconditional MISRA-compliant component**:
the scoped deviations, analyzer limitations, and application obligations are
documented in [Standards](docs/standards.md). Physical-board acceptance is
separate from host testing and target compilation; see
[Verification](docs/verification.md).

## Included

| Location | Purpose |
| --- | --- |
| `include\as5600.h`, `src\as5600.c` | Portable sensor protocol and checked API |
| `ports\stm32_hal` | STM32F4 HAL adapter; synchronous memory transfers |
| `ports\cmsis_rtos2` | Caller-owned mutex, ownership checks, tick conversion |
| `examples\nucleo_f446re` | Bare-metal and CMSIS-FreeRTOS binding/polling examples |
| `tests` | Exhaustive, fault-injection, threaded, actual FreeRTOS/QEMU and target-compile checks |
| `tools` | Repeatable static-analysis and coverage checks |

Features include raw and scaled angles, magnet diagnostics, a magnet-checked
sample, settings readback, and **volatile** configuration/range writes.
Writes preserve undocumented factory bits, wait for register-setting
propagation, and verify readback. Initialization does not overwrite the
sensor's existing or OTP-loaded configuration.

**There is no OTP/burn API, arbitrary register-write API, automatic retry,
automatic bus-reset sequence, DMA API, or ISR-mode transfer API.** Permanent
programming needs a separate controlled manufacturing procedure. This is an
AS5600 driver, not an AS5600L driver.

## Board setup

Use a **3.3 V-compatible AS5600 circuit**, common ground, and external SDA/SCL
pull-ups to **3.3 V**. Do not assume a breakout powered from 5 V also has
3.3 V-safe pull-ups. On a bare AS5600, follow the datasheet's 3.3 V supply
connection, including both supply pins and local decoupling; check the
schematic of a breakout before connecting it.

| Signal | NUCLEO-F446RE | AS5600 |
| --- | --- | --- |
| SCL | D15 / PB8, AF4 | SCL |
| SDA | D14 / PB9, AF4 | SDA |
| Supply | 3V3 | Correctly wired 3.3 V supply circuit |
| Ground | GND | GND |
| Direction | GND for the datasheet's clockwise-positive direction | DIR |

Keep PGO in its normal, pulled-high state rather than enabling OUT-pin
programming. OUT is not needed for I2C readout. Use a **diametrically
magnetized on-axis magnet** and check the magnet-status flags and air gap.
Start with short wiring and 100 kHz; select pull-ups for the actual bus
capacitance and measure rise time before increasing speed.

In STM32CubeMX:

1. Select **NUCLEO-F446RE**, I2C1, PB8/PB9 AF4 open-drain, and **7-bit
   addressing**. Choose 100 kHz initially, or at most **400 kHz** for this
   classic I2C1 HAL port. The sensor's 1 MHz capability does not change that
   port limit.
2. Enable the HAL I2C module and its dependencies. Include the normal
   `STM32F446xx` and `USE_HAL_DRIVER` definitions. Keep HAL's `USE_RTOS`
   setting at **0**; the external mutex supplies synchronization.
3. For FreeRTOS, select **CMSIS-V2**, enable mutexes and static allocation,
   and use **32-bit kernel ticks**. A 1 kHz kernel tick is a convenient
   starting point, but the adapter converts other supported tick rates.
4. For FreeRTOS, use a **dedicated 1 ms HAL timebase**, for example TIM6,
   leaving SysTick to the kernel. Keep that timebase running during every
   transaction and wait. See the [RTOS integration contract](docs/integration.md).

The sensor address is **0x36, 7-bit**. The HAL adapter alone shifts it to
**0x6C**. Do not shift it again.

## Add the source files

For bare metal, add `src\as5600.c` and
`ports\stm32_hal\as5600_stm32_hal.c` to your CubeIDE project, with their two
header directories on the include path.

For FreeRTOS, also add
`ports\cmsis_rtos2\as5600_cmsis_rtos2.c` and its header directory.
Use your project's generated HAL configuration, real STM32/CMSIS headers,
and HAL/RTOS implementations. **Never add `tests\fakes` or the
`tests\vendor_compile` configuration headers to firmware.**

The two small example modules can be included as-is, or their binding code
can be adapted when the application needs direct access to a driver handle
for configuration.

### Bare-metal example

Add `as5600_example_baremetal.c` and include its header. After `HAL_Init()`,
clock/GPIO configuration and `MX_I2C1_Init()`, with interrupts enabled:

```c
as5600_result_t result;
as5600_sample_t sample = {0};

HAL_Delay(AS5600_POWER_UP_MS);
result = as5600_example_baremetal_bind(&hi2c1);
if (result != AS5600_OK)
{
    Error_Handler();
}

result = as5600_example_baremetal_poll(&sample);
if (result != AS5600_OK)
{
    Error_Handler();
}
/* Consume sample.raw_angle only after checking result. */
```

Binding is a one-time lifecycle operation, not a sensor probe. The first
read detects transport and magnet faults. These snippets use Cube's
`Error_Handler()` as a deliberately simple halt-on-error policy; a real
application should implement its own fault reporting and recovery.

### FreeRTOS example

Add `as5600_example_rtos.c` and include its header, plus `FreeRTOS.h`.
After hardware initialization and the sensor power-up delay, call
`osKernelInitialize()` and create **one mutex for the entire I2C1 bus**:

```c
static StaticSemaphore_t i2c1_mutex_storage;
static const osMutexAttr_t i2c1_mutex_attributes = {
    .name = "i2c1",
    .attr_bits = osMutexPrioInherit,
    .cb_mem = &i2c1_mutex_storage,
    .cb_size = sizeof(i2c1_mutex_storage)
};
osMutexId_t i2c1_mutex = osMutexNew(&i2c1_mutex_attributes);

if (i2c1_mutex == NULL)
{
    Error_Handler();
}
if (as5600_example_rtos_bind(&hi2c1, i2c1_mutex) != AS5600_OK)
{
    Error_Handler();
}
```

Call `as5600_example_rtos_poll()` **from a running, privileged task after
`osKernelStart()`**, with a task-local sample object, and check its result.
All other I2C1 clients must acquire that same mutex. Do not acquire it again
around the driver call: the driver owns acquisition and release for the
whole operation.

The driver and adapters allocate nothing. Creating static tasks, providing
the kernel's idle/timer-task storage hooks, and configuring interrupt
priorities remain application/CubeMX responsibilities.

## API behavior that matters

| API | Contract |
| --- | --- |
| `as5600_read_raw_angle` | Unscaled 12-bit count; transport success alone does not prove magnet validity |
| `as5600_read_angle` | Sensor's scaled ANGLE register, including its endpoint hysteresis |
| `as5600_read_diagnostics` | STATUS, AGC and magnitude, including unhealthy magnet states |
| `as5600_read_sample` | Rejects missing/weak/strong magnet flags both before and after sequential readings |
| `as5600_read_settings` | Volatile ZPOS, MPOS, MANG, decoded CONF and two-bit ZMCO |
| `as5600_write_config` | Validated complete profile, reserved-bit-preserving RMW and verification |
| `as5600_write_positions` | ZPOS/MPOS pair; wrapped nonzero span must be at least 205 counts |
| `as5600_write_max_angle` | MANG: 0 for the default full range, or 205..4095 counts |
| `as5600_counts_to_millidegrees` | Rounded integer conversion for a raw full-turn count |

All hardware operations require a positive finite millisecond budget.
Zero and values above `AS5600_TIMEOUT_MAX_MS` are rejected. **Read outputs
remain unchanged on any failure**, including a failed unlock; they must not
be treated as fresh data after an error.

Raw/ANGLE/magnitude are always read as separate two-byte transactions, not
one burst across the sensor's special auto-wrapping registers. A checked
sample is **not a simultaneous hardware snapshot**, a freshness detector,
or a redundant safety measurement.

Each high-level operation acquires the shared bus once. Callers may share
an initialized driver, but must use separate output objects and must not
modify/reinitialize its configuration or port storage concurrently.
Full contracts, timeout limitations and write-failure handling are in
[Integration](docs/integration.md).

## Run host tests

With CMake, Ninja, Clang and Clang++ available in PowerShell:

```powershell
cmake -S . -B .\build -G Ninja -DCMAKE_C_COMPILER=clang -DCMAKE_CXX_COMPILER=clang++ -DCMAKE_BUILD_TYPE=Release
cmake --build .\build
ctest --test-dir .\build --output-on-failure
```

No test framework download is needed for the host tests. Sanitizer,
coverage, static-analysis, real-kernel emulation, real-header Cortex-M4
compilation, and hardware acceptance commands are in
[Verification](docs/verification.md).

For an existing CMake firmware build, enable `AS5600_BUILD_STM32_HAL` and,
if needed, `AS5600_BUILD_CMSIS_RTOS2`; supply the HAL/CMSIS/application
directories through `AS5600_VENDOR_INCLUDE_DIRS`. The HAL target exports
`STM32F446xx` by default, selectable through `AS5600_STM32_DEVICE`.
Link the application with the matching HAL and kernel, not just the driver
archives.

## References

The protocol source is the supplied
`datasheets\infineon-as5600-datasheet-en.pdf`: legacy AS5600 datasheet
v1-06, 2018-06-20, supplied with an Infineon rebranding notice.
Important sections are pp. 7, 9-13, 18-24 and 31.
Pinned ST/Arm source revisions used for integration are listed in
[Verification](docs/verification.md).
