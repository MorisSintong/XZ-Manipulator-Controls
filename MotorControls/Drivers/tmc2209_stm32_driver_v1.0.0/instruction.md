# TMC2209 STM32F446RE Integration Guide

This is a breaking, checked-API revision of the C11 driver. It targets the
STM32F446RE software integration. Other STM32 families and real electrical,
timing, motor-current, thermal, and mechanical behavior are unverified.

The driver configures the TMC2209 over half-duplex PDN_UART. STEP/DIR generation
and an independent ENN/stop mechanism belong to the application. No transport
initialization routine automatically selects motor current or enables TOFF.

## Hardware and Peripheral Setup

Keep ENN inactive-high using an appropriate external bias/inhibit during MCU
reset, setup, and faults. Transport initialization and deinitialization cannot
guarantee that physical outputs are off.

| Signal | Connection / requirement |
|---|---|
| PDN_UART | Dedicated STM32 USART TX pin configured for single-wire half-duplex |
| GND | Common MCU, module, and power-supply reference |
| VIO / VM | Actual module specifications; do not infer board ratings from IC limits alone |
| MS1 / MS2 | Low/low = address 0; high/low = 1; low/high = 2; high/high = 3 |
| ENN / STEP / DIR | Application-controlled; not managed by this UART library |

Use CubeMX's actual **Single Wire (Half-Duplex)** peripheral configuration,
8 data bits, no parity, one stop bit, no hardware flow control. The generated
initialization must use `HAL_HalfDuplex_Init`; an ordinary full-duplex UART
configuration is not silently repaired by this driver. Follow the F446
reference manual's half-duplex GPIO/pull-up requirements and the module's
wiring guidance; qualify line voltage and turnaround with hardware instruments.

Start integration at 115200 baud. The supported software parameter range is
9000 through 500000 baud, subject to timeout constraints and the chip clock.
This is not a claim that every wiring arrangement works across that range.
On NUCLEO-F446RE, USART2 is normally ST-LINK VCP; use a different dedicated
USART, for example USART1, for the driver.

Do not share that USART with debug logging, DMA, interrupt-driven UART calls,
or another HAL user. Keep the HAL timebase, DWT counter, and SystemCoreClock
valid. A stopped counter/timebase or continuously asserted RX flag must be
treated as an error, not a successful transfer.

## Files and Build Configuration

Copy `Inc` and `Src` into the application and put `Inc` on its include path.
Compile:

```text
tmc2209.c
tmc2209_ll.c
tmc2209_port_hal.c
```

Add **exactly one** of `tmc2209_os_none.c` or `tmc2209_os_freertos.c`.
Use the real STM32F4 HAL/CMSIS headers with `STM32F446xx` and `USE_HAL_DRIVER`.
Never copy the host-test fake headers into firmware.

Copy `Config\tmc2209_conf_template.h` as `tmc2209_conf.h` on the application
include path. For bare metal select `TMC2209_OS_NONE=1` and
`TMC2209_OS_FREERTOS=0`; reverse them for FreeRTOS. Invalid combinations fail
compilation.

| Configuration | Contract |
|---|---|
| `TMC2209_DEFAULT_BAUD` | Default declaration; the bus argument and actual UART configuration must match |
| `TMC2209_UART_TX_TIMEOUT_MS`, `TMC2209_UART_RX_TIMEOUT_MS` | Finite whole-frame limits, 1 through 1000 ms |
| `TMC2209_BUS_LOCK_TIMEOUT_MS` | 0 through 60000 ms |
| `TMC2209_BUS_IDLE_US` | Optional additional minimum interframe idle; not a delay before receiving a reply |
| `TMC2209_POLL_LIMIT`, `TMC2209_RX_DRAIN_LIMIT` | Secondary finite bounds for stuck timing/RX conditions |
| `TMC2209_OS_CREATE_MUTEX` | Driver-created native mutex, or explicit externally owned mutex |

RX timeout must cover the maximum SENDDELAY plus reply (200 bit times) and
the specified margin. Increase it for low baud rates. The driver observes
at least 80 bit times of interframe idle for parser recovery and TX release.
It begins receiving a reply immediately after turnaround.

The old `TMC2209_REPLY_GAP_US` and `TMC2209_REPLY_POLL_US` settings are rejected.
Remove them rather than preserving an unsafe pre-receive delay.

## Lifecycle and Explicit Motor Configuration

All bus and unit storage must be zero-initialized before first use:

```c
static tmc2209_bus_t bus;
static tmc2209_unit_t unit;
```

Never copy a live object. Use one bus per physical UART and one unit per
address on that bus. Lifecycle operations need exclusive access.

1. Hold external ENN disabled, initialize the MCU UART, then check
   `TMC2209_BusInit(&bus, &huart1, 115200U)`.
2. Check `TMC2209_UnitInit(&unit, &bus, address)`.
3. Check `TMC2209_Available(&unit, &present)`. VERSION must be `0x21`.
   Presence is not a motor-health guarantee.
4. Observe GSTAT and DRV_STATUS. Deliberately acknowledge a handled startup
   reset using `TMC2209_ClearGSTAT(&unit, 0x01U)` when appropriate.
   Acknowledge before Configure, never between Configure and Activate.
   Do not periodically clear faults and continue motion.
5. Supply an application-owned `tmc2209_motor_config_t` and check
   `TMC2209_Configure(&unit, &profile)`. This is not `SetupDefault`.
6. On a separate start decision, check `TMC2209_Activate(&unit, chosen_toff)`.
   Only then may the application release ENN and issue bounded motion.
7. On a fault or shutdown, use the external stop/inhibit first and check
   `TMC2209_Deactivate`. Deinitialize units before the bus.

Configuration requires all ten fields:

| Field | Important requirement |
|---|---|
| `gconf` | Set PDN_DISABLE and MSTEP_REG_SELECT; prohibited/test bits are rejected |
| `slaveconf` | SENDDELAY at least 2 for the supported shared-bus configuration |
| `ihold_irun` | Explicit motor-dependent current codes and hold delay |
| `chopconf` | TOFF must be zero; choose valid microstep/current-sense/hysteresis fields |
| `pwmconf` | Valid automatic-current/tuning settings for the intended mode |
| `coolconf` | Explicit CoolStep choice; current minima and operating conditions apply |
| `tcoolthrs`, `tpwmthrs` | 20-bit period thresholds, not universal RPM values |
| `sgthrs` | 8-bit threshold |
| `tpowerdown` | 8-bit delay; automatic PWM tuning requires at least 2 |

There is no universally safe current preset. A current code represents its
encoded fraction of the board's selected full scale, not a known current in
amperes. For example, IRUN code 15 represents 16 of 32 current units, not code
16. Determine actual current from the board/current-sense configuration.

Configure verifies identity before programming, confirms TOFF zero first,
forces VACTUAL zero for STEP/DIR operation, and verifies configuration writes.
Nonzero UART velocity is deliberately outside the qualified interface. A
failed disable command cannot prove that bridges are off; ENN remains necessary.
Reset/brownout or a failed configuration requires deliberate recovery and
reconfiguration, not reuse of an old software authorization flag.

Configure and Activate check GSTAT reset/fault bits and the driver error bits
before and after their operations. Observed faults return `TMC2209_ERR_FAULT`.
Any GSTAT acknowledgement attempt disarms the old profile; acknowledgement
alone never re-arms it. A reset that preserves the same silicon version still
requires reconfiguration. These checks cannot detect an event occurring after
the final sample. If an on-write failed late, TOFF may already be nonzero:
keep ENN inactive until the complete activation operation returns success.

## Bare-Metal and RTOS Examples

Copy both the `.c` and `.h` of the selected example, not both OS examples
together. Check each status returned by the example's public functions.

The bare-metal example's `TMC2209_AppInit` performs bounded identity probing
without motor configuration or activation. `AppConfigureMotor`, `AppActivate`,
`AppDeactivate`, and `AppClearGSTAT` are separate checked operations. Failed
initialization cannot be mistaken for successful setup. `AppDeinit` supports
explicit ownership cleanup; it does not itself disable hardware.

The FreeRTOS example must initialize from one task after the scheduler starts.
Its worker diagnostics share an application-level unit mutex. Do not add
unserialized register-mirror accesses merely because the UART bus has a lock.
Explicit motor operations must use the same unit lock.

`TMC2209_RtosInit(uart, baud, address)` is checked and performs only transport
setup and identity probing. Call it once per boot from an unlocked running
scheduler. The example requires dynamic CMSIS mutex/thread allocation and
`TMC2209_OS_CREATE_MUTEX=1`; the library's externally owned/static-lock option
is a separate integration pattern, not a mode of this service.

Use the checked `RtosConfigure`, `RtosActivate`, `RtosDeactivate`, and
`RtosClearGSTAT` methods for deliberate motor actions. `RtosReadDiagnostics`
returns IFCNT, GSTAT, and DRV_STATUS without modifying the caller's result on
error. `RtosIsReady` describes communication-service readiness, not motor
safety. `RtosGetStatus` and `RtosGetCleanupStatus` expose the first operation
and cleanup failures separately. These names have the `TMC2209_` prefix.

Workers remain parked after a latched failure, even if `Error_Handler` returns.
There is no automatic runtime restart or motor-action retry. Initialization
failure cannot start normal workers, and partial task creation is handled.

The application's asynchronous `Error_Handler` must assert its hardware inhibit
and stop its STEP source. Disabling interrupts alone does not stop an already
running hardware PWM/STEP timer. No example proves actual scheduler timing,
stack headroom, or mechanical stop behavior.

With `TMC2209_OS_CREATE_MUTEX=0`, call `TMC2209_BusInitWithLock` with a live,
application-owned **normal native FreeRTOS mutex**, not a binary semaphore or
an arbitrary CMSIS wrapper handle. The driver never deletes it. Static-only
RTOS builds must provide their mutex this way. Bare metal is single-context
and supplies a null external handle.

## Errors, Raw Access, and Mirrors

Fallible operations return `tmc2209_status_t`. Read outputs remain unchanged
on error; `UINT32_MAX` is valid data, not a success/failure test.

- `WriteReg` and typed write helpers confirm transmission only.
- `WriteRegVerified` holds the bus lock across IFCNT-before/write/IFCNT-after,
  checks the increment modulo 256, and reads back readable registers.
- Typed mirrors represent desired configuration, not proof of chip acceptance.
  `UINT32_MAX` retries a valid desired mirror; it is an error before a first
  request. Invalid values do not replace mirrors.
- Register constants are **bare addresses**. Do not pre-OR the write bit.
- Raw access rejects reserved/prohibited operations. Factory programming,
  test mode, and nonzero UART velocity are not qualified interfaces.
- Harmless raw diagnostic reads remain available, including OTP_READ,
  FACTORY_CONF, TSTEP, MSCNT, MSCURACT, PWM_SCALE, and PWM_AUTO. Restoring these
  reads does not authorize factory-trim writes or OTP programming.
- GSTAT reads do not acknowledge anything. `ClearGSTAT` accepts reset/DRV_ERR
  acknowledgement bits 0/1; UV_CP bit 2 is live, not an acknowledgement target.
- Bus serialization does not make shared unit mirrors or lifecycle operations
  thread-safe. Serialize every operation on a shared unit.

## Validation and Migration

The package-local `run_tests.bat` uses CMake/CTest and accepts an optional build
directory. From the Drivers workspace root, `.\run_tests.ps1 -Strict` and its
Release configuration enforce host tests, instrumentation, and coverage.
Static analysis and mutations have separate reproducible runners. Actual
F446RE compile/link checks use `validation\build_arm.ps1` and pinned real
dependencies; host stubs never count as target qualification.

Breaking changes include removed `SetupDefault` and motor `*_DEFAULT_*`
presets, bare register addresses, zero-initialized object ownership, checked
frame builders, the expected-register argument to reply decoding, unchanged
read outputs on failure, the `uv_cp` field name, and checked OS delay/ISR
semaphore operations. Update external callers as well as the bundled examples.

Passing software gates does not certify UART signal quality, current accuracy,
motor temperature, stall detection, mechanical bounds, or any other STM32
family. Those require separate hardware evidence.
