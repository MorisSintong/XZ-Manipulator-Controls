# Integration and operational contract

## Initialization and lifetime

Zero-initialize device/port objects, bind them once before publishing them to
tasks, and keep them and the HAL handle/mutex alive for their entire use.
The synchronization descriptor passed to the HAL initializer is copied;
the descriptor itself may be local, but its **context** must persist.

Binding does not perform an I2C probe, delay, reset or configuration write.
The application must wait at least the datasheet's **10 ms power-up time**,
then check an actual read. The AS5600 has no device-ID register that this
driver could use to authenticate the device at 0x36.

The normal lifecycle is:

1. Initialize clocks, GPIO, the live HAL tick and I2C1.
2. Wait for sensor power-up. Initialize the RTOS kernel and static mutex if used.
3. Bind the port and driver, checking every result.
4. Start the scheduler and use the RTOS-bound driver only from permitted tasks.

Do not rebind while any caller might use a handle. Do not delete/recreate a
referenced mutex or HAL handle. There is no teardown API because the driver
does not own these resources.

## Bus ownership and task safety

The lock represents the **physical I2C controller**, not merely this
sensor. Every peripheral driver using I2C1 must cooperate with the same
non-recursive, priority-inheriting mutex. HAL's internal `__HAL_LOCK` is not
an RTOS mutex.

One operation keeps ownership through all reads, read-modify-write steps,
settling and verification. This prevents interleaving by cooperating tasks.
It does not stop an external I2C controller or make a multi-register change
atomic inside the sensor.

An application using another driver's IT/DMA transfers must retain bus
ownership until that transfer has fully finished, not just until its start
function returns. This AS5600 port itself is **blocking/polling only**.

Do not:

- Call the driver with this mutex already owned by the same task.
- Suspend, delete or cancel a task that owns the bus.
- Acquire application locks in conflicting orders around sensor operations.
- Hold a critical section or scheduler lock around a driver call.
- Share an output object or modify an input configuration while a call uses it.

The STM32 port rejects exception mode, PRIMASK/BASEPRI/FAULTMASK masking,
unprivileged thread mode, and a non-1-ms HAL tick setting before any transfer.
The CMSIS adapter rejects kernel states other than `osKernelRunning`,
missing current tasks, recursive acquisition, and wrong-owner release.
Unprivileged MPU tasks need a separate privileged service; they cannot call
this direct-peripheral port.

ISRs should notify a task rather than calling these APIs. The portable core
delegates context validation to its port. If replacing the STM32 port,
provide the equivalent context checks: the CMSIS synchronization adapter
alone is not a replacement for Cortex-M execution-context validation.

## HAL and kernel timebases

The default STM32 HAL tick must advance once per millisecond, including
while a task is preempted or blocked. Configure a dedicated timer such as
TIM6 for the HAL tick under FreeRTOS, and let the kernel use SysTick.
Check the generated IRQ handlers and timer callback: increment the HAL
tick only for the intended timer, not for every timer interrupt.

Do not suspend the HAL tick during transactions. Tickless idle, STOP mode,
clock changes, debugger freeze settings, and custom HAL timebase overrides
need explicit review. Preserve a monotonic elapsed-millisecond clock across
sleep, or disallow such sleep while transactions are possible. Entry guards
cannot prove that a timer interrupt is correctly wired or still enabled.

The core uses unsigned elapsed-time subtraction to tolerate one 32-bit
clock rollover. All durations must be at most `INT32_MAX` ms; use small
application budgets such as 20-50 ms in practice. A callback must not stall
for a complete timer-counter cycle.

### What the timeout does and does not guarantee

The budget starts before bus acquisition. Each callback receives the
remaining budget; the core checks again after successful callbacks and
after release. It never resets the budget for each register or reports a
late successful transfer as successful. Original transport errors are
preserved even when their callback also consumed the remaining time.

**This is not a hard wall-clock execution bound.** The pinned STM32F4
`HAL_I2C_Mem_Read/Write` first use an independent **25 ms BUSY-flag wait**.
Their polling checks also have tick granularity. An I2C operation with a
1 ms remaining budget can therefore return much later. Kernel mutex/delay
rounding and task scheduling add further latency. A stopped HAL tick can
make polling fail to terminate; use an independent system watchdog for
system-level fault containment.

The driver does not patch vendor HAL or forcibly abort an in-progress
polling call. Applications requiring a proven sub-millisecond deadline or
a hard WCET must use a separately designed asynchronous/low-level port and
perform a system-level timing analysis.

HAL errors are translated while ownership is held:

| HAL outcome | Driver result |
| --- | --- |
| `HAL_OK` | Success, subject to core deadline and cleanup checks |
| `HAL_BUSY` | `AS5600_ERROR_BUSY`; stale error bits are not reinterpreted |
| `HAL_TIMEOUT` | `AS5600_ERROR_TIMEOUT` |
| `HAL_ERROR` with timeout bit | `AS5600_ERROR_TIMEOUT` |
| `HAL_ERROR` with only acknowledge-failure bit | `AS5600_ERROR_NACK` |
| Mixed/other HAL errors | `AS5600_ERROR_IO` |

The vendor does not identify every physical bus fault precisely; some
timeout-related paths become a generic I/O error. No error-code mapping is
a substitute for checking bus waveforms.

### CMSIS tick conversion

The mutex timeout is rounded up from milliseconds using a 64-bit
intermediate. Positive waits cannot become zero, overflow, or become
`osWaitForever`. An unrepresentable wait returns `AS5600_ERROR_ARGUMENT`
without acquiring a mutex.

Configure the adapter with a backend-compatible `max_wait_ticks`:
`0x7FFFFFFF` for a 32-bit tick, or `0xFFFE` for a compatible 16-bit backend.
The example checks the actual `TickType_t` width. The validated
CMSIS-FreeRTOS configuration uses **32-bit ticks**; the smaller-cap
arithmetic tests are not a claim that every CMSIS wrapper supports a
16-bit kernel. Use the documented 32-bit configuration for this project.

Configuration settling sleeps for rounded-up ticks **plus one tick** to
guarantee the minimum delay regardless of tick phase. It refuses a sleep
that cannot fit conservatively in the remaining budget. At slow kernel tick
rates, write operations therefore require larger budgets. No busy-wait is
used for this settling delay in the RTOS adapter.

Create CMSIS objects after `osKernelInitialize()`. Supply a real static
`StaticSemaphore_t` control block, `osMutexPrioInherit`, and no recursive or
robust attribute. The pinned CMSIS-FreeRTOS wrapper does not support robust
mutexes. Static allocation must be enabled in the kernel.

## Read and measurement semantics

All output parameters are committed only after the complete operation and
unlock succeed. On failure they are **unchanged, not zeroed**. This protects
callers from partial data but does not mean the old value is fresh or safe
to reuse.

`as5600_read_sample()` checks the magnet status before and after reading raw
angle, scaled angle, AGC and magnitude. Simultaneous weak/strong flags are
treated as invalid data; missing MD is no-magnet; weak/strong flags with MD
set return their respective errors. Reserved STATUS bits are ignored.

These reads are sequential. The bus mutex does not freeze the rotor or
latch all registers together, and the device supplies no sample sequence
number or timestamp in this API. A transient fault between status reads
can be missed. Transport success cannot detect every sensor failure or
stale physical measurement. The AS5600 is not thereby converted into a
redundant or safety-rated encoder.

The raw and scaled read APIs intentionally do not validate magnet presence.
Use the checked sample for ordinary application acquisition; use diagnostics
to investigate a magnet error. In 3.3 V operation the datasheet's AGC range
is 0-128; in 5 V operation it is 0-255.

The ANGLE register includes scaling and endpoint hysteresis; raw angle is
the appropriate input for the full-turn millidegree conversion helper.
Low-power modes, slow/fast filtering and the sensor watchdog affect
latency/noise. Do not enable them implicitly in a control-loop application.

## Configuration and failure recovery

CONF is validated field-by-field. Its OUTS value 3 is rejected on both
write and decode. Factory bits in CONF's upper two bits and the position
registers' upper nibbles are preserved by read-modify-write.

Changed words are written, allowed at least 1 ms of propagation through
the delay callback, and read back in full. An unchanged word is not
rewritten. This verifies register contents, not analog output accuracy or
complete filter settling; physical settling still depends on the selected
filter/power settings.

ZPOS/MPOS use modular increasing-angle order as defined by the sensor/DIR
configuration. Equal endpoints denote a full-turn interval; otherwise the
encoded span must be at least 205 counts, strictly greater than 18 degrees.
MANG accepts 0 for the default full range or 205..4095.
The position-pair API changes only ZPOS and MPOS; the MANG API changes only
MANG. Select and validate the intended datasheet calibration method rather
than assuming these independent registers are reset for you.

ZPOS/MPOS are **not a hardware-atomic pair**. A failed write may have
changed some or all of a register, and a failed later step may leave an
earlier write committed. There is no automatic retry, rollback or OTP
write. Suspend use of calibrated ANGLE/OUT after a write failure, inspect
settings, explicitly reapply the intended profile, and verify actual
behavior before resuming.

`AS5600_ERROR_UNLOCK` takes precedence over any earlier failure: bus
ownership can no longer be trusted. Escalate this as a synchronization
fault. Do not blindly continue or give a mutex on behalf of another task.

For an ordinary NACK/I/O error, every acquired driver lock is released
when the OS permits it. Higher-level recovery may retry a **new** operation
after reporting the fault. For stuck lines or a wedged controller, stop
other bus users and perform a board-specific recovery under shared
ownership. The driver never pulses GPIOs, resets I2C, or changes pin modes
behind another bus client's back.

## Replacing the transport

A custom `as5600_bus_t` must provide:

- Synchronous 1-/2-byte register reads/writes with a 7-bit address; buffers
  may not be retained after return.
- A monotonic modulo-32-bit millisecond clock and nonblocking context check.
- A minimum-delay callback respecting its finite budget.
- Either both lock callbacks or neither. Missing locking is only valid
  with one caller, no interrupt reentry and no concurrent bus clients.

Successful lock return means ownership was acquired exactly once; failed
lock return means nothing was acquired. Unlock must report failure
explicitly. A transport callback must return failure for a short/partial
transfer rather than success with incomplete data.

The core has no dynamic allocation, hidden retries, recursion, cache,
global mutable state, or application notification side effects. Result
handling and the system's safe response are explicit caller obligations.
