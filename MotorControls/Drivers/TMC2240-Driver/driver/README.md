# TMC2240 checked C driver

Canonical reusable implementation for the TMC2240, referenced to Analog Devices
**Rev. 2 (11/23)**. The STM32 adapter's software integration target is STM32F446RE.
Host simulation does **not** establish electrical, motor, timing, thermal, or
mechanical readiness. No hardware qualification or formal MISRA claim is made.

## Build and headers

Add `driver\inc` to your include paths. Compile these files **once**:

- `src\TMC2240.c` — portable checked register/field access, SPI/UART framing, cache.
- `src\tmc2240_hal.c` — STM32F4 HAL SPI transport and checked motor helpers.
- `src\CRC.c` and `src\Functions.c` — optional shared CRC/math utilities.

`inc\tmc2240.h` is the STM32 umbrella; **`inc\tmc2240_core.h`** is the portable
core header. They intentionally have different names, not merely different case.
`TMC2240_HW_Abstraction.h` contains register/field definitions and portable
callback declarations. The old duplicate `src\helpers` implementations are gone;
`inc\helpers` is only a legacy include-path forwarding layer.

C11 is required. Public declarations and field descriptors also compile as C++17.
Real STM32 HAL/CMSIS and `stm32f4xx_hal_conf.h` are application dependencies, not
provided by the portable core. `Tests\fake_hal` must **never** be used in firmware.

## Error and ownership contract

Every bus, register, field, and HAL helper operation returns `TMC2240Status`.
Compare with **`TMC2240_OK` (zero)**. Failed reads do not modify output parameters;
both `0` and `0xFFFFFFFF` are legitimate register data. Never infer a failure from
a data word, signed encoder position, ADC value, or boolean-like field.

```c
uint32_t status_register;
TMC2240Status result = tmc2240_driver_status(0U, &status_register);
if (result != TMC2240_OK) {
    /* Stop normal work and use the application's hardware-disable path. */
}
```

- `writeRegister` success means **transport completion**, not device acceptance.
  `writeRegisterVerified` additionally performs a two-frame hardware readback.
  SPI's write echo is not a readback. W1C registers are not eligible for this
  generic verifier.
- A failed/timeout write can already have reached the IC. Multi-register setup is
  not an atomic hardware transaction. Partial-configuration, transport, and
  reset/fault errors revoke software authorization. Failures after an activation
  write attempt trigger a best-effort TOFF=0. Validation-only errors do not write
  TOFF and can leave a prior valid configuration intact.
  Cleanup never replaces the original error with a later success.
- `hal_spiStatus` is diagnostic **last-frame** state, not a helper's result.
  `hal_getSPIStatus` returns the last valid device status byte separately.
- **Caller serialization is mandatory** over whole reads, RMW operations, and
  multi-register helpers, including every other user of a shared SPI bus.
  There are no internal RTOS locks and no thread-safe or ISR-safe claim.
- Configuration arrays/handles/pointers must identify live, sufficiently sized
  objects. Nulls, representable invalid IDs/counts, bad peripheral instances, and
  invalid pins/lengths/timeouts are rejected. C cannot validate dangling pointers
  or infer the allocation size behind a non-null pointer.

## Explicit startup; initialization does not power a motor

1. Assert an application-owned external **ENN hardware disable** before setup.
   `hal_init` cannot disable a driver left active by another host/reset domain.
2. Initialize the MCU SPI and CS GPIO. SPI must be master, mode 3, 8-bit,
   MSB-first, software NSS, TI/CRC disabled, and at most 10 MHz.
3. Call and check `tmc2240_hal_init(configs, count)`. The structure remains
   `{hspi, cs_port, cs_pin, spi_timeout_ms}`. Each CS is one GPIO bit; entries must
   not share CS, and one SPI peripheral must have one shared HAL handle.
   Timeouts must be finite, in `1..INT32_MAX` milliseconds.
   Initialization only validates/copies configuration, clears software cache
   state, and sets CS inactive. **It writes no IC registers and clears no faults.**
4. Check `tmc2240_hal_testConnection(id) == TMC2240_OK`. VERSION must be `0x40`;
   an all-ones disconnected read must not qualify as a connected IC.
5. Read `tmc2240_check_faults(id, &gstat_flags)`. Deliberately acknowledge
   application-approved flags with `tmc2240_clear_faults(id, mask)` only after
   handling their cause. The helper checks that requested flags cleared.
6. Fill an application-owned `TMC2240_MotorConfig_t` with all six explicit
   register values: `gconf`, `drv_conf`, `global_scaler`, `ihold_irun`,
   `chopconf`, `pwmconf`. **TOFF must be zero.** There are no universally safe
   motor/current defaults.
7. Check `tmc2240_hal_configureMotor(id, &config)`. It first disables TOFF, then
   writes and verifies the six chosen settings, leaving the chopper disabled.
8. Check `tmc2240_hal_activateMotor(id, toff)`. It requires successful explicit
   configuration, fresh matching readback, and clear GSTAT. A reset or driver
   error observed at any SPI phase revokes authorization, including the final
   activation/readback phases. Changing prepared settings or observing reset
   requires reconfiguration.
9. Only after successful startup may the application deliberately release ENN
   and generate bounded motion. On shutdown, check `tmc2240_hal_disableMotor`.
   `hal_deinit` releases software ownership/CS; it does not power off the motor.

The core raw write API intentionally remains available for advanced operation
and **can bypass** the HAL activation policy. Do not call raw writes "checked
motor startup." External enable/disable, mechanical bounds, and fault response
remain application responsibilities.

The adapter inserts a conservative CS guard of at least one SCK period and 1 µs
(covering four IC clocks for supported IC clock frequencies >=8 MHz).
`SystemCoreClock` and RCC clock values must be correct and stable. Use a context
where the HAL tick advances; a stopped HAL timebase cannot enforce a finite HAL
timeout. Host GPIO traces do not replace timing measurements on the board.

## Current and feature helpers

`tmc2240_calculateCurrent(run_mA, rref_ohm, range, gs, hold_percent,
hold_delay, run_delay, &ihold_irun)` is a pure checked calculation:

- RREF: 12–60 kΩ. Range 0/1/2/3 uses KIFS 11.75/24/36/36 A·kΩ.
- GS=0 means full scale (256). Values 1–31 are forbidden; 32–255 are fractional.
  Use zero, not a raw register value of 256, with checked register writes.
- Formula: RMS current = `(GS/256) * ((CS+1)/32) * (KIFS/RREF) / sqrt(2)`.
  The integer calculation uses 64-bit intermediates and `32*sqrt(2)*1e6`
  rounded down. Quantization is downward, **not saturation**.
- Zero run current, zero hold percent, unrepresentable small/large currents, or a
  hold current below one CS unit return RANGE. **CS=0 is 1/32 current, not off.**
  Use explicit disable for zero current. Hold units are
  `floor((IRUN+1)*hold_percent/100)`, then encode units minus one.
- Both delays are `0..15`. The legacy-shaped `hal_setCurrent` helper reads range,
  GS, and existing IRUNDELAY, preserves IRUNDELAY, and verifies the new current.

Other helper contracts:

- Microsteps: `1` and legacy `0` both mean fullstep; otherwise powers of two
  through 256. Invalid values are errors, not silent defaults.
- StealthChop toggling preserves PWMCONF. Enabling with an active chopper
  requires standstill and equal IHOLD/IRUN. Tuning is **not** performed for you.
  TPOWERDOWN must be at least 2 for automatic offset tuning (Rev.2 p.88).
- StallGuard4 threshold updates preserve filter/phase-offset bits. SG4_RESULT is
  the StealthChop result; bits 9 and 0 are always zero in valid hardware data.
  Set TCOOLTHRS and DIAG routing explicitly. Threshold zero is not advertised as
  a universal "disable"; use the documented velocity/routing controls.
- CoolStep SEMIN/SEMAX are load thresholds, not current fractions. Configuration
  preserves SEIMIN and SFILT; SGT is signed `-64..63`. For StealthChop, respect
  the documented IRUN minimums for SEIMIN (16/28), operating speed, and tuning.
- TSTEP measures time per **1/256 microstep**, independent of MRES. A larger
  TPWMTHRS means a **lower** transition velocity. Thresholds are 20-bit and
  overflow is an error.
- Chopper shape is updated in one RMW, never by enabling TOFF first and changing
  hysteresis later. TOFF=1 requires TBL>=2. Full-scale CS=31 with SpreadCycle
  requires effective HEND+HSTRT<=16; checked configuration/activation enforce it.
  TBL=0 still requires an external <=8 MHz IC clock, and TBL=1 an IC clock <=13 MHz;
  the application must know its actual IC clock and motor tuning.
- Encoder initialization uses signed binary 16.16 scaling, explicitly zeros XENC,
  and arms one N-event latch+clear. N edge selection is not A/B counting setup;
  CLR_ENC_X alone does not clear the counter. Decimal mode remains accessible via
  raw registers; its fractional units are 1/10000, not binary 1/65536.
- Fault observation never writes W1C. Fault output uses GSTAT bits 0..4 directly.
  Encoder N-event acknowledgement is likewise a deliberate operation.
- ADC helpers reject IOIN.ADC_ERR, mask the 13-bit sample (excluding AIN/reserved
  upper bits), and truncate integer scaling towards zero. Supply is
  `sample*9732/1000` mV; temperature is `(sample-2038)*100/77` tenths of °C.
  PWM automatic scale and both phase currents are signed nine-bit quantities.

## Register cache and portable callbacks

All TMC2240 implemented registers, including MSLUT, are **readable**. Reads always
use the transport, even with caching enabled. Reserved/nonexistent addresses,
read-only writes, reserved write bits, and unsupported field values are rejected.
Only documented even SENDDELAY encodings are supported; this is a library
restriction, not a claim that undocumented odd encodings have specific hardware
behavior.

The cache keeps **desired** and **confirmed** values separately:

- A write attempt records desired data and invalidates confirmation, even if
  transport fails; only a subsequent successful register read confirms it.
- Dirty means a desired value is not confirmed equal to hardware.
- Failed reads invalidate confirmation. Reset observation/invalidation retains
  desired data but invalidates all confirmations. Initialization clears both.
- W1C writes are commands, not desired register images.
- Factory reset metadata is never substituted for observed hardware data.

`tmc2240_getCachedRegister` exposes the state; old mutable shadow/dirty arrays and
sentinel-based cache operations are replaced. Build switches:

| Define | Supported values |
|---|---|
| `TMC2240_CACHE` | 0 or 1 (default 1); off does not break MSLUT RMW |
| `TMC2240_ENABLE_TMC_CACHE` | 0 = application hook, 1 = built-in (default) |
| `TMC2240_IC_CACHE_COUNT` | Built-in cache only: 1..255; default 4, must cover HAL IC count |
| `TMC2240_HAL_MAX_ICS` | 1..255; default 4 |
| `TMC_API_EXTERNAL_CRC_TABLE` | 0 or 1; default 0, 1 requires the external 256-byte table |

See `tmc2240_core.h` for the custom cache operation contract. Custom hook failures
propagate; invalidation is attempted after an observation failure. A failing
custom invalidation cannot be made reliable by the driver: treat cache state as
unknown and recover the hook before using its results. No cached read is used as
a substitute for hardware.

SPI/UART callbacks and bus/node lookup callbacks now return status. A custom SPI
callback must implement CS and timing for an exact five-byte frame. A UART
callback must perform the complete half-duplex request/reply, including line
turnaround and timeout. Portable UART frames/CRC are tested; the supplied STM32
UART callback returns **UNSUPPORTED**. NODEADDR changes require updating the
application's effective node address before reading back; do not expect an
unchanged callback address to follow an address-changing write automatically.

## Host validation

Package-local `Tests\CMakeLists.txt` works standalone or via the root
`add_subdirectory`. It builds actual production sources against independent
register/wire and HAL GPIO/SPI models, with C11 and strict warnings. Build out of
source. `TMC_ENABLE_SANITIZERS` and `TMC_ENABLE_COVERAGE` are optional global
options; the package does not replace parent compiler flags.
Standalone tests enable `TMC_EXTRA_WARNINGS` by default, matching the root warning
set. Package sanitizer self-probes are standalone-only; root builds use the
parent-owned instrumentation probes instead.
Standalone tests assign unique per-executable LLVM profile filenames. When added
by the root project, tests inherit `LLVM_PROFILE_FILE` unchanged from the runner.

Tests cover built-in cache capacities 1/4, cache off, custom cache, internal/
external UART CRC combinations, every shipped field descriptor, all 256 address
bytes, phase-by-phase transfer failures and acceptance uncertainty, checked
startup, and math/CRC utilities. Deterministic property and recovery loops print
their actual counts and initial seed. Sanitizer builds also run isolated,
deliberately defective fixtures to prove instrumentation detects errors.

These tests do not exercise a real STM32 scheduler, electrical bus, motor, or
stall/homing load. Firmware and copied CubeIDE sources must be synchronized and
separately rebuilt by their integration owner before deployment.
