# TMC2240 Checked API Reference

The authoritative declarations are `driver\inc\tmc2240_core.h`,
`driver\inc\tmc2240_hal.h`, and `driver\inc\tmc2240_status.h`. Include
`tmc2240.h` for STM32 use, or `tmc2240_core.h` for the portable core.
The detailed integration and feature contracts are in `driver\README.md`.

**This is a breaking API hardening revision.** Compare operation results with
`TMC2240_OK`, which is **zero**, including `tmc2240_hal_testConnection`.
Do not preserve the old connection-test comparison with `1`.
Reads use output parameters and leave them unchanged on failure. Zero,
all-ones data, negative encoder positions, and negative temperatures are not
error indicators. Always inspect the returned status before using data.

## Scope and Ownership

The software target is STM32F446RE using SPI. Portable UART framing is tested,
but the supplied STM32 UART transport remains unsupported. No hardware,
electrical, thermal, mechanical, or real-time qualification is implied.

The application must serialize complete public operations, including both
frames of a pipelined read and complete read-modify-write/configuration
sequences. A HAL SPI lock around individual frames is insufficient.
Do not call these operations from interrupt context.

## Register and Field Operations

| Operation | Contract |
|---|---|
| `tmc2240_readRegister(id, address, &value)` | Hardware read into `uint32_t`; two SPI frames, no error sentinel |
| `tmc2240_writeRegister(id, address, value)` | Validated address/value, transport completion only |
| `tmc2240_writeRegisterVerified(id, address, value)` | Additional hardware readback; not a generic W1C acknowledgement |
| `tmc2240_updateRegister(id, address, mask, value)` | Checked read-modify-write; no write after a failed read |
| `tmc2240_fieldExtract(data, field, &value)` | Pure extraction into `int64_t`, including sign extension |
| `tmc2240_fieldUpdate(data, field, value, &result)` | Pure checked update; unspecified bits preserved |
| `tmc2240_fieldRead(id, field, &value)` | Checked hardware read plus extraction |
| `tmc2240_fieldWrite(id, field, value)` | Checked field write; W1C fields do not use a destructive RMW |
| `tmc2240_getRegisterInfo(address, &info)` | Access masks and known reset metadata |
| `tmc2240_validateRegisterValue(address, value)` | Checks supported register/value constraints |

Signed field values are mathematical signed values; unsigned fields accept
their full declared range. Invalid descriptors, reserved addresses, read-only
writes, and prohibited/reserved write values are errors, not silent no-ops.
Known reset metadata is not a universally safe motor configuration.

All implemented TMC2240 registers, including MSLUT, are readable. A read is
never substituted with a cached guess. `TMC2240CacheEntry` distinguishes desired
and confirmed values, validity, and dirty state; failed writes cannot establish
confirmation. See the core header for cache-disabled, custom-cache, and
external-CRC contracts.

## Explicit Motor Startup

1. Hold an application-controlled ENN inactive-high before communication.
   Transport initialization cannot disable outputs left active by another
   host or reset domain.
2. Configure the real SPI peripheral: master, mode 3, 8-bit, MSB first, software
   NSS, TI/CRC disabled, and at most 10 MHz. CS must be an application GPIO.
3. Check `tmc2240_hal_init(configs, count)`. It performs no motor-register writes
   or fault acknowledgement.
4. Check `tmc2240_hal_testConnection(id)`. Expected IOIN version is `0x40`.
5. Observe `tmc2240_check_faults(id, &flags)`. After addressing the causes,
   acknowledge only deliberately selected flags using
   `tmc2240_clear_faults(id, mask)`. Persistent flags remain errors.
6. Supply all six fields of `TMC2240_MotorConfig_t`: `gconf`, `drv_conf`,
   `global_scaler`, `ihold_irun`, `chopconf`, and `pwmconf`. TOFF must be zero.
7. Check `tmc2240_hal_configureMotor(id, &motor)`. It disables TOFF first,
   programs and verifies the selected profile, and leaves the chopper disabled.
8. On a separate, deliberate start action, check
   `tmc2240_hal_activateMotor(id, toff)` before releasing ENN or producing steps.
   Changed settings, readback mismatches, and reset/fault observations invalidate
   authorization.
9. On shutdown, use the hardware-disable path and check
   `tmc2240_hal_disableMotor(id)`. `hal_deinit()` does not power down the motor.

The raw core write API can bypass the high-level activation policy. A failed
write may already have reached the IC. Never rely on software cleanup alone as
a physical inhibit.

GSTAT's documented reset word is `0x1D`. The demo requires an explicit
`TMC2240_DEMO_ACK_STARTUP_FLAGS` policy rather than silently clearing startup
or runtime faults. Its compile-only fixture permits `0x1D`, excluding DRV_ERR;
the application must still verify that the underlying causes are resolved.

## Current, Chopper, and Load Helpers

`tmc2240_calculateCurrent(run_mA, rref_ohm, range, gs, hold_percent,
hold_delay, run_delay, &ihold_irun)` calculates a nominal RMS setting without
hardware access. RREF must match the actual module, and requested current must
respect the motor, board, and thermal limits.

GS zero represents full scale; 1 through 31 are forbidden. Current is quantized
downward, not silently saturated. Zero/unrepresentable current and invalid
parameters return a range error. Current code zero is not zero current; use
explicit motor disable for that purpose.

| Helpers | Important constraints |
|---|---|
| `hal_setCurrent`, `hal_setMicrosteps` | Check status; invalid values are not silently ignored. Fullstep is 1, with legacy 0 also accepted. |
| `stealthchop_enable` | Preserves PWMCONF; does not perform physical tuning or install hidden PWM defaults |
| `stallguard_set_threshold`, `stallguard_read` | SG4 belongs to StealthChop; threshold/velocity/routing and motor tuning matter |
| `coolstep_configure` | Load thresholds are not current fractions; SGT is signed |
| `set_tpwmthrs`, `set_tcoolthrs`, `set_thigh` | 20-bit thresholds; TSTEP uses internal 1/256-microstep periods, not a fixed external STEP frequency |
| `set_chopper`, `set_tpowerdown` | Checked field/range rules; nonzero TOFF uses the activation gate |
| `diag_configure` | Explicit routing and output configuration |

The names in this table have the `tmc2240_` prefix. Review the full declarations
and constraints in `tmc2240_hal.h` and `driver\README.md`. Mutating prepared
settings requires reconfiguration before activation.

## Diagnostics and Encoder Data

| Function | Output type and unit |
|---|---|
| `tmc2240_driver_status` | `uint32_t *`, raw DRV_STATUS |
| `tmc2240_check_faults` | `uint8_t *`, GSTAT bits 0 through 4; no acknowledgement |
| `tmc2240_read_vsupply` | `uint16_t *`, raw ADC sample |
| `tmc2240_get_vsupply_mV` | `uint32_t *`, millivolts |
| `tmc2240_read_temperature` | `uint16_t *`, raw ADC sample, **not degrees Celsius** |
| `tmc2240_get_temperature_c10` | `int16_t *`, tenths of a degree Celsius |
| `tmc2240_stallguard_read` | `uint16_t *`, SG4 result |
| `tmc2240_encoder_read`, `encoder_read_latch` | `int32_t *`, signed position |
| `tmc2240_encoder_get_status` | `uint8_t *`, N-event indication |
| `tmc2240_pwm_get_scale` | `uint16_t *` sum and `int16_t *` signed automatic scale |
| `tmc2240_get_microstep_counter` | `uint16_t *`, microstep counter |
| `tmc2240_get_microstep_current` | Two `int16_t *` signed phase-current values |

Encoder initialization accepts signed binary 16.16 scaling; N-event
acknowledgement is explicit. ADC helpers reject ADC_ERR and apply documented
integer scaling. The last SPI status accessors are diagnostics, not substitutes
for a complete operation's return status.

## Demo and Validation

The demo defaults to diagnostics only. Motion requires explicit motor values,
ENN wiring, startup acknowledgement, and bounded travel/time settings.
Watchdog exhaustion is a fault, not a fabricated endpoint. Commanded step
counts do not prove mechanical position, and an EMA does not make StallGuard
reliable for every load.

Busy-wait pulse rates are nominal and must be measured. No maximum stable
speed, hardware-tested homing result, or generic "DWT needs a debugger" claim is
made. The IC has no internal ramp/position controller.

Use the workspace host runner, static analysis, mutation checks, and
`validation\build_arm.ps1` for their distinct software gates. Command-line
Cube-source builds are not an invocation of Eclipse's managed builder.
Hardware qualification remains a separate requirement.
