# MotionFirmware — dual-axis motion diagnostics

STM32F446RE HAL firmware for two STEP/DIR TMC2240 motors and motor-shaft AS5600
encoders. The runtime is a cooperative foreground state machine with timer
compare ISRs, not an RTOS or an autonomous bounce/cycling demo.

The implementation/wire contract and bench acceptance procedure are in
[`motion_diagnostics_design.md`, especially §7](../../docs/architecture/motion_diagnostics_design.md).
Datasheet limitations are tracked in
[`diagnostics_datasheet_notes.md`](../../docs/hardware/diagnostics_datasheet_notes.md).

## Hardware and coordinate mapping

| Wire axis | Motor | CS / STEP / DIR / ENN | Timer | Encoder |
|---|---|---|---|---|
| 0 = X, positive away from home | 1 | PC3 / PC4 / PC5 / PC7 | TIM5 | I2C1 PB8/PB9 |
| 1 = Z, positive downward | 0 | PC0 / PC1 / PC2 / PC6 | TIM2 | I2C3 PA8/PC9 |

SPI2 is shared, mode 3, 2.625 MHz, on PB13/PB14/PB15. Encoder buses are
400 kHz, both at 7-bit address 0x36. Follow the physical harness specified in
[`wiring.md`](../../docs/hardware/wiring.md); verify direction, encoder and MSCNT
signs separately. There is no automatic detection or swapping of buses.

**USART2 is binary only:** 115200 8-N-1 on PA2/PA3, circular RX DMA1 Stream5,
whole-frame TX DMA1 Stream6. COM numbers are host-dependent. Human messages and
`printf` go only to nonblocking SEGGER RTT terminal 0. No ASCII tokenizer,
RTT input merging, `readline`, or UART text banner is supported.

## Runtime

1. Start with ENN high; validate/configure both drivers and the volatile AS5600
   profile. Acknowledge captured **startup** GSTAT explicitly; runtime driver
   reset/fault bits are never automatically cleared.
2. Home **Z upward first, then X**: standstill preparation, bounded coarse seek,
   four low SG4 confirmations after acceleration blanking, backoff, repeat seek
   at the same capped operating speed, clearance, endpoint settle, software zero.
   No opposite-end probing or stroke measurement is performed.
3. Emit HOME_RESULT; await binary commands. Picks are accepted only with both
   axes homed, valid references, confirmed usable limits and reserved result space.
4. Type `0x01` is an absolute X pick target: **X move → Z down → dwell → Z up**.
   Every phase, including zero-distance phases, has separate start/end evidence.
   Y, angle, correction and class are echoed, not used for Z/servo/vacuum control.
   Four picks can wait behind one active command; repeated object IDs are allowed.
5. Type `0x02` emits an immediate identity-bearing STATUS without motion.
   Unsolicited status is coalesced at 1 Hz. Type `0x03` aborts: inhibit new rising
   edges, finish an existing high pulse, report partial work and cancel queued
   work. Type `0x04` requests explicit idle/empty-queue re-home, after health checks.

Abort or continuity loss invalidates position; no automatic resume/retry.
An explicit re-home can reseed healthy encoders and clear the software abort
only after B1 is released; a held B1 also blocks boot homing.
Persistent driver GSTAT faults require operator review and board restart after
the underlying cause is corrected; re-home does not silently acknowledge them.
B1 is an immediate software abort, **not** a start/resume/off control.

### Bring-up defaults

`App\Src\motion_config.c` is the immutable run configuration (ID 1):

- 3200 rising STEP events/rev, MRES=4, **DEDGE=0, INTPOL=0**.
- X 40 mm/rev (80 steps/mm); Z 8 mm/rev (400 steps/mm).
- **500 steps/s start, 2000 steps/s cap**, 4000 steps/s² acceleration.
- STEP high/low minimum 5 µs; DIR setup/hold 20 µs; late-edge budget 20 µs.
  These are conservative application choices, **not verified electrical minima**.
- Safe Z = pick depth = **0 (dry run)**; no suction or servo actuation.
- Candidate usable maxima X 290 mm / Z 140 mm, but
  **`bounds_confirmed=false` disables all picks** until bench qualification.
  Qualify clearance, usable limits and positive pick depth before enabling them.
- Homing seek budgets X 23200 steps / 30 s; Z 56000 steps / 40 s; SG threshold 10,
  400 ms blanking, four distinct low observations. These are tuning inputs, not
  verified SG4 operating conditions; support Z and supervise first homing.
- AS5600 volatile CONF documented bits **0x0300**, preserving reserved bits.
  No range/OTP/BURN writes. RAW acquisition targets 2 ms; health targets 20 ms.
  Records retain the actual full CONF word read at boot/health/endpoints;
  documented-bit mismatch against 0x0300 invalidates encoder health until recovery.
  NACK, bad magnet, half-turn ambiguity, overflow or >10 ms gap invalidates
  continuity and stops motion. Recovery requires fresh reseeding and re-home.

Change the config ID/run metadata whenever settings or calibration change.
MSCNT is reported **unqualified (check=3)** by default; modulo agreement is not
proof of physical travel. Shaft-equivalent displacement is not carriage metrology.

## Build, tests and deterministic fixtures

Requirements: Arm GNU Toolchain, CMake ≥3.22, Ninja; LLVM-MinGW/Clang for host tests.
From this directory:

```powershell
cmake --preset Debug
cmake --build --preset Debug
.\Tests\run_tests.ps1
```

For long Windows worktree paths, map an **unused local** drive and always remove
it afterward (run from the repository root):

```powershell
subst X: (Get-Location).Path
try {
    cmake --preset Debug -S X:\MotorControls\MotionFirmware -B X:\MotorControls\MotionFirmware\build\C1Debug
    cmake --build X:\MotorControls\MotionFirmware\build\C1Debug
} finally { subst X: /d }
```

Host CTest uses `-Wall -Wextra -Werror -Wconversion -Wsign-conversion` and
ASan/UBSan when the compiler supports them. `run_tests.ps1` also compiles every
HAL-independent App source for Cortex-M4. Tests cover framing, full record
packing, rational rounding, timer edges/abort/starvation, encoder validity,
queue admission, phase sequencing and bounded Z-before-X homing.
All host executables share pre-main Windows Error Reporting/assert-abort dialog
suppression; failures appear as nonzero CTest exit codes, not desktop dialogs.
Every registered test has a 30-second timeout (fixture subprocess: 15 seconds).
Add reproductions to this registered suite; do not run scratch debug executables.

`Tests\fixtures\diag_records_v1.json` is generated by the host `fixture_writer`
using the **production C packer**. It contains all three record types, signed
extremes, zero/missing measurements, aborted and cancelled commands. Regenerate:

```powershell
cmake --build Tests\build-host --target generate_diag_fixtures
ctest --test-dir Tests\build-host --output-on-failure
```

Every fixture includes expected header fields plus `phases`, `home`, or `status`
objects. CTest checks byte-for-byte fixture stability. Frame lengths are
**356 / 124 / 116 bytes**; response CRC covers bytes 2…39+L, command CRC covers
bytes 0…13. No native C struct is transmitted.

## Source ownership and safety

`App\Inc` / `App\Src` contain HAL-independent config, packer, DMA-queue/parser
transport, profile/step engine, sampler, driver evidence, homing, executor and
dispatch. `Core\Src\motion_board.c` binds GPIO/timer/DMA and existing driver APIs;
`Core\Src\console.c` is RTT-only. CubeMX scaffold was selectively imported from
the reference branch because CubeMX was unavailable; `.ioc`, MSP, interrupts,
HAL configuration and TIM/I2C vendor sources were updated together. Compare
interrupt hooks and application calls remain in USER CODE sections. TIM2/TIM5
free-run at 1 MHz; CCR1 is a software deadline, not an AF output on the STEP pins.

Accepted commands reserve terminal records; DMA owns complete frames until
completion. Sustained TX failure inhibits admission/new picks without blocking
RX/abort. RX uses monotonic producer progress and reports full-lap overwrite.
After RX errors, complete buffered frames are drained; the partial parser is
discarded and the logical origin is aligned to DMA's restarted index zero.
Recovery first aborts RX, then snapshots its frozen counter and drains it.
Only RX line/RX DMA errors restart reception; TX DMA errors recover TX alone.
Counter publication is gated by a successfully started RX epoch; failed starts
are retried without reading non-programmed DMA counters or replaying stale bytes.
STATUS bit 12 is a one-record RX-error event, not a latched readiness fault;
counters stay cumulative. The v1 readiness fault mask is `0x0003E7E2`.
MSCNT sign derives from the driven DIR polarity with enforced SHAFT=0
(X positive, Z negative); it is not separately configurable.

Reset starts with drivers disabled; configured runtime abort/fault paths retain
holding torque, particularly Z. Electrical faults can still remove torque:
use an independent brake/support and physical power/interlock. Never hot-plug
motors under power. UART/B1 stops are not safety-rated emergency stops.

Before hardware release, perform the design **§7 bench procedure** with an
external calibrated linear reference, qualify SG4/current/signs, obtain official
STEP/DIR limits, and measure ISR jitter, sample gaps, UART backpressure and Z
anti-drop behavior. Compilation and simulated tests do not qualify the hardware.
