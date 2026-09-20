# Coding standards and compliance scope

## Claim and limits

This component is **MISRA C:2012-oriented C99**, not a MISRA compliance
certificate, a MISRA Compliance:2020 claim, or an ISO 26262 / IEC 61508
safety-qualified software element. It also applies relevant defensive
embedded/CERT C practices; that is not a claim of complete CERT C coverage.

A project making a formal compliance claim must select its licensed
guideline edition/amendments, define the enforcement and recategorization
plans, approve deviations, assess adopted HAL/RTOS code, run an appropriate
complete toolchain, and retain system-level evidence. Free analyzers and
unit coverage alone cannot satisfy those obligations.

The supplied source of record for device behavior is the local AS5600
v1-06 datasheet. No licensed MISRA rule text is reproduced here.

## Production scope

The assessed production files are:

- `include\as5600.h`, `src\as5600.c`
- `ports\stm32_hal\as5600_stm32_hal.h` and `.c`
- `ports\cmsis_rtos2\as5600_cmsis_rtos2.h` and `.c`

Examples, test fixtures, host C++ code, build scripts and upstream vendor
sources are not included in the production MISRA-rule claim. Examples are
compiled against real target headers, but an application's integration must
be assessed as part of that application. Header stubs are excluded from
analyzer findings only; findings in driver source are not blanket-suppressed.

## Enforced engineering practices

| Concern | Implementation |
| --- | --- |
| Language and ABI | C99, 8-bit bytes, fixed-width integer storage, explicit narrowing and unsigned bit operations |
| Memory | Caller-owned persistent contexts; fixed local buffers; no allocation, VLA, recursion or unbounded retry loop |
| Initialization | Explicit local initialization and bind-time callback validation; no implicit sensor reconfiguration |
| Side effects | Ordered calls, single exits, explicit result propagation, no hidden logging/transport work during binding |
| Input ranges | 12-bit positions, supported CONF enums, reserved output value rejection, finite positive timeouts |
| Arithmetic | Bounded conversion products and shift widths; intentional modulo timestamp subtraction; checked 64-bit tick conversion |
| Error handling | Distinct argument, lifecycle, context, lock, transport, verification and magnet errors |
| Resource handling | One acquisition per high-level operation; release after failures; wrong-owner and recursive-use rejection |
| Output validity | Commit on success only, including cleanup; no success-shaped default data |
| Hardware writes | Reserved-bit preservation, settling and readback; no OTP or unsolicited recovery operations |
| Concurrency | Immutable published handles, shared bus mutex, per-call temporaries, no shared last-error cache |

These address concerns represented by MISRA's essential-type, expression,
initialization, control-flow, function, pointer and library-use rules, and
CERT C concerns such as bounds, intentional integer behavior, resource
ownership and checked errors. This table is not a rule-by-rule compliance
matrix for a complete application.

The build treats strict diagnostics as errors, including conversion,
sign-conversion, shadowing, missing prototypes, undefined preprocessing
symbols and double promotion. C99 compilation is checked both on the host
and for the actual Cortex-M4 ABI.

## Scoped dispositions requiring project approval

`tools\cppcheck-suppressions.txt` contains only the following dispositions.
They are reviewable **proposals**, not automatic approval for a safety
project. Re-run `tools\analyze.ps1 -RawFindings` to see the unsuppressed
production findings.

| ID | Diagnostic / scope | Rationale and control |
| --- | --- | --- |
| D-01 | MISRA 11.5, `stm32_context()` and `cmsis_context()` only | A generic callback context is recovered as its original concrete object type. Binding installs the matching callback/context pair; contexts have correct alignment, static/caller-controlled lifetime, and remain immutable while published. No integer-to-pointer conversion or const removal is used. Each adapter centralizes the conversion in one function. |
| D-02 | MISRA 8.7, exported APIs in the three production `.c` files | This standalone-library analysis does not include the adopting application's call graph. Public functions must remain externally visible even when an optional API has no caller inside the library. All internal helpers have internal linkage. Reassess actual API use in the final linked project. |
| D-03 | MISRA 2.5, public constants in `as5600.h` | `AS5600_POWER_UP_MS` is part of the integration contract and is used by application startup, not by a library that deliberately does not own power-up. The suppression is scoped to this public header, not all macros. |
| T-01 | Cppcheck `constParameterCallback`, the two adapters | Callback parameters deliberately match the common `void *` transport ABI; other transports may mutate their context. The concrete adapter views are pointer-to-const, and no function-pointer cast is introduced to silence the warning. |

MISRA 11.5, 8.7 and 2.5 are advisory in the baseline edition; a project may
recategorize them more strictly. The generic context conversion remains a
real design deviation, not a claimed analyzer false positive. Changing a
callback signature, context storage or published lifetime requires review
of D-01 and T-01.

## Analyzer coverage and evidence

The supplied analyzer script selects Cppcheck's **ARM 32-bit, unsigned-char,
4-byte-wchar** data model, C99, exhaustive/inconclusive analysis and the
MISRA C:2012 add-on. It checks the three production translation units
together. Its HAL/CMSIS stubs model API boundaries for source analysis;
they do not prove vendor implementation behavior.

The separate real-header target build prevents a host-only stub interface
from being the sole integration evidence. Clang's path-sensitive analyzer,
address/undefined-behavior sanitizers, exhaustive register tests,
fault-injection tests and actual host-thread contention complement
Cppcheck; none of them proves complete absence of defects.

The MISRA add-on implements only a subset of the guidelines and has
analysis limitations. Requirements involving whole-program information,
implementation-defined behavior, documentation, timing, system lifetime,
hardware behavior and manual review remain application obligations.
The standard's licensed rule text can be supplied privately to a suitably
configured analyzer; it is intentionally not bundled.

Coverage thresholds concern executable driver lines, functions and ordinary
branch outcomes. They are **not MC/DC, formal verification or hardware
coverage**. Stack-usage reports are compiler-local frame estimates, not a
complete bound including HAL, kernel, interrupts, FPU context or application
callbacks.

See [Verification](verification.md) for reproducible commands and the
measured results, and [Integration](integration.md) for the conditions under
which the API guarantees apply.
