# Firmware Artifacts

**The `Test-TMC2240.bin`, `.hex`, and `.elf` files have been regenerated from the
audited, synchronized STM32F446RE Release source.** They are diagnostics-only:
`TMC2240_DEMO_ENABLE_MOTION=0`. Motor preparation/configuration/activation
symbols are absent from this garbage-collected default image.

These are **build-verified, hardware-unverified** artifacts, not unconditional
release qualification. `build-provenance.json` records the compiler, architecture
flags, tracked source hashes, and binary hashes. The compiler was GNU Arm GCC
14.3.1; the complete build matrix passed Debug and Release without executing any
firmware. Eclipse's managed builder was not invoked.

| File | SHA-256 |
|---|---|
| `Test-TMC2240.bin` | `342C9FF72FBD3F15B5B04A715B19C2F3CC2D0785328FBA6596DDD8A82B55D59E` |
| `Test-TMC2240.hex` | `9C29E38BFC6FDC3F2C241A5B5187EBF8F247F0585ABEDD659C109FA86057067E` |
| `Test-TMC2240.elf` | `95139234546EA382D3C913DC88E7E991DF0120D99CC06866810652B220DDEC13` |

No board has been flashed or operated during this software-only audit.
Compare the recorded hashes before considering a hardware test. A successful
build does not qualify the board, actual wiring, or motor. The previous
auto-motion binaries were replaced; their originals remain in the audit's
preserved baseline archive.

The revised source defaults to diagnostics-only operation. Motion requires
explicit compile-time opt-in, motor current and RREF settings, travel/time
limits, and a wired, application-controlled ENN GPIO. See
`..\examples\StallGuard4_Homing_Demo\README.md`.

Any subsequent source change requires a new build and provenance record.
The opt-in motion image is retained only as an audit build artifact; it is not
the firmware distributed here. Current official datasheet/errata currency could
not be confirmed because the vendor site timed out, and bench qualification
remains outstanding.
