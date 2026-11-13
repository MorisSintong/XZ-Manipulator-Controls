# Documentation Index

This directory covers the checked TMC2240 API, legacy F446 examples, and the current physical-board wiring status.

- The current physical controller is BIGTREETECH SKR Octopus Pro V1.1 (STM32H723ZET6); see the [current system wiring](../../../../docs/hardware/wiring.md) and active firmware in [MotorControls/MotionFirmware](../../../MotionFirmware/README.md). The `driver/` library supports both STM32H7xx and STM32F4xx HALs, while the bundled examples and prebuilt images in this package remain legacy F446-only.
- The official TMC2240 datasheet PDF is not bundled in this checkout. Verify electrical limits against the current Analog Devices datasheet before hardware work.
- **[API_Reference.md](API_Reference.md)**: Checked API overview, units, ownership, and explicit startup. Headers and the canonical driver README contain the authoritative contracts.
- **[Hardware_Guide.md](Hardware_Guide.md)**: Legacy F446 example wiring; not the current SKR pinout.
- **[Audit_and_Fix_Log.md](Audit_and_Fix_Log.md)**: Historical remediation record, not current release or hardware qualification evidence.
- **[Dual_Motor_and_Encoder_Config.md](Dual_Motor_and_Encoder_Config.md)**: Legacy dual Nucleo-F446RE / AS5600 wiring reference.
