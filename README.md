# XZ-Manipulator-Controls

Integrasi Computer Vision dan Sistem Sortasi Konveyor untuk Pengendalian Kualitas (Quality Control) Komponen Kapasitor Berdasarkan Polaritas.

## Subsystem Overview
- **`MotorControls/`**: STM32F446RE firmware, FreeRTOS, TMC2240 stepper driver controls for 2-axis (XZ) Cartesian Manipulator, AS5600 magnetic encoders, and vacuum gripper subsystem.
- **`VisionSystem/`**: Machine vision inspection pipeline, conveyor tracking, orientation calculation, and UART communication to the manipulator controller.
- **`VisionSystem/01_Dokumen_TA/`**: Official Thesis Proposal and specification documentation.

## Active Engineering Task & Codebase Audit
A comprehensive audit of the `VisionSystem/` module identified critical architectural, algorithmic, and communication discrepancies against the thesis specifications.

👉 **All developers and AI assistants MUST review and follow the strict remediation roadmap in:**  
**[`VisionSystem/REMEDIATION_PLAN_AI.md`](VisionSystem/REMEDIATION_PLAN_AI.md)**  
*(Also see [`AGENTS.md`](AGENTS.md), [`CLAUDE.md`](CLAUDE.md), and [`.cursorrules`](.cursorrules) for automated agent prompts)*