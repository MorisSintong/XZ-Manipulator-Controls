# Complete STM32CubeIDE Project

This folder contains the STM32CubeIDE project for the **STM32F446RE Nucleo-64**
board and **MKS TMC2240** driver. Hardware qualification is pending; importing
or compiling the project is not evidence of electrical or motor readiness.

## How to Import & Build

1. Open **STM32CubeIDE**.
2. Go to **File** -> **Import...**
3. Select **General** -> **Existing Projects into Workspace** and click **Next >**.
4. In **Select root directory**, browse to this folder (`STM32CubeIDE_Project`).
5. Ensure `Test-TMC2240` is checked in the Projects list, and click **Finish**.
6. Right-click the project in Project Explorer and select **Build Project**.
   - Check both Debug and Release configurations. Do not rely on historical build claims.
7. Review the default diagnostics-only behavior and the motion opt-in requirements
   in `..\StallGuard4_Homing_Demo\README.md` before considering a hardware test.

The reusable `driver` tree is authoritative. From the Drivers workspace root,
`.\sync_tmc2240.ps1` checks the bundled copies; `.\sync_tmc2240.ps1 -Update`
explicitly synchronizes them after reviewed source changes. Do not independently
patch the copies under `Core\Inc\tmc`.

## Hardware Setup
Refer to `docs/Hardware_Guide.md` or the example folder README for complete wiring pinout tables.
