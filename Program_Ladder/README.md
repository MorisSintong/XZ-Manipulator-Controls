# PLC Conveyor Ladder Program (`Program_Ladder`)

This directory contains the ladder logic program for the conveyor feeding and sorting workstation, developed in **OMRON CX-Programmer**.

---

## 1. Subsystem Overview

The PLC operates as the industrial material handling layer, responsible for:
- Conveyor belt start/stop and variable speed control.
- Optical proximity sensing for part presence and arrival gating.
- Interlocking with the STM32F446RE motion controller (handling pickup synchronization, busy signals, and cycle completion).
- Emergency stop (E-STOP) hardware safety trip and fault monitoring.

---

## 2. Project Files

| File | Type | Description |
| :--- | :--- | :--- |
| `TUGAS_AKHIR_CX.cxp` | CX-Programmer Project | Primary ladder logic project file |
| `TUGAS_AKHIR_CX.opt` | Option File | Workspace options, memory layout, and window states |
| `TUGAS_AKHIR_CX.bak` | Backup File | Previous version auto-backup |

---

## 3. Supported Hardware & Software

- **Programming Environment:** OMRON CX-One / CX-Programmer (v9.x or higher)
- **Target PLC Families:** OMRON CP1E / CP1L / CP2E series
- **Communication Interface:** USB (Toolbus) / Host Link (RS-232C) / Ethernet

---

## 4. Conveyor Operational Logic

```text
       +------------------+
       |   System Start   |  ➔ Conveyor motor runs forward
       +------------------+
                 |
                 v
   [ Optical Sensor 1 Triggered ]  ➔ Part arrives in Camera Inspection Zone
                 |
                 v
   [ Inspection Dwell / Tracking ] ➔ PC Vision captures & computes coordinates
                 |
                 v
   [ Optical Sensor 2 Triggered ]  ➔ Part reaches Manipulator Pickup Station
                 |
                 v
     [ Interlock Signal to STM32 ] ➔ Conveyor halts/indexes; STM32 begins pick cycle
                 |
                 v
     [ STM32 "Cycle Complete" ]   ➔ Conveyor resumes motion for next part
```

---

## 5. Typical I/O Memory Allocation

| Address | Type | Function | Device / Destination |
| :--- | :--- | :--- | :--- |
| **0.00** | Input | Start Push Button | Control Panel (NO) |
| **0.01** | Input | Stop Push Button | Control Panel (NC) |
| **0.02** | Input | E-Stop Button | Safety Interlock (NC) |
| **0.03** | Input | Photoelectric Sensor 1 | Inspection Trigger |
| **0.04** | Input | Photoelectric Sensor 2 | Manipulator Station Trigger |
| **0.05** | Input | Handshake Busy In | STM32 Manipulator In-Progress |
| **100.00**| Output | Conveyor Motor Contactor | Main Conveyor VFD / Relay |
| **100.01**| Output | Inspection Trigger Out | Camera / PC Trigger |
| **100.02**| Output | Sorting Gating Solenoid | Auxiliary Reject Pusher |
| **100.03**| Output | Ready / Status Indicator | Tower Lamp (Green) |
