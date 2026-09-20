# TMC2240 Library Fix Plan

> **Historical record, not current release evidence.** This document records an
> earlier remediation pass. Its completion claims and code snippets are
> superseded by the current checked APIs, explicit motor-activation policy,
> and reproducible validation gates. It does not establish hardware readiness.

This document tracks the remediation of datasheet mismatches in the TMC2240 library for STM32F446RE.  
**Datasheet reference:** Trinamic TMC2240 Datasheet Rev. 2 (pages 22–23, 46–48, 50–57, 73–123).

**Historical status:** the earlier pass reported its phases complete and a
clean firmware build. Those results were not a qualification of the currently
distributed standalone package.

---

## ✅ Completed Fixes (All Phases 1–8)

### Phase 1: Remove Fake Internal Ramp API
The TMC2240 **does not have an internal ramp generator** (STEP/DIR only). The library contained a complete but invalid ramp/position API copied from a different driver (likely TMC5160).

**Files changed:**
- `Core/Inc/tmc/ic/TMC2240/TMC2240_HW_Abstraction.h`:
  - Removed ramp register defines: `RAMPMODE` (0x20), `XACTUAL` (0x21), `VACTUAL` (0x22), `VSTART` (0x23), `A1` (0x24), `V1` (0x25), `AMAX` (0x26), `VMAX` (0x27), `DMAX` (0x28), `D1` (0x29), `VSTOP` (0x2A), `TZEROWAIT` (0x2B).
  - Removed `SW_MODE` (0x34), `RAMP_STAT` (0x35), `XLATCH` (0x36), `ENC_DEVIATION` (0x3D) — nonexistent on TMC2240.
  - Removed `TMC2240_MAX_VELOCITY`, `TMC2240_MAX_ACCELERATION` (ramp-specific constants).
  - Removed Ramp Mode Constants (`MODE_POSITION`/`MODE_VELPOS`/`MODE_VELNEG`/`MODE_HOLD`).
  - Removed Switch Mode bits (`SW_STOPL_ENABLE`…`SW_SOFTSTOP`).
  - Removed Ramp Status bits (`RS_STOPL`…`RS_SG`).
  - Removed `EM_LATCH_XACT` (ENCMODE bit 9 — **reserved per datasheet p.92**).
  - Removed `DEVIATION_WARN` (ENC_STATUS bit 1 — does not exist).
  - Removed `ENC_DEVIATION` register (0x3D — does not exist).
  - Removed all ramp field definitions (`RAMPMODE`…`XTARGET`, `SW_MODE`, `RAMP_STAT`, `XLATCH`).
  - Removed `LATCH_X_ACT`, `DEVIATION_WARN`, `ENC_DEVIATION` field definitions.
  - **Added** real `DIRECT_MODE` register (0x2D) with proper fields:
    - `DIRECT_A` — bits 8:0, signed 9-bit
    - `DIRECT_B` — bits 24:16, signed 9-bit
  - Updated header comment: "no internal ramp generator (STEP/DIR driver only)".
  - Fixed MSCURACT phase swap (Phase 4).

- `Core/Inc/tmc/ic/TMC2240/TMC2240_motion.h` **DELETED** entirely.  
  The entire file was a fake ramp API. Valid encoder helpers duplicate the HAL API (fixed in Phase 3).

- `Core/Inc/tmc/ic/TMC2240/TMC2240.c`:
  - Cache access table updated:
    - `0x04` (IOIN): `0x03` → `0x01` (IOIN is **read-only** per datasheet p.22).
    - `0x20–0x2B`: all blank except `0x2D` (DIRECT_MODE) = `0x03` (R/W).
    - `0x34` (SW_MODE), `0x35` (RAMP_STAT), `0x36` (XLATCH), `0x3D` (ENC_DEVIATION): blank.
    - `0x39` (XENC): `0x01` → `0x03` (R/W per datasheet p.22).
    - `0x60–0x69` (MSLUT0–7, MSLUTSEL, MSLUTSTART): `0x42` → `0x03` (R/W per datasheet pp.98–108).

- `Core/Inc/tmc/platform/STM32/tmc2240_hal.c`:
  - Removed `tmc2240_switches_configure()` and `tmc2240_switches_read_status()` (target nonexistent registers).
  - Rewrote encoder API (Phase 3).

- `Core/Inc/tmc/platform/STM32/tmc2240_hal.h`:
  - Removed `tmc2240_switches_configure()`, `tmc2240_switches_read_status()`.
  - Updated encoder API signatures (Phase 3).

- `README.md`: Updated feature table, removed ramp/limit-switch claims.

---

### Phase 3: Fix Encoder APIs
**Files changed:**
- `Core/Inc/tmc/platform/STM32/tmc2240_hal.c`:
  - `tmc2240_encoder_init()`:
    - Removed `EM_LATCH_XACT` (reserved bit 9).
    - Kept the `CLR_ENC_X` pulse (bit 8, valid W1C) to zero the counter.
    - Default `ENCMODE=0` with `EM_POS_EDGE | EM_NEG_EDGE | EM_CLR_ONCE` (standard quadrature, count both edges, clear-once on index).
    - Added 16.16 fixed-point docs: `1.0 = 65536`.
  - `tmc2240_encoder_get_status()`:
    - Returns `n_event` (index pulse) only.
    - Removed `deviationWarn` (reserved bit 1 in ENC_STATUS).
  - Added `tmc2240_encoder_clear_n_event()` (W1C bit 0).
  - Removed `deviationWarn` parameter from `get_status`.

- `Core/Inc/tmc/platform/STM32/tmc2240_hal.h`:
  - Updated signatures: `encoder_init(uint32_t enc_constant_16_16)`, `get_status()` returns `uint8_t` (n_event only), added `clear_n_event()`.

- `README.md`: Updated encoder docs to specify 16.16 fixed-point constant.

---

### Phase 4: Fix MSCURACT Phase Swap
**Datasheet p.109:** `CUR_A` = bits 24:16, `CUR_B` = bits 8:0 (both 9-bit signed).

**Files changed:**
- `Core/Inc/tmc/ic/TMC2240/TMC2240_HW_Abstraction.h`:
  - `CUR_A_MASK` = `0x01FF0000`, shift 16.
  - `CUR_B_MASK` = `0x000001FF`, shift 0.

- `Core/Inc/tmc/platform/STM32/tmc2240_hal.c`:
  - `tmc2240_get_microstep_current()` decode now reads CUR_A from bits 24:16 and
    CUR_B from bits 8:0 (it references the corrected constants; stale
    bit-position comments also fixed).

---

### Phase 2: Fix Current Formula (RREF/KIFS Model)
**Datasheet pp. 46–48:** TMC2240 uses integrated current sensing with external `RREF` resistor and `KIFS` factor per `CURRENT_RANGE`. It does **not** use a motor sense resistor.

**Original bug:** `tmc2240_hal.c` used an `rSense_mOhm` parameter and hardcoded V_fs factors from a different driver model — the API parameter was misleading (no shunt resistor exists).

**Correct formula (Datasheet p.46):**
```
I_rms = (GLOBAL_SCALER/256) × ((CS+1)/32) × (KIFS / RREF) × (1/√2)
```
- `GLOBAL_SCALER`: 0 = 256 (full scale), 1–255 = value/256.
- `KIFS` by `CURRENT_RANGE` (DRV_CONF bits 1:0):
  - 0: 11.75 A·kΩ
  - 1: 24.0 A·kΩ
  - 2: 36.0 A·kΩ
- `RREF`: external precision resistor in ohms (typically 12 kΩ on MKS modules).

**Solve for CS (IRUN field):**
```
CS+1 = I_rms_A × 256/GS × 32 × √2 × RREF_Ω / (KIFS × 1000)
```

**Implemented in `tmc2240_hal.c:tmc2240_hal_setCurrent()`:**
1. Parameter changed: `rSense_mOhm` → `rRef_Ohm` (e.g., 12000 for 12 kΩ).
2. Reads `CURRENT_RANGE` from DRV_CONF (bits 1:0).
3. Reads `GLOBAL_SCALER` (0 → treated as 256).
4. Selects `KIFS1000` (KIFS × 1000): {11750, 24000, 36000}.
5. Computes using 64-bit intermediate to avoid overflow:
   ```c
   GS = (gs == 0) ? 256 : gs;
   uint64_t num = (uint64_t)runCurrent_mA * 256 * rRef_Ohm * 45248; // 32*√2*1000 ≈ 45248
   uint64_t den = (uint64_t)GS * KIFS1000 * 1000000;
   temp = num / den;
   irunValue = (temp > 0) ? (temp - 1) : 0;
   ```
6. Clamps `irunValue` to 0–31.
7. Updated `tmc2240_hal.h` signature and docs:
   ```c
   void tmc2240_hal_setCurrent(uint16_t icID,
                               uint16_t runCurrent_mA,
                               uint16_t rRef_Ohm,
                               uint8_t  holdPercent,
                               uint8_t  holdDelay);
   ```

**Files changed:**
- `Core/Inc/tmc/platform/STM32/tmc2240_hal.c`
- `Core/Inc/tmc/platform/STM32/tmc2240_hal.h`
- `README.md` (Current control description)
- `main.c` (added `tmc2240_hal_setCurrent(0, 1000, 12000, 50, 5)`)

---

### Phase 5: Cache Hardening

**Files changed: `Core/Inc/tmc/ic/TMC2240/TMC2240.c`, `Core/Inc/tmc/ic/TMC2240/TMC2240.h`**

1. **Bounds check in cache** (`TMC2240.c:tmc2240_cache()`):
   ```c
   if (address >= TMC2240_REGISTER_COUNT) return false;
   ```

2. **Public API bounds check** (`tmc2240_readRegister`, `tmc2240_writeRegister`):
   ```c
   if (address > 0x7F) return -1; // read
   if (address > 0x7F) return;    // write
   ```

3. **Signed-shift UB fix** (`readRegisterSPI`, `readRegisterUART`):
   ```c
   uint32_t raw = ((uint32_t)data[1]<<24) | ((uint32_t)data[2]<<16) |
                  ((uint32_t)data[3]<<8)  | (uint32_t)data[4];
   result = (int32_t)raw;
   ```

4. **CACHE=0 compile fix** (`TMC2240.h`):
   - `TMC2240CacheOp` enum and `TMC2240_CACHE_READ/WRITE/FILL_DEFAULT` constants moved outside the `#if TMC2240_CACHE` block so `-DTMC2240_CACHE=0` compiles (verified).

5. **Cleanup unused preset array** (`TMC2240.c`):
   - Removed `tmc2240_sampleRegisterPreset` array (was unused, triggered warning).
   - Removed `R00`…`R70` defines and `N_A` (used only in that array).
   - Kept `____` (used in access table), removed `____REG` (unused).
   - Kept `tmc2240_RegisterConstants` (MSLUT constants) for documentation; `initCache` is now a no-op since no write-only entries remain.

6. **Access table corrected to match datasheet** (`TMC2240.c`):
   - `0x04` (IOIN): `0x03` → `0x01` (read-only).
   - `0x20–0x2B`: all blank except `0x2D` (DIRECT_MODE) = `0x03` (R/W).
   - `0x35`, `0x36` (RAMP_STAT/XLATCH): `0x01` → blank (reserved).
   - `0x39` (XENC): `0x01` → `0x03` (R/W).
   - `0x60–0x69` (MSLUT0–7, MSLUTSEL, MSLUTSTART): `0x42` → `0x03` (R/W).

---

### Phase 6: SPI Error Reporting + SG4 Docs + DRV_CONF Comment

**Files changed: `Core/Inc/tmc/platform/STM32/tmc2240_hal.c`, `tmc2240_hal.h`**

1. **SPI error reporting:**
   - Added static `lastSpiStatus[4]` array.
   - `tmc2240_readWriteSPI()` stores `halStatus` per `icID`.
   - Added accessor: `HAL_StatusTypeDef tmc2240_hal_spiStatus(uint16_t icID);`
   - Documented: cache is updated even on failed writes (void callback limitation).

2. **SG4 threshold docs (was inverted):**
   - Datasheet p.53, 121: **higher** `SG4_THRS` = more sensitive (triggers easier).
   - Fixed `tmc2240_hal.h`: "Lower value = more sensitive" → "Higher value = more sensitive (lower = less sensitive). 0 disables comparator."
   - Fixed `main.c` comment: "lower = more sensitive" → "higher = more sensitive".

3. **DRV_CONF comment fixed:**
   - `tmc2240_hal.c`: value `0x20` encodes `SLOPE_CONTROL=2` (400 V/us), not 1.

---

### Phase 7: main.c UART Debug Fixes

**File changed: `Core/Src/main.c`**

1. **StallGuard threshold comment fixed** (was inverted):
   ```c
   // tmc2240_stallguard_set_threshold(0, 50);  // 0-255, higher = more sensitive
   ```

2. **Debug output moved after the stepping state machine** (was between pulses):
   - The `print_timer` block now runs after the STEP pulse logic (was lines 143–159, before the stall detection).
   - Added comment: "blocking UART + SPI reads may pause STEP pulses ~2 ms at 164 kHz SPI."
   - Added `tmc2240_get_vsupply_mV()` and `tmc2240_get_temperature_c10()` to the ~1 Hz line.

3. **Removed stale comment** about "Infinite loop unreachable".

---

### Phase 8: Docs & Config Sync

**Files changed: `README.md`, `Test-TMC2240.ioc`, `tmc2240_hal.c/h`**

1. **README.md:**
   - SPI frequency corrected to `164 kHz` (was 328 kHz). APB1 = 42 MHz / 256 = 164.06 kHz.
   - DRV_CONF: `SLOPE_CONTROL=2` (400 V/µs).
   - Current control: documented RREF/KIFS model, removed `rSense_mOhm`.
   - Encoder: specified 16.16 fixed-point (`1.0 = 65536`).
   - Temperature: `T = (ADC - 2038) / 7.7 °C` conversion helper noted.
   - Supply voltage: `V = ADC × 9.732 mV` conversion helper noted.
   - Removed ramp/limit-switch/encoder-deviation rows from feature table.
   - Removed deleted `TMC2240_motion.h` from Library Files table.
   - StallGuard threshold docs corrected (higher = more sensitive).
   - "Known Issue #1" reworded: TMC2240 has no ramp registers at all.
   - Lessons Learned #8 updated accordingly.

2. **Test-TMC2240.ioc:**
   - `SPI2.BaudRatePrescaler=SPI_BAUDRATEPRESCALER_256` (was /32, mismatching main.c)
   - `SPI2.CalculateBaudRate=0.1640625 MBits/s`

3. **ADC Conversion Helpers (new API in HAL):**
   - `int32_t tmc2240_get_vsupply_mV(uint16_t icID)` → returns mV (ADC × 9732 / 1000).
   - `int16_t tmc2240_get_temperature_c10(uint16_t icID)` → returns °C × 10 ( `(ADC-2038)*100/77` ).

---

## 🔧 Build Verification — PASSED

**Toolchain:** `C:\ST\STM32CubeIDE_2.0.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.14.3.rel1.win32_1.0.100.202602081740\tools\bin\arm-none-eabi-gcc.exe`

**Command run (from `Debug/`):**
```bash
make -j16 all
```

**Result:**
```
arm-none-eabi-size Test-TMC2240.elf
   text    data     bss     dec     hex
  19284      92    2676   22052    5624
```
**0 errors, 0 warnings.**

**Syntax checks (host GCC) — PASSED:**
```bash
# Register layer (no HAL deps)
gcc -std=gnu11 -Wall -Wextra -fsyntax-only Core/Inc/tmc/ic/TMC2240/TMC2240.c \
  -ICore/Inc/tmc/helpers -ICore/Inc/tmc/ic/TMC2240 \
  -ICore/Inc/tmc/platform/STM32 -ICore/Inc/tmc

# Cache-disabled config (now compiles)
gcc -std=gnu11 -Wall -Wextra -DTMC2240_CACHE=0 -fsyntax-only Core/Inc/tmc/ic/TMC2240/TMC2240.c \
  -ICore/Inc/tmc/helpers -ICore/Inc/tmc/ic/TMC2240 \
  -ICore/Inc/tmc/platform/STM32 -ICore/Inc/tmc
```

---

## 📋 Status

All phases implemented and build-verified. Remaining step is **hardware validation** by the user (see Safety Notes).

---

## 📁 Worktree State

- Modified: `TMC2240_HW_Abstraction.h`, `TMC2240.c`, `TMC2240.h`, `tmc2240_hal.c`, `tmc2240_hal.h`, `main.c`, `README.md`, `Test-TMC2240.ioc`
- Deleted: `TMC2240_motion.h`
- Untracked: `config.md` (user's dual-motor plan — left as-is)

---

## ⚠️ Safety Notes

- **Current formula**: Verify actual `RREF` value on the module (typically 12 kΩ on MKS TMC2240) before relying on `tmc2240_hal_setCurrent()` numbers.
- **SPI errors**: Call `tmc2240_hal_spiStatus(0)` in production code; expect `HAL_OK`.
- **UART debug**: The demo pauses STEP pulses ~2 ms once per second for diagnostics — acceptable for a demo, not for production motion control.
- **GLOBAL_SCALER**: Written as `0` (full scale) at init; the current formula reads it back.

---

*Completion record — supersedes the earlier draft plan. All items implemented and build-verified; hardware validation is the user's remaining step.*