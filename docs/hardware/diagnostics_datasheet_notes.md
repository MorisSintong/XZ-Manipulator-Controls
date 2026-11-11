# Motor diagnostics: datasheet and repository reference

Research date: 2026-10-06. Documentation only; no firmware changes or hardware qualification.

## Sources and verification status

References below use **printed datasheet pages**, not PDF viewer page numbers. A derived value cites its inputs; recommendations are engineering choices, not manufacturer specifications.

| ID | Source | Status |
| --- | --- | --- |
| **A** | [Official AS5600 datasheet](https://www.infineon.com/assets/row/public/documents/30/49/infineon-as5600-datasheet-en.pdf), legacy ams **v1-06, 2018-Jun-20**, linked by [Infineon's AS5600 product page](https://www.infineon.com/part/AS5600) | VERIFIED: fetched using `web_fetch`, then PDF text read in memory. Front notice says Infineon acquired ownership effective **2026-Jul-01** and is distributing the unchanged legacy ams-OSRAM document. PDF viewer page = printed page + **2**, because of the notice/cover. [A, front notice/cover] |
| **T** | [Official ADI TMC2240 datasheet URL](https://www.analog.com/media/en/technical-documentation/data-sheets/tmc2240_datasheet.pdf); alternate official Trinamic URLs under `https://www.trinamic.com/fileadmin/assets/Products/ICs_Documents/` | **UNVERIFIED**: repeated `web_fetch` and direct retrieval attempts timed out; alternate `TMC2240_datasheet.pdf` and `TMC2240_datasheet_rev1.09.pdf` redirected to unreachable ADI support. No official TMC2240 PDF was available in this worktree. Do not interpret repository constants as independently verified datasheet facts. |
| **R-M** | [BOM/mechanics](bom_and_mechanics.md), §§2–3; task hardware specification | Design inputs, not independently measured mechanics |
| **R-W** | [Wiring](wiring.md), §§1–4 | Repository board connection specification |
| **R-P** | [UART protocol](../architecture/communication_protocol.md), §§1–3 | Repository protocol specification |
| **R-A** | `MotorControls\Drivers\AS5600\include\as5600.h`, `src\as5600.c`, `ports\stm32_hal\as5600_stm32_hal.{h,c}`, `docs\integration.md` | Directly inspected driver/API evidence |
| **R-T** | `MotorControls\Drivers\TMC2240-Driver\driver\inc\{tmc2240_hal.h,tmc2240_core.h,TMC2240_HW_Abstraction.h}`, `driver\src\{tmc2240_hal.c,TMC2240.c}` | Directly inspected driver/API evidence; hardware interpretation remains UNVERIFIED against T |
| **R-F** | `MotorControls\MotionFirmware\Core\Src\main.c`: `configure_tmc2240_motor`, `probe_until_stall`, `dump_driver_diagnostics`, peripheral initialization | Current single-axis firmware, not the future dual-axis design |

**Scope boundary:** motor-shaft sensing is not carriage-position sensing. Neither driver evidence nor shaft angle detects belt slip, pulley/coupler slip downstream of the magnet, screw backlash, compliance, or pitch/lead calibration errors.

## AS5600 measurement registers

All addresses, widths and bit positions in this table: **A p.18, Fig.21**; interpretation: **A pp.19–21, Figs.22–23**. Words are high byte first; mask reserved upper bits.

| Register | Address / significant bits | Diagnostics meaning |
| --- | --- | --- |
| STATUS | `0x0B`; MD bit **5**, ML bit **4**, MH bit **3** | MD: magnet detected. ML: maximum-gain overflow / weak magnet. MH: minimum-gain overflow / strong magnet. Require MD set and ML/MH clear; MD alone is insufficient. |
| RAW ANGLE | `0x0C` high, `0x0D` low; **12 bits**, **0…4095** | Datasheet calls it “unscaled and unmodified”; use for full-turn shaft tracking, not calibrated ANGLE. |
| ANGLE | `0x0E` high, `0x0F` low; **12 bits** | Scaled output angle; affected by programmed ZPOS and MPOS/MANG range. Includes fixed endpoint hysteresis of **10 LSB** at the full-turn boundary. This is separate from configurable HYST. |
| AGC | `0x1A`; **8 bits** | Gain indicator, not an angle or field-strength unit. Aim near the center of the supply-specific range. |
| MAGNITUDE | `0x1B` high, `0x1C` low; **12 bits** | Internal CORDIC magnitude; do not invent a counts-to-mT conversion. |
| ZPOS | `0x01/0x02`; **12 bits** | Start position for output-range programming |
| MPOS | `0x03/0x04`; **12 bits** | Stop position for output-range programming |
| MANG | `0x05/0x06`; **12 bits** | Maximum angular range; alternative range-programming method |
| CONF | `0x07/0x08`; documented bits **13:0** | Fields below; preserve factory/reserved bits during read-modify-write |
| ZMCO | `0x00`; bits **1:0** | Number of permanent angle burns, not revolution count |

**RAW versus ANGLE:** ZPOS/MPOS/MANG do not turn RAW ANGLE into a reduced-range encoder. ANGLE is the scaled output, and its fixed endpoint hysteresis makes it unsuitable for clean wrap detection. Configurable HYST is described as output hysteresis; RAW is described as unmodified. The text does **not explicitly specify register-by-register placement of configurable HYST**; that narrower claim is **UNVERIFIED**. Set HYST off rather than depending on an inferred bypass. “RAW” is not a promise of zero filtering latency. [A p.19 “ANGLE/RAW ANGLE”; pp.28–31 “Step Response”, “Hysteresis”]

### CONF field encoding

Every bit position and encoding below: **A p.19, Fig.22**; low-power/filter details: **A pp.28–31, Figs.32–36**.

| Field | Bits | Encoding |
| --- | --- | --- |
| PM | **1:0** | `00` normal; `01` LPM1; `10` LPM2; `11` LPM3 |
| HYST | **3:2** | `00` off; `01` **1 LSB**; `10` **2 LSB**; `11` **3 LSB** |
| OUTS | **5:4** | `00` analog **0–100%**; `01` analog **10–90%**; `10` PWM; `11` undocumented, do not select |
| PWMF | **7:6** | `00/01/10/11` → **115/230/460/920 Hz**, typical, **±5%** [A p.7, Fig.9] |
| SF | **9:8** | `00/01/10/11` → **16×/8×/4×/2×**; low-power modes force **16×** |
| FTH | **12:10** | `000` slow only; `001/010/011/100/101/110/111` → slow-to-fast thresholds **6/7/9/18/21/24/10 LSB** |
| WD | **13** | `0` off; `1` on. After angle stays within **4 LSB** for **1 minute**, watchdog enters LPM3. [A p.31, Fig.36] |

Fast-to-slow thresholds for FTH `001…111` are respectively **1/1/1/2/2/2/4 LSB**. Fast mode responds within **two full sampling periods**, then settles according to SF; it is not continuously active below its threshold. [A p.29, Fig.33; p.30, Fig.34]

### Resolution, accuracy, noise and response

| Quantity | Value / qualification | Citation |
| --- | --- | --- |
| Resolution | **12 bits**, **4096 counts/revolution**; derived **360/4096 = 0.087890625°/count** | A p.8, Fig.12; p.30, Fig.35 |
| System INL | **±1°** maximum, best-line-fit deviation, full-turn range, no magnet displacement, no zero programming, PWM/I²C conditions | A p.8, Fig.12 |
| Internal sampling period | **150 µs** typical; table names it “Sampling rate” but unit is time. Timing-condition typical-value tolerance **±5%** | A p.7, Fig.10 and note |
| Power-up | **10 ms** before acquisition | A p.7, Fig.10 |
| Register-setting propagation | Allow at least **1 ms** after each setting command; not equivalent to complete filter settling | A p.22, Fig.24 note; p.24, Fig.26 note |

With **FTH off**, the following table gives step-response delay and **maximum RMS output noise, one sigma**, not guaranteed instantaneous angle accuracy. [A p.28, Fig.32; settling times also A p.7, Fig.10]

| SF | Filter | Step response / settling | RMS output noise |
| --- | --- | --- | --- |
| `00` | **16×** | **2.2 ms** | **0.015°** |
| `01` | **8×** | **1.1 ms** | **0.021°** |
| `10` | **4×** | **0.55 ms** | **0.030°** |
| `11` | **2×** | **0.286 ms** | **0.043°** |

The INL figure is not a complete installed-system accuracy guarantee. Filter response creates dynamic lag; measure accuracy/precision only after a defined settle interval, and retain the filter profile with the result.

### Magnet installation and supply

| Item | Requirement | Citation |
| --- | --- | --- |
| AGC range | **0–128** at **3.3 V**, **0–255** at **5 V**; derived midpoints approximately **64/127.5** | A p.20, “AGC Register” |
| Recommended field | Orthogonal component **30–90 mT** at die surface along a circle of **1 mm**; magnet-detection level **8 mT** is not the recommended operating minimum | A p.8, Fig.11 |
| Air gap | Typical **0.5–3 mm**, magnet-dependent; adjust for centered AGC, not distance alone | A p.33, Fig.40 |
| Alignment | Maximum rotational-axis displacement **0.25 mm** for a **6 mm** diameter reference magnet | A p.33, “Magnetic Requirements” |
| Supply hookup | At **3.3 V**, tie VDD5V and VDD3V3 together; operating supply **3.0–3.6 V**, local **100 nF** decoupling per circuit | A p.9, Fig.13 |
| Rotation sign | DIR tied to GND: clockwise **viewed from sensor top** increases raw angle; assign separate mechanical signs for X and Z | A p.30, Fig.35 |

### I²C transfer rules and forbidden commands

| Topic | Verified rule | Citation |
| --- | --- | --- |
| Address/clock | Fixed **7-bit `0x36`**; sensor maximum **1 MHz**; no clock stretching | A p.10, “I²C Interface”; p.12, Fig.16 |
| Current HAL port | Restricts bus clock to **≤400 kHz** and shifts address once to **`0x6C`** for HAL. Two encoders use separate instances/buses, not different addresses. | R-A, `as5600_stm32_hal_init`, `stm32_read`; R-W §4 |
| Pointer | Register address is written into a pointer; ordinary sequential reads/writes increment it after each byte | A p.13, “I²C Modes” |
| Special pointer wrap | When initialized to the **high byte** of RAW ANGLE, ANGLE or MAGNITUDE, repeated reads cycle that register's byte pair instead of progressing into neighboring registers. Read each word separately; do not burst RAW→ANGLE expecting consecutive registers. | A p.13, “Automatic Increment…” |
| High/low consistency | Use one high-byte-first, two-byte transaction. **UNVERIFIED:** A does not explicitly guarantee high/low snapshot latching or coherence if the internal result changes mid-read. Nor is there a simultaneous latch of all diagnostic registers. Existing paired reads are not proof of atomic hardware sampling. | A pp.13,18; R-A `read_word`, `as5600_read_sample` |
| Timing examples | At sensor Fast-mode Plus limit: SCL low **≥0.5 µs**, high **≥0.26 µs**, bus-free **≥0.5 µs**, data setup **≥50 ns**, rise **≤120 ns**. These do not replace the controller's slower-mode requirements. | A p.12, Fig.16 |
| Forbidden OTP commands | **NEVER write `0x80` (BURN_ANGLE) or `0x40` (BURN_SETTING) to `0xFF`** in diagnostics, homing, startup or recovery. Angle burn limit **3**; setting burn limit **1**. MANG burn requires ZMCO **0**; angle burn requires MD set. | A pp.18,21 |
| Entire programming path | Keep runtime writes away from **all `0xFF` commands**, including OTP-reload sequence **`0x01,0x11,0x10`**. Do not enter OUT/PGO programming: pulling OUT low for **≥100 ms** in programming mode can permanently program the device. PGO must remain in normal operation. | A pp.22–24, Figs.24–26 |

Runtime zero is a software reference captured after homing, not an OTP operation. Leave range programming untouched for RAW tracking; read settings to document pre-existing programming.

## Derived kinematics and multi-turn limits

Inputs: motor full step **1.8°**, **200 full steps/rev**, external **1/16** microstepping → **3200 STEP events/rev**; X **40 mm/rev**, Z **8 mm/rev**. [R-M §§2–3; task hardware specification] Encoder: **4096 counts/rev**. [A p.8, Fig.12]

| Quantity | Formula | X | Z |
| --- | --- | --- | --- |
| Microstep angle | `1.8° / 16` | **0.1125°** | **0.1125°** |
| Encoder counts/microstep | `4096 / 3200` | **1.28** | **1.28** |
| STEP events/mm | `3200 / lead_mm` | **80** | **400** |
| mm/microstep | `lead_mm / 3200` | **0.0125** | **0.0025** |
| mm/encoder count | `lead_mm / 4096` | **0.009765625** | **0.001953125** |
| Single absolute-read rounding bound | `±lead_mm / (2 × 4096)` | **±0.0048828125 mm** | **±0.0009765625 mm** |
| Command rounding to nearest microstep | `±lead_mm / (2 × 3200)` | **±0.00625 mm** | **±0.00125 mm** |
| Ideal uniform single-read quantization RMS | `(lead_mm / 4096) / √12` | **0.0028191 mm** | **0.0005638 mm** |
| Linear equivalent of stated INL | `±1° × lead_mm / 360°` | **±0.111111 mm** | **±0.022222 mm** |

All table results are derived from R-M and A inputs above. Quantization is a resolution/error floor, **not proven accuracy or independent random noise**. A displacement subtracting two rounded encoder positions can have up to **one count** error; best-line-fit INL can differ between endpoints by up to **2°**, before additional installation error. [Derived from A p.8, Fig.12] One microstep is not guaranteed mechanical motion of exactly its nominal angle.

The received UART X coordinate has **0.1 mm** units. Quantizing an original continuous target to this format gives up to **±0.05 mm** rounding; this is not an additional error when comparing motion against the already-quantized received target. Target Y is a conveyor coordinate, **not Z**. [R-P §2; derived half-unit rounding]

### Safe unwrapping bound

Let `Δt` be the **actual time between accepted raw measurements**, including missed polls and jitter. Nearest-wrap unwrapping requires strictly:

`|Δθ| < 180°`, equivalently `|Δcounts| < 2048`.

For approximately constant speed, with `f_step` in external STEP events/s and `Δt` in seconds:

`|Δθ| = |f_step| × (360 / 3200) × Δt`

`|f_step| < 1600 / Δt`; `rpm < 30 / Δt`; `v_axis < lead_mm / (2 × Δt)`.

These are mathematical alias ceilings, **not usable motor/axis feed-rate ratings**. Inputs: half-turn uniqueness, A p.8 resolution, R-M motor/mechanics. Acceleration or reversal requires a bound on total angular travel over the gap, not just net commanded displacement.

| Nominal poll interval | Shaft-speed ceiling (strictly less) | STEP-event ceiling, both axes | X linear ceiling | Z linear ceiling |
| --- | --- | --- | --- | --- |
| **1 ms** | **30,000 rpm** | **1,600,000 steps/s** | **20,000 mm/s** | **4,000 mm/s** |
| **2 ms** | **15,000 rpm** | **800,000 steps/s** | **10,000 mm/s** | **2,000 mm/s** |
| **5 ms** | **6,000 rpm** | **320,000 steps/s** | **4,000 mm/s** | **800 mm/s** |

Derived from the preceding formulas; substitute the worst accepted-sample gap, not nominal scheduling. Use a substantial guard band for noise, byte coherence, filter lag, and fault recovery.

**Comparison to repository feed rates:** R-F `probe_until_stall` requests a Z cruise period of **500 µs**, nominal **2000 steps/s = 5 mm/s**, with start period **2000 µs = 500 steps/s**. At nominal cruise, angular travel per **1/2/5 ms** is **0.225/0.45/1.125°**, comfortably below the alias ceiling. GPIO, SPI and blocking logging add time; this is not measured maximum feed rate. **X maximum feed rate and Z production maximum feed rate are UNVERIFIED/not specified** in the inspected docs/current firmware. Validate each future `f_max_axis × Δt_max < 1600`, rather than inventing maxima. [R-F `probe_until_stall`; derived using R-M]

## TMC2240: repository evidence pending official datasheet verification

**Every hardware interpretation in this section is UNVERIFIED against T.** Numeric addresses/fields are explicitly **repository observations [R-T]**, not guessed datasheet specifications. The repository's historical audit points to Rev.2 **p.109** for MSCURACT and **pp.111–112** for CHOPCONF; `tmc2240_hal.h` points to Rev.2 **p.89** for TSTEP. These page references are themselves not independently checked. Do not use this section as electrical qualification.

### STEP evidence and configuration

| Item | Existing repository definition/evidence [R-T] | Limitation / verification needed |
| --- | --- | --- |
| MSCNT | `0x6A`, mask `0x3FF`, **10 bits**, **0…1023**; `tmc2240_get_microstep_counter` | Intended microstep-table phase, not a full-turn shaft position or persistent absolute position. Actual advancement with interpolation, DIR/SHAFT and stepping modes must be verified against T. |
| MSCURACT | `0x6B`; CUR_A bits **24:16**, CUR_B bits **8:0**, signed **9-bit**; helper decodes **−256…255** | Current-table evidence, not independent motor-position feedback or a measured winding-current ADC. Exact hardware scaling/semantics UNVERIFIED. |
| CHOPCONF | `0x6C`; MRES bits **27:24**, INTPOL bit **28**, DEDGE bit **29** | Verify interpolation and double-edge behavior before cross-checking STEP counts. |
| MRES helper | `tmc2240_hal_setMicrosteps` maps `0…8` to **256/128/64/32/16/8/4/2/1** microsteps/full step; codes above **8** rejected | Correctness versus official T remains UNVERIFIED; do not use `1 << MRES` as microsteps/full step. |
| Direction/filter bits | GCONF `0x00`: SHAFT bit **4**, MULTISTEP_FILT bit **3**, DIRECT_MODE bit **16** | Distinguish axis sign, driver direction inversion, pulse filtering and direct coil control; exact truth table/timing UNVERIFIED. |
| Direct coil mode | Repository register name **DIRECT_MODE**, `0x2D`; DIRECT_A bits **8:0**, DIRECT_B bits **24:16**, signed **9-bit** | Do not borrow XDIRECT/XACTUAL/ramp APIs from another chip. Existing docs say this device is STEP/DIR-only, with no internal ramp position register. Hardware confirmation pending T. |
| XENC | `0x39`; `tmc2240_encoder_read` | Separate quadrature encoder-interface count, not MSCNT or AS5600 I²C data. Current wiring has no AS5600 quadrature connection; do not call it measured shaft position. |

**Candidate phase model, NOT datasheet-verified:** with interpolation disabled and single-edge stepping, infer advancement `q = 256 / microsteps_per_fullstep = 2^MRES`. For the repository mapping:

| MRES | Microsteps/full step | Candidate MSCNT increment/accepted STEP edge |
| --- | --- | --- |
| **0/1/2/3/4/5/6/7/8** | **256/128/64/32/16/8/4/2/1** | **1/2/4/8/16/32/64/128/256** |

This is a **derived conditional model from R-T**, not verified pulse-acceptance semantics. If qualified, compare `(MSCNT_after − MSCNT_before) mod 1024` to `(signed_edges × q) mod 1024`. At **1/16**, that would advance **16** and alias every **64** edges, corresponding to **4 full steps / 7.2°** for this motor. [Conditional R-T model; R-M motor angle]

Modulo agreement cannot prove every STEP was received: missed/extra events can cancel or alias, and MSCNT cannot detect mechanical step loss. With INTPOL enabled, do not assume arbitrary moving-phase reads match this exact edge formula; verify interpolator timing or use a controlled non-interpolated diagnostic test. Reset, MRES changes and mode changes invalidate the baseline. Sampling during motion also needs a synchronized emitted-edge count.

### STEP/DIR timing: missing official evidence

| Required electrical limit | Result |
| --- | --- |
| Minimum STEP high time | **UNVERIFIED — no numeric value asserted** |
| Minimum STEP low time | **UNVERIFIED — no numeric value asserted** |
| DIR setup before accepted STEP edge | **UNVERIFIED — no numeric value asserted** |
| DIR hold after accepted STEP edge | **UNVERIFIED — no numeric value asserted** |
| Accepted edge with DEDGE clear/set, filter restrictions, external/internal clock dependencies | **UNVERIFIED** |

Current R-F uses **5 µs** STEP-high dwell and **20 µs** after DIR changes. These are application choices, **not datasheet minima**. The firmware worker must obtain T's STEP/DIR timing table before reducing either or qualifying double-edge operation. [R-F `probe_until_stall`, lines 276,313]

### Driver status and load registers

Addresses, bit positions and masks below are repository observations **[R-T, `TMC2240_HW_Abstraction.h`]**. Hardware meanings/operating validity remain **UNVERIFIED**.

| Register / field | Repository mapping | Diagnostics use / caution |
| --- | --- | --- |
| DRV_STATUS | `0x6F` | Preserve raw word as well as decoded fields |
| SG_RESULT | DRV_STATUS bits **9:0** | Load indicator; do **not** substitute for separate SG4_RESULT without confirming chopper/mode semantics |
| s2vsa / s2vsb | bits **12/13** | Short-to-supply flags, per repository naming |
| stealth | bit **14** | Record actual chopper mode with load evidence |
| cs_actual | bits **20:16** | Actual current-scale code, not amperes |
| stallguard | bit **24** | Status flag; distinguish from SG4 raw load and application stall policy |
| ot / otpw | bits **25/26** | Overtemperature / prewarning flags |
| s2ga / s2gb | bits **27/28** | Short-to-ground flags |
| ola / olb | bits **29/30** | Open-load flags; mode, current and speed validity conditions need T |
| stst | bit **31** | Driver standstill state, not proof the shaft/carriage is stationary |
| SG4_THRS | `0x74`; threshold bits **7:0**, filter-enable bit **8**, angle-offset bit **9** | Existing helper changes threshold only. Threshold comparison, effective sensitivity and filter-update cadence need T. |
| SG4_RESULT | `0x75`; bits **9:0** | `tmc2240_stallguard_read` reads this register. HAL header says bits **9 and 0** are zero and SG4 is for StealthChop; independently UNVERIFIED. |
| SG4_IND | `0x76`; indicator bytes at **7:0/15:8/23:16/31:24** | Per-indicator interpretation/update phase UNVERIFIED; retain raw word |
| GSTAT | `0x01`; reset **0**, drv_err **1**, uv_cp **2**, register_reset **3**, vm_uvlo **4** | Existing read helper is non-acknowledging; clear helper explicitly writes selected bits (W1C policy). Capture faults before any clear. |
| TSTEP | `0x12`; HAL documentation says **20-bit**, clock periods per **1/256 microstep** regardless of MRES | Counter saturation/standstill value, exact clock-frequency conversion and read update latency UNVERIFIED; do not call it a position counter |

**SPI pipeline:** `TMC2240.c::readTransport` sends **two 5-byte frames** to the same device, discards first frame's data and uses second frame's data. This implements next-transfer read response in the repository. Hardware protocol confirmation against T is **UNVERIFIED**, but callers should use this existing implementation, not decode the first transfer themselves. Serialize the whole operation on shared SPI2, including both frames and CS selection. SPI status-byte access exists separately. [R-T `readTransport`, `tmc2240_hal_getSPIStatus`]

## Repository API mapping and gaps

Header abbreviations:

- **AH:** `MotorControls\Drivers\AS5600\include\as5600.h`
- **AP:** `MotorControls\Drivers\AS5600\ports\stm32_hal\as5600_stm32_hal.h`
- **TH:** `MotorControls\Drivers\TMC2240-Driver\driver\inc\tmc2240_hal.h`
- **TC:** `MotorControls\Drivers\TMC2240-Driver\driver\inc\tmc2240_core.h`

| Requirement | Exact existing API / header | Remaining gap |
| --- | --- | --- |
| Raw / scaled angle | `as5600_read_raw_angle`, `as5600_read_angle` — AH | No turns, timestamps, freshness or wrap-loss detection; raw read alone does not validate magnet |
| STATUS / AGC / magnitude | `as5600_read_diagnostics`, `as5600_magnet_status` — AH | No standalone getters needed: diagnostics struct already exposes all. No periodic health policy. |
| Checked combined sample | `as5600_read_sample` — AH | Sequential, not simultaneous: STATUS→RAW→ANGLE→AGC→MAGNITUDE→STATUS. Outputs unchanged on error; not fresh on error. No sample timestamp or byte-coherence qualification. |
| All CONF fields | `as5600_read_config`, `as5600_write_config`, `as5600_default_config` — AH | No diagnostics profile/poll scheduler; default profile is not the proposed fast profile |
| ZPOS/MPOS/MANG/ZMCO | `as5600_read_settings`, `as5600_write_positions`, `as5600_write_max_angle` — AH | No gap for readback. Avoid changing these to establish homing zero. |
| Angle conversion | `as5600_counts_to_millidegrees` — AH | Single-turn only, rounded millidegrees; lacks signed multi-turn counts and axis displacement conversion |
| Supply/magnet specs and timing | No runtime specification accessor required; above tables are reference | Application must qualify magnet installation, deadlines and coherence |
| I²C transport / binding | `as5600_stm32_hal_init` — AP; `as5600_init` — AH | Two instances and actual I2C1/I2C3 initialization/integration are absent from current R-F; no asynchronous transfer API |
| OTP | **No burn API** — intentionally absent from AH | Keep absent; no addition needed |
| MSCNT / MSCURACT | `tmc2240_get_microstep_counter`, `tmc2240_get_microstep_current` — TH | **Accessors already exist.** Add application-level phase baseline/cross-check, not duplicate register getters. |
| MRES | `tmc2240_hal_setMicrosteps` — TH; readback via `tmc2240_fieldRead` — TC | Need snapshot/verification of configured MRES; check error results |
| INTPOL / DEDGE / SHAFT / direct/filter bits | `tmc2240_fieldRead`, `tmc2240_fieldWrite`, `tmc2240_updateRegister` — TC, with R-T field descriptors | Generic access sufficient; exact timing/phase qualification still required |
| DRV_STATUS | `tmc2240_driver_status` — TH; `tmc2240_fieldExtract` — TC | No application-owned timestamped decoded fault snapshot |
| SG4_RESULT / threshold | `tmc2240_stallguard_read`, `tmc2240_stallguard_set_threshold` — TH | No homing event/reference record; verify mode and calibrated stall policy |
| SG4 filter / offset / indicators | `tmc2240_fieldRead`, `tmc2240_fieldWrite`, `tmc2240_readRegister` — TC | No dedicated SG4_IND convenience getter, but generic access already works |
| GSTAT / SPI status | `tmc2240_check_faults`, `tmc2240_clear_faults`, `tmc2240_hal_getSPIStatus`, `tmc2240_hal_spiStatus` — TH | Capture-and-report policy, reset invalidation and shared-bus serialization belong to application |
| TSTEP | `tmc2240_readRegister` — TC, using `TMC2240_TSTEP` | No dedicated getter/conversion helper; generic read suffices |
| Quadrature XENC | `tmc2240_encoder_read`, `tmc2240_encoder_read_latch` — TH | Not connected to these I²C sensors; not a substitute for AS5600 tracking |
| STEP timing/count/angle | No driver STEP generator or signed emitted-edge counter | Application must count actual accepted-edge candidates, not just planned distance; preserve planned vs emitted distinction |
| Linear displacement / host diagnostics | No driver-level API | Software zero/home epoch, per-axis scale/sign, endpoint settling, trial identity and telemetry schema needed |

### Observed conflicts and cautions

| Location | Finding |
| --- | --- |
| R-F `dump_driver_diagnostics`, line 468 | Displays microstep divisor as `1 << mres`; R-T helper uses `256 >> mres`. At MRES **4**, both give **16** coincidentally; other encodings display incorrectly relative to R-T. Official T confirmation remains pending. |
| R-F lines 471–474 | Comment labels SG4_IND as **`0x74`**, and log labels SG4_RESULT **`0x74`**. R-T has threshold **`0x74`**, result **`0x75`**, indicators **`0x76`**. Actual `tmc2240_stallguard_read` uses R-T result address, so the demonstrated discrepancy is in labels, not that helper's read target. |
| R-F `configure_tmc2240_motor` | CHOPCONF **`0x14410150`** decodes by R-T as MRES **4**, INTPOL **1**, DEDGE **0**. Do not apply the non-interpolated phase formula blindly. [R-T fields; R-F initializer] |
| R-F startup and peripheral initialization | HAL initializes **one** TMC device; firmware initializes SPI2/UART2 but no I2C1/I2C3 acquisition or X motion. R-W documents dual-axis intended wiring, not proof it is initialized today. |
| R-F `probe_until_stall` | Blocking STEP dwell/SPI/logging loop cannot guarantee a periodic encoder acquisition deadline. Existing “steps moved” is emitted pulse count, not encoder-proven displacement. |
| R-M mechanics | **80 steps/mm X** and **400 steps/mm Z** match the stated motor/microstep/lead inputs. No steps/mm conflict found. Verify effective installed lead and microstep readback. |
| R-A write settling | Driver's **1 ms** propagation wait matches A's register-setting guidance, but does not cover SF **16×** full settling of **2.2 ms**. Do not confuse verified register contents with settled angle. [A pp.7,22; R-A `settle_configuration`] |
| R-P coordinates | Frame contains **X/Y**, not X/Z. Diagnostic commanded Z must come from the motion operation's own target; retain vision Y separately. Protocol docs describe future RTOS/DMA reception, whereas current R-F is a superloop test. |

## Implementation guidance for diagnostics firmware

These are recommendations for the firmware worker, **not implemented behavior**.

1. **Encoder profile:** use volatile CONF documented-field value **`0x0300`**: PM normal, HYST off, OUTS analog full/default, PWMF default, SF **2×**, FTH off, WD off. Preserve upper factory bits through `as5600_write_config`, then read back. [Derived encoding: A p.19, Fig.22; R-A API] OUT is unused, so its mode is not the measurement source. This profile gives stated response **0.286 ms** and noise **0.043° RMS**. [A p.28, Fig.32] A slower low-noise endpoint profile is optional but must be reported and allowed to settle; do not change profiles silently during a trial.
2. **Acquisition cadence:** start with a **2 ms** target period per axis at **400 kHz**, subject to measured worst-case execution time and the actual-sample-gap bound above; this is a design choice, not a guaranteed deadline. A full `as5600_read_sample` issues **six** transactions per sensor (**three** one-byte, **three** two-byte reads). Conventional address/pointer/data/ACK framing gives a derived **243 SCL clocks**, approximately **0.6075 ms** per sensor or **1.215 ms** for both serialized at **400 kHz**, before START/STOP gaps, HAL overhead, SPI and logging. At **100 kHz**, the corresponding lower bound is **4.86 ms** for both, so that full-sample strategy cannot meet the proposed period. [R-A `as5600_read_sample`; A pp.10–13 I²C protocol; derived wire budget] Separate scheduled raw-angle acquisition from slower health snapshots if necessary; label health age and faults. Do not issue blocking HAL calls inside STEP timing/ISRs.
3. **Unwrap independently for each axis:** maintain last accepted raw count, signed wide accumulated count, raw-read timestamp, validity and home epoch. For current `r` and previous `p`, form `d = r − p`; if `d > 2048`, subtract **4096**; if `d < −2048`, add **4096**. Reject the exactly-half-turn case rather than arbitrarily choosing a direction. Add `d` only after healthy/fresh acquisition and a conservative speed×actual-gap check. [Derived from A p.8 resolution and half-turn uniqueness] On NACK, bad magnet, incoherent sample, excessive gap or reset, do not reuse unchanged output as new data; invalidate continuity if a turn could have been missed. A scheduler overrun must be observable. Test both directions across wraps, reversals, missing samples and magnet faults.
4. **Home reference:** after qualified StallGuard4 homing, backoff/re-approach and settling as defined by the motion design, snapshot raw/unwrapped reference and emitted-edge reference for that motor. Set linear zero in software, record homing event/epoch and load evidence. Avoid burning or changing AS5600 range. Retain actual raw angle at home; do not force it to physical encoder zero. Homing repeatability is part of measured system error, not removed by a modulo phase match.
5. **MSCNT cross-check:** first obtain and verify T, particularly timing and interpolation semantics. Qualification model is `q = 2^MRES`, `expected = (signed_emitted_edges × q) mod 1024`, `observed = (phase_end − phase_start) mod 1024`, with fixed mode/MRES and calibrated driver phase sign. At the stated resolution, conditional `q = 16`. [R-T-derived model above, **UNVERIFIED against T**] Count rising edges only if qualified DEDGE is clear; with double-edge mode, count accepted edges rather than pulses. Prefer stationary endpoint snapshots for stable comparison; record read acquisition time/count boundaries. Distinguish MCU-command emission, electrical driver acceptance evidence and actual shaft motion. Never use MSCNT as a full-turn absolute position.
6. **Displacement arithmetic:** keep signed unwrapped encoder counts, not repeatedly rounded single-turn degrees. Derive `shaft_degrees = counts × 360 / 4096`, `linear_mm = axis_sign × (counts − home_counts) × lead_mm / 4096`, and `commanded_mm = signed_emitted_steps × lead_mm / 3200`. [A p.8; R-M; derived] Keep planned steps separately; snapshot at defined operation boundaries. Use independent encoder/driver direction calibrations.
7. **Recommended records:** include command/object sequence and trial ID; axis/motor/bus identity; received X/Y command values and units; operation's Z target; planned and emitted signed steps; full-step angle, MRES, INTPOL, DEDGE and direction sign; start/end MSCNT and timestamps; MSCURACT; TSTEP; raw DRV_STATUS/GSTAT/SPI status and decoded faults; SG4 result/threshold/indicators and homing evidence; AS raw count, unwrapped count, home count/epoch, STATUS/AGC/MAGNITUDE, CONF/settings; raw-read timestamp, health age, actual maximum sample gap, acquisition errors and continuity validity; converted angle/displacement; endpoint settle condition; outcome/abort reason. Host accuracy uses signed bias against the received-command reference; precision requires repeated comparable trials, not successive moving samples.

**Outstanding UNVERIFIED items:** all TMC2240 hardware claims/electrical timing, exact interpolated MSCNT behavior and SG4/TSTEP operating/update semantics; AS5600 high/low hardware coherence and explicit configurable-HYST register-path detail; production X/Z maximum feeds and physical installation accuracy. Obtain the official T PDF and close those qualification items before treating this reference as complete hardware validation.
