# TMC2209-UART → STM32F446RE HAL + RTOS Implementation Plan

> **Historical porting plan, not the current integration contract.** The checked
> API hardening revision removes `SetupDefault` and implicit motor presets,
> replaces the transport/error contracts, and requires explicit configuration
> and activation. Follow `instruction.md` and the current headers instead of
> copying the obsolete examples below. Historical checklists are not release
> or hardware qualification evidence.

**Target MCU:** STM32F446RE (e.g. NUCLEO-F446RE)  
**Target IDE:** STM32CubeIDE + STM32CubeMX  
**Target HAL:** STM32F4xx HAL  
**Target RTOS:** FreeRTOS (CMSIS-RTOS v2 optional wrapper)  
**Source baseline:** Arduino / STM32duino `TMC2209-UART` (this repo)  
**License constraint:** Original is GPLv3 — derivative work remains GPLv3; keep license headers.

---

## 0. Objectives

| ID | Objective | Done when |
|----|-----------|-----------|
| O1 | Remove all Arduino / STM32duino dependencies | No `Arduino.h`, `HardwareSerial`, `delayMicroseconds` |
| O2 | Drop-in usable from CubeIDE C projects | Compiles with CubeMX-generated `main.c` + HAL only |
| O3 | Preserve TMC2209 UART protocol correctness | CRC, datagrams, register R/W match datasheet + original lib |
| O4 | Half-duplex single-wire UART on F446RE USART | `Available()` / `IOIN` / `IFCNT` work on real HW |
| O5 | RTOS-safe multi-task use | Concurrent callers cannot corrupt UART or shared unit state |
| O6 | Bare-metal still supported | Same API works with RTOS hooks stubbed to no-ops |

**Non-goals (v1):**
- STEP/DIR motion planner / acceleration profiles
- DMA-only high-throughput streaming (optional v1.1)
- Software bit-bang UART
- Support for non-STM32 HALs (can keep port layer open for later)

---

## 1. Current Library Analysis

### 1.1 File map (baseline)

```
src/TMC2209_REG.h      — Register bitfields, addresses, defaults (portable)
src/TMC2209_UNIT.h     — Per-chip config + HardwareSerial* (Arduino-bound)
src/TMC2209-UART.h/.cpp — Classes TMC2209_LL + TMC2209 (Arduino-bound I/O)
examples/...ino        — STM32duino half-duplex example @ 500000 baud
```

### 1.2 What is already portable

- Register unions / bitfields (`GCONF`, `IHOLD_IRUN`, `CHOPCONF`, …)
- Register addresses and default values
- CRC-8 (poly 0x07) with precomputed start CRCs per slave address
- 32-bit read-request and 64-bit write/reply datagram packing
- Byte-reversal of 32-bit payload
- High-level register write/read helpers and `setupDefault` / `available`

### 1.3 What must be replaced

| Baseline | Problem | HAL + RTOS replacement |
|----------|---------|------------------------|
| `#include <Arduino.h>` | Arduino framework | `stdint.h`, `stdbool.h`, `stm32f4xx_hal.h` |
| `HardwareSerial*` | C++ Arduino UART | `UART_HandleTypeDef*` via port layer |
| `write()` byte loop | Arduino stream API | `HAL_UART_Transmit` / IT / DMA |
| `read()` + `available()` poll | Arduino stream API | `HAL_UART_Receive` / IT + timeout |
| `flush()` | Wait TX complete | Wait `UART_FLAG_TC` + clear errors |
| `enableHalfDuplexRx()` | STM32duino-only | `HAL_HalfDuplex_EnableReceiver()` or HDSEL switch |
| `delayMicroseconds()` | Blocking Arduino delay | DWT µs delay (bare) or RTOS-aware delay policy |
| C++ `class` / ctor | C++ only | C structs + init functions (`extern "C"` capable) |

### 1.4 Protocol timing constraints (TMC2209 datasheet)

- UART: 8N1, typically 115200–500000 baud (chip auto-bauds from first frame edge)
- Write datagram: 8 bytes (SYNC, addr, reg|W, data32, CRC)
- Read request: 4 bytes; reply: 8 bytes from master address `0xFF`
- Chip reply delay depends on `SLAVECONF.SENDDELAY` (default small)
- Bus is **half-duplex / single wire** on `PDN_UART`
- TX echo may appear in RX FIFO — must flush before collecting reply
- Only one master transaction at a time per UART bus

### 1.5 Concurrency hazards (why RTOS matters)

1. **Shared UART bus** — two tasks calling read/write interleave bytes → CRC fail / lockup  
2. **Shared `TMC2209_UNIT` register cache** — torn read/write of mirrored registers  
3. **Half-duplex direction flips** — task switch mid-TX/RX leaves line in wrong state  
4. **Blocking delays inside critical sections** — must not hold scheduler lock across UART waits  
5. **ISR vs task** — if using UART IT/DMA, completion must signal tasks via semaphore/queue, not spin in ISR  

---

## 2. Architecture

### 2.1 Layered design

```
┌─────────────────────────────────────────────────────────┐
│  Application tasks (motion, UI, diagnostics)            │
└───────────────────────────┬─────────────────────────────┘
                            │  public C API
┌───────────────────────────▼─────────────────────────────┐
│  tmc2209.c  — high-level: SetupDefault, Available,      │
│               typed register helpers, unit cache        │
│               takes per-unit mutex                      │
└───────────────────────────┬─────────────────────────────┘
                            │
┌───────────────────────────▼─────────────────────────────┐
│  tmc2209_ll.c — CRC, datagrams, WriteReg/ReadReg        │
│               bus lock scope starts here for R/W        │
└───────────────────────────┬─────────────────────────────┘
                            │
┌───────────────────────────▼─────────────────────────────┐
│  tmc2209_port.h  — transport + OS abstraction (vtable)  │
└─────────────┬─────────────────────────────┬─────────────┘
              │                             │
┌─────────────▼─────────────┐ ┌─────────────▼─────────────┐
│ tmc2209_port_hal.c        │ │ tmc2209_os_freertos.c     │
│ UART HDX TX/RX, flush,    │ │ mutex, sem, delay,        │
│ µs/ms timebase hooks      │ │ critical-section stubs    │
└─────────────┬─────────────┘ └─────────────┬─────────────┘
              │                             │
        STM32 HAL UART                 FreeRTOS / CMSIS-RTOS2
```

### 2.2 Design principles

1. **C API first** — works in CubeIDE default C projects; C++ can wrap later.  
2. **Port interface (dependency inversion)** — protocol never calls HAL or FreeRTOS directly.  
3. **One lock per UART bus** — not per motor if motors share one USART.  
4. **Optional per-unit lock** — if unit cache is touched from multiple tasks.  
5. **Bare-metal profile** — OS hooks compile to no-ops / simple spin delay via `TMC2209_OS_NONE`.  
6. **Blocking API v1** — simpler and matches original; async API is Phase E.  
7. **No dynamic allocation in driver hot path** — caller owns `TMC2209_UNIT` / bus objects.  

### 2.3 Object model

```
TMC2209_Bus
  ├─ TMC2209_Port     (UART handle, baud, timeouts)
  ├─ TMC2209_OsMutex  (bus lock — one per physical UART)
  └─ list of units (logical; units hold back-pointer to bus)

TMC2209_Unit
  ├─ TMC2209_Bus*
  ├─ uint8_t address          (0..3 from MS1/MS2)
  ├─ register mirrors         (GCONF, IHOLD_IRUN, …)
  └─ optional TMC2209_OsMutex (unit cache lock)
```

**Rule:** All wire transactions acquire **bus mutex**; cache updates that span read-modify-write acquire **unit mutex** then **bus mutex** (fixed order to avoid deadlock: unit → bus).

---

## 3. Target repository / CubeIDE layout

```
TMC2209-UART/
├── implementation-plan.md          ← this document
├── LICENSE                         ← keep GPLv3
├── README.md                       ← update after port
│
├── Inc/
│   ├── tmc2209_reg.h               ← from TMC2209_REG.h (C)
│   ├── tmc2209_types.h             ← status codes, config structs
│   ├── tmc2209_unit.h              ← TMC2209_Unit + mirrors
│   ├── tmc2209_ll.h                ← low-level datagram API
│   ├── tmc2209.h                   ← public high-level API
│   ├── tmc2209_port.h              ← UART port interface
│   └── tmc2209_os.h                ← RTOS / bare-metal OS interface
│
├── Src/
│   ├── tmc2209_reg.c               ← optional: default initializers only
│   ├── tmc2209_ll.c                ← CRC + datagrams + Write/Read
│   ├── tmc2209.c                   ← SetupDefault, helpers
│   ├── tmc2209_port_hal.c          ← STM32 HAL half-duplex implementation
│   ├── tmc2209_os_none.c           ← bare-metal stubs
│   └── tmc2209_os_freertos.c       ← FreeRTOS implementation
│
├── Config/
│   └── tmc2209_conf_template.h     ← user copy as tmc2209_conf.h
│
├── Examples/
│   ├── baremetal_f446re/
│   │   └── app_tmc2209_example.c
│   └── freertos_f446re/
│       └── app_tmc2209_rtos_example.c
│
└── Tests/                          ← host-side (optional)
    └── test_crc_datagram.c
```

### 3.1 CubeIDE integration steps (user project)

1. Copy `Inc/`, `Src/`, and generate/copy `tmc2209_conf.h` from template.  
2. **Project → Properties → C/C++ General → Paths and Symbols → Includes** add `TMC2209-UART/Inc`.  
3. Add all `Src/*.c` to build **except** leave only one OS file:  
   - bare-metal: `tmc2209_os_none.c`  
   - FreeRTOS: `tmc2209_os_freertos.c`  
4. In CubeMX: enable USART half-duplex + FreeRTOS if needed.  
5. Call `TMC2209_BusInit` / `TMC2209_UnitInit` after `MX_USARTx_UART_Init()` (and after scheduler start only if using RTOS objects created post-init — see §7).  

---

## 4. Public API specification

### 4.1 Status codes

```c
typedef enum {
    TMC2209_OK = 0,
    TMC2209_ERR_PARAM,
    TMC2209_ERR_PORT,
    TMC2209_ERR_TIMEOUT,
    TMC2209_ERR_CRC,
    TMC2209_ERR_BUSY,       /* mutex timeout / bus locked */
    TMC2209_ERR_NODEV,      /* no reply / not present */
    TMC2209_ERR_OS
} tmc2209_status_t;
```

Do **not** overload `0xFFFFFFFF` as the only error path in the public API; keep internal sentinel if needed but surface `tmc2209_status_t` + out-parameters.

### 4.2 Configuration (`tmc2209_conf.h`)

```c
/* tmc2209_conf_template.h */
#ifndef TMC2209_CONF_H
#define TMC2209_CONF_H

/* OS selection: exactly one */
#define TMC2209_OS_NONE          0
#define TMC2209_OS_FREERTOS      1
/* #define TMC2209_OS_CMSIS_V2   2  — future */

#ifndef TMC2209_OS
#define TMC2209_OS               TMC2209_OS_FREERTOS
#endif

/* Defaults */
#ifndef TMC2209_DEFAULT_BAUD
#define TMC2209_DEFAULT_BAUD           115200u
#endif

#ifndef TMC2209_BUS_LOCK_TIMEOUT_MS
#define TMC2209_BUS_LOCK_TIMEOUT_MS    100u
#endif

#ifndef TMC2209_UART_TX_TIMEOUT_MS
#define TMC2209_UART_TX_TIMEOUT_MS     20u
#endif

#ifndef TMC2209_UART_RX_TIMEOUT_MS
#define TMC2209_UART_RX_TIMEOUT_MS     20u
#endif

#ifndef TMC2209_REPLY_POLL_US
#define TMC2209_REPLY_POLL_US          500u
#endif

/* 1 = use HAL_UART IT + FreeRTOS binary semaphore for RX complete */
#ifndef TMC2209_USE_UART_IT
#define TMC2209_USE_UART_IT            0
#endif

/* 1 = create mutexes inside BusInit; 0 = caller provides */
#ifndef TMC2209_OS_CREATE_MUTEX
#define TMC2209_OS_CREATE_MUTEX        1
#endif

#endif
```

### 4.3 Port interface

```c
/* tmc2209_port.h */
typedef struct tmc2209_port {
    UART_HandleTypeDef *huart;
    uint32_t baud;
    uint32_t tx_timeout_ms;
    uint32_t rx_timeout_ms;
    uint32_t reply_gap_us;     /* post-TX before RX, default ~8 bit times */
    void    *os_rx_signal;     /* optional semaphore for IT mode */
} tmc2209_port_t;

tmc2209_status_t TMC2209_Port_Init(tmc2209_port_t *port);
tmc2209_status_t TMC2209_Port_Deinit(tmc2209_port_t *port);
tmc2209_status_t TMC2209_Port_Transmit(tmc2209_port_t *port, const uint8_t *data, uint16_t len);
tmc2209_status_t TMC2209_Port_Receive(tmc2209_port_t *port, uint8_t *data, uint16_t len);
tmc2209_status_t TMC2209_Port_FlushRx(tmc2209_port_t *port);
void             TMC2209_Port_DelayUs(uint32_t us);
```

### 4.4 OS interface

```c
/* tmc2209_os.h */
typedef void *tmc2209_mutex_t;
typedef void *tmc2209_sem_t;

tmc2209_status_t TMC2209_Os_MutexCreate(tmc2209_mutex_t *m);
tmc2209_status_t TMC2209_Os_MutexDelete(tmc2209_mutex_t *m);
tmc2209_status_t TMC2209_Os_MutexLock(tmc2209_mutex_t m, uint32_t timeout_ms);
tmc2209_status_t TMC2209_Os_MutexUnlock(tmc2209_mutex_t m);

tmc2209_status_t TMC2209_Os_SemCreate(tmc2209_sem_t *s);
tmc2209_status_t TMC2209_Os_SemTake(tmc2209_sem_t s, uint32_t timeout_ms);
tmc2209_status_t TMC2209_Os_SemGive(tmc2209_sem_t s);      /* task context */
void             TMC2209_Os_SemGiveFromISR(tmc2209_sem_t s); /* ISR safe */

void             TMC2209_Os_DelayMs(uint32_t ms);
uint32_t         TMC2209_Os_GetTickMs(void);
```

**Bare-metal (`tmc2209_os_none.c`):**
- Mutex lock/unlock → always `TMC2209_OK` (no-op)
- Sem take → spin with timeout using tick/DWT
- DelayMs → `HAL_Delay`
- SemGiveFromISR → no-op or flag set

**FreeRTOS (`tmc2209_os_freertos.c`):**
- Mutex → `xSemaphoreCreateMutex` / `xSemaphoreTake` / `xSemaphoreGive`
- Sem → binary semaphore for UART IT completion
- DelayMs → `vTaskDelay(pdMS_TO_TICKS(ms))`
- GetTickMs → `xTaskGetTickCount() * portTICK_PERIOD_MS`
- FromISR → `xSemaphoreGiveFromISR` + `portYIELD_FROM_ISR`

### 4.5 Bus + unit

```c
typedef struct tmc2209_bus {
    tmc2209_port_t   port;
    tmc2209_mutex_t  lock;
} tmc2209_bus_t;

typedef struct tmc2209_unit {
    tmc2209_bus_t   *bus;
    uint8_t          address;     /* 0..3 */
    tmc2209_mutex_t  lock;        /* optional cache protection */

    /* WRITE mirrors */
    tmc2209_gconf_t       gconf;
    tmc2209_ihold_irun_t  ihold_irun;
    tmc2209_chopconf_t    chopconf;
    tmc2209_pwmconf_t     pwmconf;
    tmc2209_coolconf_t    coolconf;
    tmc2209_thrs_t        tcoolthrs;
    tmc2209_thrs_t        tpwmthrs;
    uint8_t               sgthrs;
    uint8_t               tpowerdown;

    /* READ mirrors */
    tmc2209_ioin_t        ioin;
    tmc2209_drv_status_t  drv_status;
    uint16_t              sg_result;
    uint8_t               ifcnt;
} tmc2209_unit_t;
```

### 4.6 High-level API

```c
tmc2209_status_t TMC2209_BusInit(tmc2209_bus_t *bus, UART_HandleTypeDef *huart, uint32_t baud);
tmc2209_status_t TMC2209_BusDeinit(tmc2209_bus_t *bus);

tmc2209_status_t TMC2209_UnitInit(tmc2209_unit_t *unit, tmc2209_bus_t *bus, uint8_t addr);
tmc2209_status_t TMC2209_UnitDeinit(tmc2209_unit_t *unit);

tmc2209_status_t TMC2209_SetupDefault(tmc2209_unit_t *unit);
tmc2209_status_t TMC2209_Available(tmc2209_unit_t *unit, bool *present);

tmc2209_status_t TMC2209_WriteReg(tmc2209_unit_t *unit, uint8_t reg, uint32_t data);
tmc2209_status_t TMC2209_ReadReg(tmc2209_unit_t *unit, uint8_t reg, uint32_t *data);

/* Typed helpers: value == use_cache sentinel means send unit->mirror */
tmc2209_status_t TMC2209_WriteGCONF(tmc2209_unit_t *unit, uint32_t value, bool use_cache);
tmc2209_status_t TMC2209_WriteIHOLD_IRUN(tmc2209_unit_t *unit, uint32_t value, bool use_cache);
/* ... same pattern for CHOPCONF, PWMCONF, COOLCONF, TCOOLTHRS, TPWMTHRS, SGTHRS, TPOWERDOWN */

tmc2209_status_t TMC2209_ReadIOIN(tmc2209_unit_t *unit, uint32_t *data);
tmc2209_status_t TMC2209_ReadSG_RESULT(tmc2209_unit_t *unit, uint32_t *data);
tmc2209_status_t TMC2209_ReadIFCNT(tmc2209_unit_t *unit, uint32_t *data);
```

**RTOS usage contract:**
- All public functions are **task-context only** (not ISR-safe), unless documented otherwise.  
- Callers may run from multiple tasks; driver serializes on bus lock.  
- Do not call public API from ISRs.  

---

## 5. Half-duplex UART strategy (F446RE)

### 5.1 CubeMX checklist

| Setting | Recommendation |
|---------|----------------|
| USART instance | Any free USART/UART (Nucleo VCP is USART2 — pick another for TMC if VCP used) |
| Mode | **Single Wire (Half-Duplex)** preferred |
| Baudrate | Start **115200**; then 250000 / 500000 |
| Word / parity / stop | 8N1 |
| DMA | Optional v1.1 |
| NVIC USART IRQ | Required if `TMC2209_USE_UART_IT == 1` |
| GPIO | Single pin = UARTx_TX (half-duplex); open-drain optional via external circuit |

### 5.2 Wiring

```
STM32 USARTx_TX  ----[ 0–1kΩ ]----  TMC2209 PDN_UART
GND -------------- GND
VM / VIO per module datasheet (logic 3.3V typical)
MS1/MS2 → address
ENN/STEP/DIR → app-specific (not this driver)
```

### 5.3 Transaction sequences

**Write (8 bytes):**
1. `MutexLock(bus)`
2. `FlushRx`
3. `HAL_HalfDuplex_EnableTransmitter` (or ensure TX mode)
4. `HAL_UART_Transmit` 8 bytes
5. Wait TC
6. `MutexUnlock(bus)`

**Read:**
1. `MutexLock(bus)`
2. `FlushRx`
3. Enable TX → transmit 4-byte request → wait TC
4. `FlushRx` again (drop echo if any)
5. Enable RX (`HAL_HalfDuplex_EnableReceiver`)
6. `DelayUs(reply_gap_us)` (~8 bit times: `8 * 1e6 / baud`)
7. Receive 8 bytes with timeout
8. CRC check → payload extract
9. `MutexUnlock(bus)`

### 5.4 Timebase

| Need | Implementation |
|------|----------------|
| Bit-time / reply gap (µs) | DWT CYCCNT busy-wait (short, holds CPU; OK under bus lock for &lt;1 ms) |
| Mutex / RX timeout (ms) | FreeRTOS ticks or `HAL_GetTick` |
| Task yield during long waits | Use RTOS delay / sem take — **never** `vTaskDelay` inside HAL ISR |

**DWT init (once at startup):**
```c
CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
DWT->CYCCNT = 0;
DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
/* delay_us = cycles / (SystemCoreClock/1e6) */
```

### 5.5 Echo / FIFO pitfalls

- After TX, RXDR may contain echoed bytes in single-wire mode depending on HW.  
- Always `FlushRx` before waiting for the 8-byte TMC reply.  
- Clear ORE, FE, NE, PE flags on error path and return `TMC2209_ERR_PORT`.  

---

## 6. Protocol port details (LL layer)

### 6.1 Keep algorithm identical to baseline

| Function (old) | Function (new) | Notes |
|----------------|----------------|-------|
| `getStartCRC` | `tmc2209_crc_start` | Addresses 0–3 only |
| `get1ByteCRC` | `tmc2209_crc_byte` | poly 0x07 |
| `get4ByteCRC` | `tmc2209_crc_u32` | LE byte walk |
| `reverseBytes` | `tmc2209_bswap32` | payload endian |
| `writeDatagram` | `tmc2209_pack_write` | → `uint8_t out[8]` preferred over `uint64_t` shift |
| `requestDatagram` | `tmc2209_pack_request` | → `uint8_t out[4]` |
| `respondData` | `tmc2209_unpack_reply` | CRC verify + bswap |

**Implementation note:** Prefer explicit `uint8_t` buffers over shifting `uint64_t` for endian clarity and MISRA-friendliness on Cortex-M4.

### 6.2 Constants

```c
#define TMC2209_SYNC              0x05u
#define TMC2209_REPLY_ADDR        0xFFu
#define TMC2209_WRITE_BIT         0x80u   /* reg address bit 7 = write */
#define TMC2209_CRC_START_A0      0x18u
#define TMC2209_CRC_START_A1      0x91u
#define TMC2209_CRC_START_A2      0xDFu
#define TMC2209_CRC_START_A3      0x56u
#define TMC2209_CRC_START_REPLY   0xEBu
```

Verify write-bit handling matches original `register_address` field usage when packing.

### 6.3 Register header conversion

- Replace C++ `namespace` with `typedef` + prefixed type names.  
- Keep bitfields; document that bitfield layout is GCC/Clang little-endian (CubeIDE default).  
- Move `TMC2209_REG_DEFAULT` values into `tmc2209_reg.h` as `static const` or macros.  
- Ensure `pdn_disable = 1` and UART-related defaults remain correct in `SetupDefault`.  

### 6.4 Host unit tests (optional but recommended)

Build `Tests/test_crc_datagram.c` on PC:
- Known vector: pack write → assert 8 bytes + CRC  
- Known vector: unpack reply → data + CRC fail cases  
- No HAL dependency  

---

## 7. RTOS integration design

### 7.1 Why a bus lock is mandatory

TMC2209 single-wire transactions are multi-step non-atomic sequences (TX request → gap → RX reply). Preemption between steps corrupts the bus. **Every** `WriteReg` / `ReadReg` must run under `bus->lock`.

### 7.2 Lock ordering (deadlock prevention)

```
Always:  unit->lock  →  bus->lock  →  (UART ops)  →  unlock bus  →  unlock unit
Never:   bus→unit
```

Helpers that only send a precomputed mirror value still take bus lock inside `WriteReg`.  
If a helper does RMW on the cache from the app side, app should either:
- use a single task for that unit, or  
- take `unit->lock` around cache edit + write helper.

### 7.3 Init order with FreeRTOS + CubeMX

CubeMX typically:
1. `HAL_Init`, clocks, peripherals  
2. `osKernelInitialize`  
3. Create tasks/mutexes  
4. `osKernelStart` — **no return**  

**Driver init options:**

| Option | When | How |
|--------|------|-----|
| A. Init in task | Recommended | First line of `StartDefaultTask`: `BusInit` / `UnitInit` / `SetupDefault` |
| B. Init before scheduler | Possible | Create mutexes with FreeRTOS API before `osKernelStart`; only if heap/scheduler primitives allow (static alloc preferred) |
| C. Static FreeRTOS objects | Best for production | `xSemaphoreCreateMutexStatic` buffers in `tmc2209_bus_t` |

**Recommendation:** static mutex storage inside `tmc2209_bus_t` / `tmc2209_unit_t` + create in `BusInit` called from first application task.

### 7.4 Blocking vs ISR UART modes

#### Mode 0 — Blocking HAL (v1 default, `TMC2209_USE_UART_IT == 0`)

```
Task: Lock → Transmit (blocking) → Receive (blocking) → Unlock
```

- Pros: simple, easy bring-up  
- Cons: task occupies CPU/core time during transfer; still yields only if HAL polling spins (it does not yield)  
- Mitigation: keep baud ≥ 115200 so 8 bytes ≪ 1 ms; acceptable for config traffic  

#### Mode 1 — Interrupt + semaphore (v1.1, RTOS-friendly)

```
Task: Lock → HAL_UART_Transmit_IT → SemTake(tx_done)
           → HAL_UART_Receive_IT  → SemTake(rx_done) → Unlock
ISR:  HAL_UART_TxCpltCallback / RxCpltCallback → SemGiveFromISR
```

- Pros: task blocks (other tasks run) during UART  
- Cons: more code; must handle HAL error callbacks  
- Required if UART transfers become long or many motors share bus under tight CPU load  

### 7.5 Priority and inversion

- Bus mutex should be **priority inheritance** mutex (FreeRTOS mutex, not binary semaphore).  
- UART ISR priority must be **≤ configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY** logically compatible (numerically ≥ in STM32 priority scheme) so `FromISR` APIs are legal.  
- Do not call driver from a task that holds another lock while waiting forever on bus — use `TMC2209_BUS_LOCK_TIMEOUT_MS`.  

### 7.6 Multi-motor topologies

| Topology | Locks | Notes |
|----------|-------|-------|
| 1 UART, 1 TMC | 1 bus mutex | Simplest |
| 1 UART, 4 TMC (addrs 0–3) | 1 shared bus mutex; 4 units | All traffic serialized on one wire — correct |
| 2 UART, N TMC | 1 bus mutex per UART | Parallel buses OK |
| Fake “parallel” on one UART from 2 tasks | Same bus mutex | Safe; second task gets `ERR_BUSY` or waits |

### 7.7 Watchdog / timeouts

- Every lock take and UART RX has a timeout → return `TMC2209_ERR_BUSY` / `TMC2209_ERR_TIMEOUT`.  
- Application may retry with backoff.  
- Never infinite wait in production builds (`portMAX_DELAY` only if explicitly configured).  

### 7.8 Stack and heap

| Object | Size guidance |
|--------|----------------|
| Task using driver | ≥ 512–1024 words if deep printf/debug |
| Mutex | static alloc inside bus/unit |
| Driver | no malloc in LL path |

### 7.9 CMSIS-RTOS v2 (optional later)

Map:
- `osMutexNew/Acquire/Release`  
- `osSemaphoreNew/Acquire/Release`  
- `osDelay` / `osKernelGetTickCount`  

Provide `tmc2209_os_cmsis_v2.c` only if needed; FreeRTOS direct is enough for CubeIDE.

### 7.10 What “RTOS compatible” guarantees (acceptance)

1. Two tasks calling `TMC2209_ReadIOIN` on the same bus never interleave bytes.  
2. Two tasks on different buses (different USART) proceed in parallel.  
3. Mutex timeout returns `TMC2209_ERR_BUSY` without corrupting state.  
4. Driver APIs are not used from ISRs.  
5. With IT mode enabled, blocked tasks yield the CPU while waiting for UART.  
6. Bare-metal build still links and runs the same examples without FreeRTOS.  

---

## 8. Phased implementation roadmap

### Phase 0 — Planning & scaffolding  
**Status:** this document  

- [x] Analyze baseline Arduino library  
- [x] Define layers, API, RTOS rules  
- [ ] Freeze `tmc2209_conf.h` defaults with project owner  

**Deliverables:** `implementation-plan.md`  

---

### Phase 1 — Headers & types (no HW)

**Tasks:**
1. Create `Inc/` structure.  
2. Convert `TMC2209_REG.h` → `tmc2209_reg.h` (C, no Arduino).  
3. Add `tmc2209_types.h`, `tmc2209_port.h`, `tmc2209_os.h`, `tmc2209_unit.h`, `tmc2209.h`.  
4. Add `Config/tmc2209_conf_template.h`.  
5. Ensure headers are `extern "C"` guarded.  

**Exit criteria:**
- Headers compile in empty CubeIDE C file with `#include "tmc2209.h"`.  
- No reference to Arduino types.  

**Estimate:** 0.5–1 day  

---

### Phase 2 — OS layer

**Tasks:**
1. Implement `tmc2209_os_none.c`.  
2. Implement `tmc2209_os_freertos.c` (mutex + delay + tick + optional sem).  
3. Document which single `.c` to add to the build.  

**Exit criteria:**
- FreeRTOS project links with mutex create/lock/unlock smoke test in a task.  
- Bare-metal project links with none backend.  

**Estimate:** 0.5 day  

---

### Phase 3 — Protocol LL (host-testable)

**Tasks:**
1. Port CRC + pack/unpack to `tmc2209_ll.c`.  
2. Prefer `uint8_t[8]` wire buffers.  
3. Add `Tests/test_crc_datagram.c` golden vectors captured from original lib or datasheet.  
4. Implement `WriteReg`/`ReadReg` calling port ops + bus lock.  

**Exit criteria:**
- CRC tests pass on host or on-target self-test.  
- Code review: lock order unit→bus respected.  

**Estimate:** 1 day  

---

### Phase 4 — STM32 HAL port (blocking)

**Tasks:**
1. `tmc2209_port_hal.c`: init, transmit, receive, flush, delay_us (DWT).  
2. Half-duplex direction helpers.  
3. Map HAL errors → `tmc2209_status_t`.  
4. `TMC2209_BusInit` / `UnitInit` / `SetupDefault` / `Available` / typed helpers.  

**Exit criteria:**
- On NUCLEO-F446RE + TMC2209 module: `Available == true`.  
- `IFCNT` increments after writes.  
- `IOIN` matches MS1/MS2/ENN.  

**Estimate:** 1–2 days (includes HW bring-up)  

---

### Phase 5 — High-level parity + example apps

**Tasks:**
1. Port all register helpers from original `TMC2209_LL`.  
2. `Examples/baremetal_f446re/app_tmc2209_example.c`.  
3. `Examples/freertos_f446re/app_tmc2209_rtos_example.c`:  
   - Task A: periodic `Available` + `ReadIOIN`  
   - Task B: periodic current tweak + `ReadIFCNT`  
   - Same bus → proves mutex  
4. Update `README.md` with CubeMX + wiring.  

**Exit criteria:**
- Example runs 10+ minutes without CRC/timeout errors under dual-task load.  

**Estimate:** 1 day  

---

### Phase 6 — RTOS hardening (IT mode + polish)

**Tasks:**
1. Optional `TMC2209_USE_UART_IT` path + callbacks.  
2. Static FreeRTOS mutex allocation.  
3. Stress test: 4 addresses / burst read-write.  
4. Error recovery: UART abort + re-init helper `TMC2209_Port_Recover`.  
5. (Optional) CMSIS-RTOS v2 backend.  

**Exit criteria:**
- IT mode passes same dual-task test with lower CPU (measure via FreeRTOS run-time stats if enabled).  
- Intentional bus conflict returns `ERR_BUSY`/`TIMEOUT`, recovers next call.  

**Estimate:** 1–2 days  

---

### Phase 7 — Cleanup

**Tasks:**
1. Deprecate or quarantine old Arduino `src/*.cpp` (subfolder `legacy_arduino/`) **or** keep dual-tree documented.  
2. Doxygen comments on public API.  
3. Final README: bare-metal vs FreeRTOS build matrices.  

**Estimate:** 0.5 day  

---

## 9. Implementation order (developer checklist)

Use this sequence when coding:

```
[ ] 1. tmc2209_conf_template.h
[ ] 2. tmc2209_types.h / tmc2209_reg.h
[ ] 3. tmc2209_os.h + os_none.c + os_freertos.c
[ ] 4. tmc2209_port.h + port_hal.c (stub RX/TX OK)
[ ] 5. tmc2209_ll.c (CRC/datagram + tests)
[ ] 6. tmc2209_unit.h + tmc2209.c (Bus/Unit/Setup/Available)
[ ] 7. Typed register helpers
[ ] 8. HW bring-up @ 115200 half-duplex
[ ] 9. Raise baud if stable
[ ] 10. FreeRTOS dual-task example
[ ] 11. IT mode (optional)
[ ] 12. README + legacy move
```

---

## 10. CubeMX / FreeRTOS recommended settings

### 10.1 FreeRTOS (CubeMX)

| Parameter | Suggestion |
|-----------|------------|
| Interface | CMSIS_V2 or FreeRTOS API (match OS backend file) |
| `configSUPPORT_STATIC_ALLOCATION` | 1 (preferred) |
| `configUSE_MUTEXES` | 1 |
| `configUSE_RECURSIVE_MUTEXES` | 0 (not required) |
| `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY` | default OK; USART IRQ must be compatible |
| Heap | heap_4 ≥ 8–16 KB depending on app |

### 10.2 USART

| Parameter | Suggestion |
|-----------|------------|
| Mode | Half-Duplex Single Wire |
| Baud | 115200 (bring-up) |
| Overrun | handle/clear in Flush/Recover |
| IRQ preemption | lower urgency than critical real-time ISR; FromISR-safe |

### 10.3 Example task split

```
InitTask (prio normal)
  BusInit / UnitInit / SetupDefault
  create CommTask, MonitorTask
  delete self

CommTask (prio normal)
  loop: adjust IHOLD_IRUN, delay 50 ms

MonitorTask (prio normal)
  loop: Available, ReadIOIN, ReadIFCNT, log, delay 100 ms
```

Both tasks share one `tmc2209_bus_t` → mutex serializes UART.

---

## 11. Testing plan

### 11.1 Unit (no HW)

| Test | Method |
|------|--------|
| CRC byte/u32 | Golden vectors |
| Pack write 8B | Compare to Arduino lib capture |
| Pack request 4B | Same |
| Unpack reply OK/fail CRC | Same |

### 11.2 Integration (HW, bare-metal)

| Test | Pass criteria |
|------|----------------|
| Link / listen | `Available` true |
| Defaults | `SetupDefault` returns OK |
| Write detect | `IFCNT` increases |
| Pin read | `IOIN.ms1/ms2` match straps |
| Noise | 1000 read loops, &lt;0.1% CRC fail |

### 11.3 Integration (HW, FreeRTOS)

| Test | Pass criteria |
|------|----------------|
| Dual-task same bus | No hardfault; no stuck mutex; low error rate |
| Lock timeout | Deliberate long hold → other task gets `ERR_BUSY` |
| Bus parallel | Two USARTs, two tasks, both OK |
| Priority smoke | Higher prio task still obtains bus within timeout |
| Soak | 30–120 min soak, watchdog fed |

### 11.4 Failure injection

- Disconnect PDN_UART → `ERR_TIMEOUT` / `ERR_NODEV`, next reconnect recovers  
- Wrong address → timeout, no crash  
- Baud mismatch → timeout  

---

## 12. Risks and mitigations

| Risk | Impact | Mitigation |
|------|--------|------------|
| Half-duplex echo mishandled | All reads fail | Aggressive FlushRx; scope line |
| Bitfield packing differs if other compiler | Wrong registers | Stick to GCC; static assert sizes |
| Holding bus lock too long | RTOS deadline miss | Short critical path; IT mode later |
| Calling API from ISR | Deadlock / assert | Document; assert `!xPortIsInsideInterrupt()` in debug |
| GPLv3 contagion | Legal | Keep license; isolate if product needs other license (rewrite clean-room) |
| Original lib bugs copied | Subtle CRC/reply bugs | Golden tests + datasheet cross-check |
| CubeMX reinit USART | Port stale handle | BusInit after MX init; no cached clocks wrong |
| 500k baud unstable wiring | Flaky | Start 115200; series R; short wires |

---

## 13. Success criteria (project complete)

- [ ] Zero Arduino dependencies in HAL port tree  
- [ ] Builds in STM32CubeIDE for STM32F446RE  
- [ ] Bare-metal example: `SetupDefault` + `Available` OK  
- [ ] FreeRTOS example: two tasks share one bus safely  
- [ ] Public API returns clear status codes  
- [ ] Register helpers cover original feature set  
- [ ] README documents wiring, CubeMX, RTOS config  
- [ ] License headers preserved (GPLv3)  

---

## 14. Open decisions (resolve before/during Phase 1)

| # | Decision | Options | Recommendation |
|---|----------|---------|----------------|
| D1 | Keep Arduino tree? | Delete / `legacy_arduino/` | `legacy_arduino/` |
| D2 | Language | C only / C++ wrapper | C only for v1 |
| D3 | Default baud | 115200 / 500000 | 115200 bring-up, conf override |
| D4 | IT UART in v1? | Yes / later | Later (Phase 6); blocking v1 |
| D5 | Static vs dynamic FreeRTOS objects | Static / dynamic | Static |
| D6 | CMSIS-RTOS2 backend | Now / later | Later |
| D7 | Naming | `TMC2209_` vs `tmc2209_` | `TMC2209_` functions, `tmc2209_` types |
| D8 | Motion (STEP/DIR) in tree? | Yes / no | No — separate module later |

---

## 15. Reference map (baseline → new)

| Baseline | New |
|----------|-----|
| `TMC2209_REG.h` | `Inc/tmc2209_reg.h` |
| `TMC2209_UNIT.h` | `Inc/tmc2209_unit.h` + bus type |
| `TMC2209_LL` class | `Src/tmc2209_ll.c` |
| `TMC2209` class | `Src/tmc2209.c` |
| `HardwareSerial` | `tmc2209_port_hal.c` |
| `delayMicroseconds` | `TMC2209_Port_DelayUs` (DWT) |
| `enableHalfDuplexRx` | HAL half-duplex enable RX |
| — | `tmc2209_os_freertos.c` / `tmc2209_os_none.c` |
| `uart-communication.ino` | `Examples/*/app_tmc2209_*.c` |

---

## 16. Minimal usage sketches

### 16.1 Bare-metal

```c
tmc2209_bus_t  bus;
tmc2209_unit_t tmc0;

void app_init(void)
{
    TMC2209_BusInit(&bus, &huart1, 115200);
    TMC2209_UnitInit(&tmc0, &bus, 0);
    if (TMC2209_SetupDefault(&tmc0) != TMC2209_OK) {
        Error_Handler();
    }
}
```

### 16.2 FreeRTOS task

```c
void vTmcTask(void *arg)
{
    tmc2209_bus_t  *bus = /* ... */;
    tmc2209_unit_t *u   = /* ... */;
    bool present;

    for (;;) {
        if (TMC2209_Available(u, &present) == TMC2209_OK && present) {
            uint32_t ioin;
            (void)TMC2209_ReadIOIN(u, &ioin);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
```

---

## 17. Timeline summary

| Phase | Content | Est. |
|-------|---------|------|
| 0 | Plan | done |
| 1 | Headers / types | 0.5–1 d |
| 2 | OS backends | 0.5 d |
| 3 | Protocol LL + tests | 1 d |
| 4 | HAL port + HW bring-up | 1–2 d |
| 5 | API parity + examples | 1 d |
| 6 | IT mode + hardening | 1–2 d |
| 7 | Docs / legacy | 0.5 d |
| **Total** | | **~6–9 working days** |

---

## 18. Next action

Proceed to **Phase 1**: create `Inc/` headers and `tmc2209_conf_template.h`, then Phase 2 OS stubs, without waiting for hardware.

When implementing, follow lock rules in §7 and transaction sequences in §5.3 exactly — those are the RTOS-safety and half-duplex correctness backbone of the port.
`)
