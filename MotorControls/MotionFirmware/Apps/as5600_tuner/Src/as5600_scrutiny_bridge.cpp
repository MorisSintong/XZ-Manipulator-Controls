/**
 * @file    as5600_scrutiny_bridge.cpp
 * @brief   Scrutiny Debugger (scrutiny-embedded) bridge over SEGGER RTT ch0
 *          for MotionFirmware_AS5600Tuner.
 *
 * Exposes:
 *  - Global DWARF struct `g_scrutiny_as5600` (X and Z raw, filtered angle,
 *    unwrapped counts, degrees, magnitude, status, AGC, CONF, error count).
 *  - 500 Hz FixedFrequencyLoopHandler ("as5600_500hz") with 16 KiB RAM
 *    datalogging buffer in RAM_D1 for oscilloscope-style captures.
 *  - Runtime Published Values (RPVs) to read/write volatile AS5600 CONF
 *    (SF, HYST, FTH) live from Scrutiny GUI without touching OTP 0xFF:
 *      0x1000: X SF code   (0=16x, 1=8x, 2=4x, 3=2x)
 *      0x1001: X HYST code (0..3)
 *      0x1002: X FTH code  (0..7)
 *      0x1010: Z SF code   (0=16x, 1=8x, 2=4x, 3=2x)
 *      0x1011: Z HYST code (0..3)
 *      0x1012: Z FTH code  (0..7)
 */

#include "as5600_scrutiny_bridge.h"

#include "SEGGER_RTT.h"
#include "scrutiny.hpp"

#include <cstdint>

as5600_scrutiny_state_t g_scrutiny_as5600{};

namespace {

constexpr uint16_t kRpvXSf   = 0x1000U;
constexpr uint16_t kRpvXHyst = 0x1001U;
constexpr uint16_t kRpvXFth  = 0x1002U;
constexpr uint16_t kRpvZSf   = 0x1010U;
constexpr uint16_t kRpvZHyst = 0x1011U;
constexpr uint16_t kRpvZFth  = 0x1012U;

uint8_t s_rx_buf[128];
uint8_t s_tx_buf[256];
#if SCRUTINY_ENABLE_DATALOGGING
uint8_t s_datalog_buf[16384];
#endif

/* 500 Hz = 2000 us = 20000 ticks of 100 ns. */
scrutiny::FixedFrequencyLoopHandler s_loop_500hz(20000U, "as5600_500hz");
scrutiny::LoopHandler *s_loops[] = {&s_loop_500hz};

scrutiny::RuntimePublishedValue s_rpvs[] = {
    {kRpvXSf, scrutiny::VariableType::uint8},
    {kRpvXHyst, scrutiny::VariableType::uint8},
    {kRpvXFth, scrutiny::VariableType::uint8},
    {kRpvZSf, scrutiny::VariableType::uint8},
    {kRpvZHyst, scrutiny::VariableType::uint8},
    {kRpvZFth, scrutiny::VariableType::uint8},
};
#if SCRUTINY_SUPPORT_PROTECTED_REGIONS
/* Peripheral MMIO, external device and System (PPB/vendor) space on STM32H723. */
scrutiny::AddressRange s_forbidden_ranges[] = {
    {reinterpret_cast<void *>(0x40000000U), reinterpret_cast<void *>(0x5FFFFFFFU)},
    {reinterpret_cast<void *>(0x60000000U), reinterpret_cast<void *>(0xDFFFFFFFU)},
    {reinterpret_cast<void *>(0xE0000000U), reinterpret_cast<void *>(0xFFFFFFFFU)},
};
#endif

scrutiny::MainHandler s_main;
uint32_t s_last_us = 0U;
uint8_t s_rtt_tx_buf[64];
uint16_t s_rtt_tx_off = 0U;
uint16_t s_rtt_tx_len = 0U;
bool s_initialized = false;

bool rpv_read_cb(const scrutiny::RuntimePublishedValue rpv, scrutiny::AnyType *outval,
                 scrutiny::LoopHandler *caller) {
    (void)caller;
    uint8_t bus = 0U;
    uint8_t field = 0U;
    switch (rpv.id) {
        case kRpvXSf:   bus = 1U; field = 0U; break;
        case kRpvXHyst: bus = 1U; field = 1U; break;
        case kRpvXFth:  bus = 1U; field = 2U; break;
        case kRpvZSf:   bus = 0U; field = 0U; break;
        case kRpvZHyst: bus = 0U; field = 1U; break;
        case kRpvZFth:  bus = 0U; field = 2U; break;
        default: return false;
    }
    uint8_t sf = 3U, hyst = 0U, fth = 0U;
    uint16_t conf_rb = 0U;
    as5600_tuner_get_conf_for_scrutiny(bus, &sf, &hyst, &fth, &conf_rb);
    if (field == 0U) outval->uint8 = sf;
    else if (field == 1U) outval->uint8 = hyst;
    else outval->uint8 = fth;
    return true;
}

bool rpv_write_cb(const scrutiny::RuntimePublishedValue rpv, const scrutiny::AnyType *inval,
                  scrutiny::LoopHandler *caller) {
    (void)caller;
    if (rpv.type != scrutiny::VariableType::uint8) {
        return false;
    }
    uint8_t bus = 0U;
    uint8_t field = 0U;
    switch (rpv.id) {
        case kRpvXSf:   bus = 1U; field = 0U; break;
        case kRpvXHyst: bus = 1U; field = 1U; break;
        case kRpvXFth:  bus = 1U; field = 2U; break;
        case kRpvZSf:   bus = 0U; field = 0U; break;
        case kRpvZHyst: bus = 0U; field = 1U; break;
        case kRpvZFth:  bus = 0U; field = 2U; break;
        default: return false;
    }
    uint8_t sf = 3U, hyst = 0U, fth = 0U;
    uint16_t conf_rb = 0U;
    as5600_tuner_get_conf_for_scrutiny(bus, &sf, &hyst, &fth, &conf_rb);
    const uint8_t v = inval->uint8;
    if (field == 0U) {
        if (v > 3U) return false;
        sf = v;
    } else if (field == 1U) {
        if (v > 3U) return false;
        hyst = v;
    } else {
        if (v > 7U) return false;
        fth = v;
    }
    if (!as5600_tuner_apply_conf_from_scrutiny(bus, sf, hyst, fth, &conf_rb)) {
        return false;
    }
    if (bus == 1U) {
        g_scrutiny_as5600.x.conf = conf_rb;
    } else {
        g_scrutiny_as5600.z.conf = conf_rb;
    }
    return true;
}

} /* namespace */

extern "C" void as5600_scrutiny_init(uint32_t now_us) {
    SEGGER_RTT_Init();

    scrutiny::Config cfg;
    cfg.set_buffers(s_rx_buf, sizeof(s_rx_buf), s_tx_buf, sizeof(s_tx_buf));
    cfg.set_published_values(
        s_rpvs,
        static_cast<uint16_t>(sizeof(s_rpvs) / sizeof(s_rpvs[0])),
        rpv_read_cb,
        rpv_write_cb);
#if SCRUTINY_SUPPORT_PROTECTED_REGIONS
    cfg.set_forbidden_address_range(
        s_forbidden_ranges,
        static_cast<uint_least8_t>(sizeof(s_forbidden_ranges) / sizeof(s_forbidden_ranges[0])));
#endif
#if SCRUTINY_ENABLE_DATALOGGING
    cfg.set_datalogging_buffers(s_datalog_buf, sizeof(s_datalog_buf));
    cfg.set_loops(s_loops, sizeof(s_loops) / sizeof(s_loops[0]));
#endif
    cfg.display_name = "AS5600_Tuner_H723";
    cfg.memory_write_enable = false; /* Passive safety: allow RPV CONF writes only */

    s_main.init(&cfg);
    s_last_us = now_us;
    s_initialized = true;
}

extern "C" void as5600_scrutiny_poll(uint32_t now_us) {
    if (!s_initialized) {
        return;
    }

    uint8_t rx_chunk[64];
    const unsigned n_rx = SEGGER_RTT_Read(0U, rx_chunk, sizeof(rx_chunk));
    if (n_rx > 0U) {
        s_main.receive_data(rx_chunk, static_cast<uint16_t>(n_rx));
        g_scrutiny_as5600.rtt_rx_bytes += n_rx;
    }

    const uint32_t dt_us = now_us - s_last_us;
    s_last_us = now_us;
    const uint32_t dt_100ns = (dt_us > (UINT32_MAX / 10U)) ? UINT32_MAX : (dt_us * 10U);
    s_main.process(dt_100ns); /* Scrutiny timestep unit = 100 ns */

    /* Flush any pending bytes from a previous partial/full RTT write before popping more. */
    while (true) {
        if (s_rtt_tx_off < s_rtt_tx_len) {
            const unsigned rem = static_cast<unsigned>(s_rtt_tx_len - s_rtt_tx_off);
            const unsigned wrote = SEGGER_RTT_Write(0U, &s_rtt_tx_buf[s_rtt_tx_off], rem);
            g_scrutiny_as5600.rtt_tx_bytes += wrote;
            s_rtt_tx_off = static_cast<uint16_t>(s_rtt_tx_off + wrote);
            if (s_rtt_tx_off < s_rtt_tx_len) {
                break; /* RTT up-ring full; retry remainder on next poll */
            }
            s_rtt_tx_off = 0U;
            s_rtt_tx_len = 0U;
        }
        if (s_main.data_to_send() == 0U) {
            break;
        }
        s_rtt_tx_len = s_main.pop_data(s_rtt_tx_buf, sizeof(s_rtt_tx_buf));
        s_rtt_tx_off = 0U;
        if (s_rtt_tx_len == 0U) {
            break;
        }
    }
}

extern "C" void as5600_scrutiny_tick_500hz(void) {
    if (!s_initialized) {
        return;
    }
    g_scrutiny_as5600.loop_count++;
#if SCRUTINY_ENABLE_DATALOGGING
    s_loop_500hz.process();
#endif
}
