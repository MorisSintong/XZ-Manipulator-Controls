#include "model_core.h"
#include "test_support.h"

CoreFrame core_frames[256];
unsigned core_frame_count;
unsigned core_fail_at;
bool core_fail_after_accept;
TMC2240Status core_failure;
TMC2240Status core_bus_failure;
TMC2240Status core_node_failure;
TMC2240BusType core_buses[4];
uint8_t core_nodes[4];
unsigned core_count;
int core_corrupt_bit;
uint8_t core_reply_reserved;

void core_trace_reset(void)
{
    core_frame_count = 0U;
    core_fail_at = 0U;
    core_fail_after_accept = false;
    core_failure = TMC2240_ERROR_TIMEOUT;
    core_corrupt_bit = -1;
    core_reply_reserved = 0U;
}

void core_model_reset(void)
{
    model_reset_devices();
    core_trace_reset();
    core_count = 4U;
    core_bus_failure = TMC2240_OK;
    core_node_failure = TMC2240_OK;
    for (unsigned i = 0U; i < 4U; ++i) {
        core_buses[i] = IC_BUS_SPI;
        core_nodes[i] = (uint8_t)i;
        model_devices[i].reg[1] = 0U;
        model_devices[i].status_pipeline = 8U;
    }
}

static void record(uint16_t id, const uint8_t *data, size_t tx, size_t rx)
{
    CHECK(core_frame_count < 256U);
    CoreFrame *frame = &core_frames[core_frame_count++];
    frame->id = id;
    frame->tx_length = tx;
    frame->rx_length = rx;
    memcpy(frame->bytes, data, tx);
}

TMC2240Status tmc2240_getBusType(uint16_t icID, TMC2240BusType *bus)
{
    CHECK(bus != NULL);
    if (core_bus_failure != TMC2240_OK) {
        return core_bus_failure;
    }
    if (icID >= core_count) {
        return TMC2240_ERROR_ID;
    }
    *bus = core_buses[icID];
    return TMC2240_OK;
}

TMC2240Status tmc2240_getNodeAddress(uint16_t icID, uint8_t *node)
{
    CHECK(node != NULL);
    CHECK(icID < core_count);
    if (core_node_failure != TMC2240_OK) {
        return core_node_failure;
    }
    *node = core_nodes[icID];
    return TMC2240_OK;
}

TMC2240Status tmc2240_readWriteSPI(uint16_t icID, uint8_t *data, size_t length)
{
    uint8_t rx[5];
    CHECK(icID < core_count);
    CHECK(data != NULL && length == 5U);
    record(icID, data, 5U, 5U);
    bool fail = core_frame_count == core_fail_at;
    if (!fail || core_fail_after_accept) {
        model_spi(icID, data, rx);
        memcpy(data, rx, 5U);
    }
    if (fail) {
        memset(data, 0xFF, 5U);
        return core_failure;
    }
    return TMC2240_OK;
}

TMC2240Status tmc2240_readWriteUART(uint16_t icID, uint8_t *data,
                                   size_t writeLength, size_t readLength)
{
    uint8_t address = data[2] & 127U;
    CHECK(icID < core_count);
    CHECK(data != NULL);
    CHECK((data[0] & 15U) == 5U && data[1] == core_nodes[icID]);
    CHECK((writeLength == 4U && readLength == 8U) ||
          (writeLength == 8U && readLength == 0U));
    record(icID, data, writeLength, readLength);
    CHECK(data[writeLength - 1U] == model_crc(data, writeLength - 1U));
    bool fail = core_frame_count == core_fail_at;
    if (fail && !core_fail_after_accept) {
        return core_failure;
    }
    if (writeLength == 8U) {
        CHECK((data[2] & 128U) != 0U);
        model_write(icID, address, model_unpack(&data[3]));
        model_devices[icID].reg[2] = (model_devices[icID].reg[2] + 1U) & 255U;
    } else {
        CHECK((data[2] & 128U) == 0U);
        data[0] = 5U | core_reply_reserved;
        data[1] = 255U;
        data[2] = address;
        model_pack(&data[3], model_devices[icID].reg[address]);
        data[7] = model_crc(data, 7U);
        if (core_corrupt_bit >= 0) {
            data[(unsigned)core_corrupt_bit / 8U] ^= (uint8_t)(1U << ((unsigned)core_corrupt_bit % 8U));
        }
    }
    return fail ? core_failure : TMC2240_OK;
}
