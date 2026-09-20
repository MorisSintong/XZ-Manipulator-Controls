#include "tmc2240.h"
#include "API_Header.h"
#include "Functions.h"
#include "helpers\CRC.h"
#include "helpers\Functions.h"
#include "helpers\API_Header.h"
#include "test_support.h"

int main(void)
{
    const RegisterField fields[] = {
#define F(n,m,s,a,b) TMC2240_##n##_FIELD,
#include "field_cases.h"
#undef F
    };
    CHECK(TMC2240_OK == 0);
    CHECK(TMC2240_SPI_MODE == 3U && TMC2240_SPI_FRAME_SIZE == 5U);
    CHECK(TMC2240_HAL_SPI_TIMEOUT_MS == 10U);
    CHECK(sizeof(uint32_t) == 4U);
    CHECK(sizeof(fields) / sizeof(fields[0]) == 114U);
    for (size_t i = 0U; i < sizeof(fields) / sizeof(fields[0]); ++i) {
        CHECK(fields[i].mask != 0U && fields[i].shift < 32U && fields[i].address < 128U);
    }
    return EXIT_SUCCESS;
}
