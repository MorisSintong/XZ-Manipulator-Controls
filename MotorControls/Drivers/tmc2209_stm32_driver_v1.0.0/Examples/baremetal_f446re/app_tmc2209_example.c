/**
 * @file app_tmc2209_example.c
 * @brief Bare-metal TMC2209 usage example for NUCLEO-F446RE.
 *
 * Copy into your CubeIDE project's Core/Src/ (inside USER CODE blocks
 * in main.c, or as a separate file added to the build).
 *
 * CubeMX prerequisites:
 *   - USARTx configured in Single Wire (Half-Duplex) mode, 8N1, 115200
 *   - No HW flow control
 *
 * Wiring:
 *   USARTx_TX (e.g. PA9) ---[ optional 1kΩ ]--- TMC2209 PDN_UART
 *   GND ---------------------------------------- TMC2209 GND
 *   VM / VIO per module datasheet
 */

#include "main.h"
#include "app_tmc2209_example.h"

/* Adjust to match your CubeMX USART handle name */
extern UART_HandleTypeDef huart1;

/* Driver instances — statically allocated (no malloc) */
static tmc2209_bus_t  tmc_bus;
static tmc2209_unit_t tmc_unit0;
static bool attached;
static bool bus_owned;
static bool unit_owned;

static tmc2209_status_t first_error(tmc2209_status_t first, tmc2209_status_t cleanup)
{
    return first != TMC2209_OK ? first : cleanup;
}

/**
 * @brief Call once after MX_USARTx_UART_Init() in main.c USER CODE 2.
 */
tmc2209_status_t TMC2209_AppInit(void)
{
    if (bus_owned || unit_owned) {
        return TMC2209_ERR_STATE;
    }
    tmc2209_status_t st;

    /* 1. Attach bus to UART handle */
    st = TMC2209_BusInit(&tmc_bus, &huart1, 115200u);
    if (st != TMC2209_OK) {
        return st;
    }
    bus_owned = true;

    /* 2. Initialise unit at address 0 (MS1=L, MS2=L) */
    st = TMC2209_UnitInit(&tmc_unit0, &tmc_bus, 0u);
    if (st != TMC2209_OK) {
        return first_error(st, TMC2209_AppDeinit());
    }
    unit_owned = true;

    /* Identification never substitutes an implicit motor profile. */
    for (uint8_t attempt = 0u; attempt < 5u; ++attempt) {
        bool present = false;
        st = TMC2209_Available(&tmc_unit0, &present);
        if (st == TMC2209_OK && present) {
            attached = true;
            return TMC2209_OK;
        }
        if (st == TMC2209_OK) {
            st = TMC2209_ERR_NODEV;
        }
        HAL_Delay(10u);
    }

    return first_error(st, TMC2209_AppDeinit());
}

/**
 * @brief Example periodic poll — call from super-loop.
 *
 * Reads IOIN and IFCNT to confirm chip is alive.
 */
tmc2209_status_t TMC2209_AppPoll(void)
{
    if (!attached) {
        return TMC2209_ERR_STATE;
    }
    bool present = false;
    tmc2209_status_t st = TMC2209_Available(&tmc_unit0, &present);
    if (st != TMC2209_OK || !present) {
        return st != TMC2209_OK ? st : TMC2209_ERR_NODEV;
    }

    uint32_t ifcnt = 0u;
    return TMC2209_ReadIFCNT(&tmc_unit0, &ifcnt);
}

tmc2209_status_t TMC2209_AppConfigureMotor(const tmc2209_motor_config_t *config)
{
    return attached ? TMC2209_Configure(&tmc_unit0, config) : TMC2209_ERR_STATE;
}

tmc2209_status_t TMC2209_AppActivate(uint8_t toff)
{
    return attached ? TMC2209_Activate(&tmc_unit0, toff) : TMC2209_ERR_STATE;
}

tmc2209_status_t TMC2209_AppDeactivate(void)
{
    return attached ? TMC2209_Deactivate(&tmc_unit0) : TMC2209_ERR_STATE;
}

tmc2209_status_t TMC2209_AppClearGSTAT(uint8_t flags)
{
    return attached ? TMC2209_ClearGSTAT(&tmc_unit0, flags) : TMC2209_ERR_STATE;
}

tmc2209_status_t TMC2209_AppDeinit(void)
{
    if (!bus_owned && !unit_owned) {
        return TMC2209_ERR_STATE;
    }
    attached = false;
    if (unit_owned) {
        const tmc2209_status_t st = TMC2209_UnitDeinit(&tmc_unit0);
        if (st != TMC2209_OK) {
            return st;
        }
        unit_owned = false;
    }
    if (bus_owned) {
        const tmc2209_status_t st = TMC2209_BusDeinit(&tmc_bus);
        if (st != TMC2209_OK) {
            return st;
        }
        bus_owned = false;
    }
    return TMC2209_OK;
}
