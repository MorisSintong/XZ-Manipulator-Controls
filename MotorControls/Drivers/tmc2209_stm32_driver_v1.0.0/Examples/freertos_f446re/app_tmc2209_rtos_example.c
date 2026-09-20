#include "app_tmc2209_rtos_example.h"
#include "main.h"
#include "cmsis_os2.h"

#include <stdatomic.h>
#include <stddef.h>
#include <stdint.h>

#if !TMC2209_OS_FREERTOS || TMC2209_OS_NONE || !TMC2209_OS_CREATE_MUTEX
#error "This example requires the native FreeRTOS backend with its own bus mutex"
#endif

enum { APP_COLD, APP_STARTING, APP_READY, APP_FAILED };
enum { PROBE_ATTEMPTS = 5 };

static tmc2209_bus_t bus = {0};
static tmc2209_unit_t unit = {0};
static osMutexId_t unit_mutex;
static osThreadId_t workers[2];
static bool profile_configured;
static uint32_t mutex_ticks;
static uint32_t retry_ticks;
static uint32_t counter_ticks;
static uint32_t monitor_ticks;
static uint32_t startup_ticks;
static atomic_int lifecycle = APP_COLD;
static atomic_int first_error = TMC2209_OK;
static atomic_int cleanup_error = TMC2209_OK;
static atomic_bool fault_notified = false;

static tmc2209_status_t latched_error(void)
{
    return (tmc2209_status_t)atomic_load(&first_error);
}

static void fail_service(tmc2209_status_t status)
{
    if (status != TMC2209_OK) {
        int expected = TMC2209_OK;
        (void)atomic_compare_exchange_strong(&first_error, &expected, (int)status);
        atomic_store(&lifecycle, APP_FAILED);
    }
}

static void fail_cleanup(tmc2209_status_t status)
{
    if (status != TMC2209_OK) {
        int expected = TMC2209_OK;
        (void)atomic_compare_exchange_strong(&cleanup_error, &expected, (int)status);
        fail_service(status);
    }
}

bool TMC2209_RtosIsReady(void)
{
    return atomic_load(&lifecycle) == APP_READY && latched_error() == TMC2209_OK;
}

tmc2209_status_t TMC2209_RtosGetStatus(void)
{
    const tmc2209_status_t status = latched_error();
    return status != TMC2209_OK ? status :
        (TMC2209_RtosIsReady() ? TMC2209_OK : TMC2209_ERR_STATE);
}

tmc2209_status_t TMC2209_RtosGetCleanupStatus(void)
{
    if (atomic_load(&lifecycle) == APP_COLD) {
        return TMC2209_ERR_STATE;
    }
    return (tmc2209_status_t)atomic_load(&cleanup_error);
}

static tmc2209_status_t mutex_error(osStatus_t status)
{
    if (status == osErrorTimeout) {
        return TMC2209_ERR_TIMEOUT;
    }
    return status == osErrorResource ? TMC2209_ERR_BUSY : TMC2209_ERR_OS;
}

static bool ticks_from_ms(uint32_t milliseconds, uint32_t frequency, uint32_t *ticks)
{
    const uint64_t value = ((uint64_t)milliseconds * frequency + 999U) / 1000U;
    if (value == 0U || value >= UINT32_MAX) {
        return false;
    }
    *ticks = (uint32_t)value;
    return true;
}

static tmc2209_status_t finish_operation(tmc2209_status_t status, bool latch_failure)
{
    if (latch_failure) {
        fail_service(status);
    }
    if (osMutexRelease(unit_mutex) != osOK) {
        fail_cleanup(TMC2209_ERR_OS);
    }
    const tmc2209_status_t failure = latched_error();
    return failure == TMC2209_OK ? status : failure;
}

static tmc2209_status_t begin_operation(void)
{
    if (!TMC2209_RtosIsReady()) {
        return TMC2209_ERR_STATE;
    }
    const osStatus_t acquired = osMutexAcquire(unit_mutex, mutex_ticks);
    if (acquired != osOK) {
        const tmc2209_status_t status = mutex_error(acquired);
        fail_service(status);
        return status;
    }
    if (!TMC2209_RtosIsReady()) {
        return finish_operation(TMC2209_ERR_STATE, false);
    }
    return TMC2209_OK;
}

tmc2209_status_t TMC2209_RtosConfigure(const tmc2209_motor_config_t *config)
{
    if (!TMC2209_RtosIsReady()) {
        return TMC2209_ERR_STATE;
    }
    if (config == NULL) {
        return TMC2209_ERR_PARAM;
    }
    const tmc2209_status_t entered = begin_operation();
    if (entered != TMC2209_OK) {
        return entered;
    }
    profile_configured = false;
    const tmc2209_status_t status = TMC2209_Configure(&unit, config);
    if (status == TMC2209_OK) {
        profile_configured = true;
    }
    return finish_operation(status, true);
}

tmc2209_status_t TMC2209_RtosActivate(uint8_t toff)
{
    const tmc2209_status_t entered = begin_operation();
    if (entered != TMC2209_OK) {
        return entered;
    }
    if (!profile_configured) {
        return finish_operation(TMC2209_ERR_STATE, false);
    }
    return finish_operation(TMC2209_Activate(&unit, toff), true);
}

tmc2209_status_t TMC2209_RtosDeactivate(void)
{
    const tmc2209_status_t entered = begin_operation();
    if (entered != TMC2209_OK) {
        return entered;
    }
    if (!profile_configured) {
        return finish_operation(TMC2209_ERR_STATE, false);
    }
    profile_configured = false;
    return finish_operation(TMC2209_Deactivate(&unit), true);
}

tmc2209_status_t TMC2209_RtosClearGSTAT(uint8_t flags)
{
    const tmc2209_status_t entered = begin_operation();
    if (entered != TMC2209_OK) {
        return entered;
    }
    profile_configured = false;
    return finish_operation(TMC2209_ClearGSTAT(&unit, flags), true);
}

static tmc2209_status_t diagnostics_locked(tmc2209_rtos_diagnostics_t *out)
{
    bool present = false;
    tmc2209_status_t status = TMC2209_Available(&unit, &present);
    if (status == TMC2209_OK && !present) {
        status = TMC2209_ERR_NODEV;
    }
    if (status == TMC2209_OK) {
        status = TMC2209_ReadIFCNT(&unit, &out->ifcnt);
    }
    if (status == TMC2209_OK) {
        status = TMC2209_ReadGSTAT(&unit, &out->gstat);
    }
    if (status == TMC2209_OK && profile_configured &&
        (out->gstat & UINT32_C(0x07)) != 0U) {
        status = TMC2209_ERR_FAULT;
    }
    if (status == TMC2209_OK) {
        status = TMC2209_ReadDRV_STATUS(&unit, &out->drv_status);
    }
    if (status == TMC2209_OK && profile_configured &&
        (out->drv_status & UINT32_C(0x3F)) != 0U) {
        status = TMC2209_ERR_FAULT;
    }
    return status;
}

tmc2209_status_t TMC2209_RtosReadDiagnostics(tmc2209_rtos_diagnostics_t *out)
{
    if (!TMC2209_RtosIsReady()) {
        return TMC2209_ERR_STATE;
    }
    if (out == NULL) {
        return TMC2209_ERR_PARAM;
    }
    const tmc2209_status_t entered = begin_operation();
    if (entered != TMC2209_OK) {
        return entered;
    }
    tmc2209_rtos_diagnostics_t sample = {0};
    const tmc2209_status_t status = finish_operation(diagnostics_locked(&sample), true);
    if (status == TMC2209_OK) {
        *out = sample;
    }
    return status;
}

static void notify_async_failure(void)
{
    if (!atomic_exchange(&fault_notified, true)) {
        Error_Handler();
    }
}

static _Noreturn void park_failed_worker(void)
{
    notify_async_failure();
    /* A failed release can leave this task owning the mutex: do not delete it. */
    for (;;) {
        if (osDelay(1U) != osOK) {
            fail_cleanup(TMC2209_ERR_OS);
        }
    }
}

static bool await_startup(void)
{
    /* Newly resumed tasks can preempt the initializer before READY is committed. */
    for (uint32_t waited = 0U; waited < startup_ticks; ++waited) {
        if (TMC2209_RtosIsReady()) {
            return true;
        }
        if (atomic_load(&lifecycle) != APP_STARTING) {
            return false;
        }
        if (osDelay(1U) != osOK) {
            fail_service(TMC2209_ERR_OS);
            return false;
        }
    }
    fail_service(TMC2209_ERR_TIMEOUT);
    return false;
}

static void CounterTask(void *argument)
{
    (void)argument;
    if (!await_startup()) {
        park_failed_worker();
    }
    for (;;) {
        tmc2209_status_t status = begin_operation();
        if (status == TMC2209_OK) {
            uint32_t counter = 0U;
            status = finish_operation(TMC2209_ReadIFCNT(&unit, &counter), true);
        }
        if (status != TMC2209_OK) {
            park_failed_worker();
        }
        if (osDelay(counter_ticks) != osOK) {
            fail_service(TMC2209_ERR_OS);
            park_failed_worker();
        }
    }
}

static void MonitorTask(void *argument)
{
    (void)argument;
    if (!await_startup()) {
        park_failed_worker();
    }
    for (;;) {
        tmc2209_rtos_diagnostics_t sample;
        if (TMC2209_RtosReadDiagnostics(&sample) != TMC2209_OK) {
            park_failed_worker();
        }
        if (osDelay(monitor_ticks) != osOK) {
            fail_service(TMC2209_ERR_OS);
            park_failed_worker();
        }
    }
}

static void cleanup_startup(bool bus_owned, bool unit_owned, bool mutex_held)
{
    if (unit_mutex == NULL) {
        return;
    }
    if (!mutex_held && (bus_owned || unit_owned)) {
        const osStatus_t acquired = osMutexAcquire(unit_mutex, 0U);
        if (acquired != osOK) {
            fail_cleanup(mutex_error(acquired));
            return;
        }
        mutex_held = true;
    }
    if (mutex_held) {
        if (unit_owned) {
            const tmc2209_status_t status = TMC2209_UnitDeinit(&unit);
            fail_cleanup(status);
            unit_owned = status != TMC2209_OK;
        }
        if (bus_owned && !unit_owned) {
            fail_cleanup(TMC2209_BusDeinit(&bus));
        }
        if (osMutexRelease(unit_mutex) != osOK) {
            fail_cleanup(TMC2209_ERR_OS);
            return;
        }
    }
    if (osMutexDelete(unit_mutex) == osOK) {
        unit_mutex = NULL;
    } else {
        fail_cleanup(TMC2209_ERR_OS);
    }
}

tmc2209_status_t TMC2209_RtosInit(UART_HandleTypeDef *uart, uint32_t baud,
                                 uint8_t address)
{
    if (atomic_load(&lifecycle) != APP_COLD) {
        return TMC2209_ERR_STATE;
    }
    if (uart == NULL || baud == 0U || address > 3U) {
        return TMC2209_ERR_PARAM;
    }
    if (osKernelGetState() != osKernelRunning) {
        return TMC2209_ERR_STATE;
    }
    int expected = APP_COLD;
    if (!atomic_compare_exchange_strong(&lifecycle, &expected, APP_STARTING)) {
        return TMC2209_ERR_STATE;
    }
    const uint32_t frequency = osKernelGetTickFreq();
    if (!ticks_from_ms(2000U, frequency, &mutex_ticks) ||
        !ticks_from_ms(50U, frequency, &retry_ticks) ||
        !ticks_from_ms(2000U, frequency, &counter_ticks) ||
        !ticks_from_ms(500U, frequency, &monitor_ticks) ||
        !ticks_from_ms(1000U, frequency, &startup_ticks)) {
        fail_service(TMC2209_ERR_OS);
        return latched_error();
    }
    const osMutexAttr_t mutex_attributes = {
        .name = "TmcUnit",
        .attr_bits = osMutexPrioInherit,
    };
    unit_mutex = osMutexNew(&mutex_attributes);
    if (unit_mutex == NULL) {
        fail_service(TMC2209_ERR_OS);
        return latched_error();
    }
    bool bus_owned = false;
    bool unit_owned = false;
    bool mutex_held = false;
    bool scheduler_locked = false;
    const osStatus_t acquired = osMutexAcquire(unit_mutex, mutex_ticks);
    tmc2209_status_t status = acquired == osOK ? TMC2209_OK : mutex_error(acquired);
    if (acquired != osOK) {
        goto failed;
    }
    mutex_held = true;
    status = TMC2209_BusInit(&bus, uart, baud);
    if (status != TMC2209_OK) {
        goto failed;
    }
    bus_owned = true;
    status = TMC2209_UnitInit(&unit, &bus, address);
    if (status != TMC2209_OK) {
        goto failed;
    }
    unit_owned = true;
    for (uint32_t attempt = 0U; attempt < PROBE_ATTEMPTS; ++attempt) {
        bool present = false;
        status = TMC2209_Available(&unit, &present);
        if (status == TMC2209_OK && present) {
            break;
        }
        if (status == TMC2209_OK) {
            status = TMC2209_ERR_NODEV;
        }
        if (attempt + 1U < PROBE_ATTEMPTS && osDelay(retry_ticks) != osOK) {
            status = TMC2209_ERR_OS;
            goto failed;
        }
    }
    if (status != TMC2209_OK) {
        goto failed;
    }
    if (osMutexRelease(unit_mutex) != osOK) {
        status = TMC2209_ERR_OS;
        goto failed;
    }
    mutex_held = false;

    /* Keep task handles valid until a partial creation can be rolled back. */
    const int32_t lock_state = osKernelLock();
    if (lock_state != 0) {
        status = lock_state < 0 ? TMC2209_ERR_OS : TMC2209_ERR_STATE;
        goto failed;
    }
    scheduler_locked = true;
    const osThreadAttr_t counter_attributes = {
        .name = "TmcCounter", .stack_size = 1024U, .priority = osPriorityNormal,
    };
    const osThreadAttr_t monitor_attributes = {
        .name = "TmcMonitor", .stack_size = 1024U, .priority = osPriorityNormal,
    };
    workers[0] = osThreadNew(CounterTask, NULL, &counter_attributes);
    if (workers[0] == NULL) {
        status = TMC2209_ERR_OS;
        goto failed;
    }
    workers[1] = osThreadNew(MonitorTask, NULL, &monitor_attributes);
    if (workers[1] == NULL) {
        status = TMC2209_ERR_OS;
        goto failed;
    }
    if (osKernelRestoreLock(0) != 0) {
        status = TMC2209_ERR_OS;
        goto failed;
    }
    expected = APP_STARTING;
    if (!atomic_compare_exchange_strong(&lifecycle, &expected, APP_READY)) {
        /* A released worker already faulted and parked; never publish readiness. */
        return latched_error();
    }
    return TMC2209_OK;

failed:
    fail_service(status);
    if (scheduler_locked) {
        for (size_t i = 0U; i < 2U; ++i) {
            if (workers[i] != NULL) {
                if (osThreadTerminate(workers[i]) == osOK) {
                    workers[i] = NULL;
                } else {
                    fail_cleanup(TMC2209_ERR_OS);
                }
            }
        }
        if (osKernelRestoreLock(0) != 0) {
            fail_cleanup(TMC2209_ERR_OS);
        }
    }
    cleanup_startup(bus_owned, unit_owned, mutex_held);
    return latched_error();
}
