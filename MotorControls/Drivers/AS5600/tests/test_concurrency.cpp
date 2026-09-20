extern "C"
{
#include "as5600.h"
}

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <thread>
#include <vector>

namespace
{
using Clock = std::chrono::steady_clock;
using Milliseconds = std::chrono::milliseconds;

std::atomic<std::uint64_t> assertions{0};
std::atomic<std::uint64_t> operations{0};
std::atomic<std::uint64_t> failures{0};
std::mutex reporting_mutex;

void check(bool condition, const char *expression, unsigned line)
{
    assertions.fetch_add(1, std::memory_order_relaxed);
    if (!condition)
    {
        const auto failure = failures.fetch_add(1, std::memory_order_relaxed);
        if (failure < 30)
        {
            std::lock_guard<std::mutex> guard(reporting_mutex);
            std::fprintf(stderr, "concurrency line=%u: %s\n", line, expression);
        }
    }
}

#define CHECK(expression) check((expression), #expression, __LINE__)

enum class Operation { sample, configuration };
enum class Fault
{
    none, context, lock, first_read, third_read, write, delay, verify,
    final_status, unlock, read_and_unlock, late_read, late_unlock, budget_reads
};

struct Gate
{
    std::mutex mutex;
    std::condition_variable changed;
    unsigned ready = 0;
    bool released = false;
};

struct Bus
{
    std::timed_mutex mutex;
    /* This separate mutex makes even a broken driver's audit race-free. */
    std::mutex audit;
    const Clock::time_point epoch = Clock::now();
    std::thread::id owner;
    std::uint64_t owner_operation = 0;
    std::uint16_t configuration = 0xc000;
    std::atomic<std::uint64_t> next_operation{1};
    std::atomic<std::uint64_t> acquired{0};
    std::atomic<std::uint64_t> released{0};
    std::atomic<std::uint64_t> contended{0};
    std::atomic<unsigned> active_callbacks{0};
};

struct Call
{
    Bus *bus = nullptr;
    Operation operation = Operation::sample;
    Fault fault = Fault::none;
    std::uint64_t id = 0;
    std::uint32_t budget = 0;
    std::uint32_t last_budget = 0;
    std::uint32_t first_transfer_budget = 0;
    std::uint32_t start_tick = 0;
    std::uint32_t last_tick = 0;
    unsigned contexts = 0;
    unsigned clocks = 0;
    unsigned locks = 0;
    unsigned unlocks = 0;
    unsigned phase = 0;
    bool context_ok = false;
    bool acquired = false;
    bool held = false;
    bool unchanged = false;
    bool fault_triggered = false;
    bool release_fault_triggered = false;
    bool contended_timeout = false;
    bool settled = false;
    Gate *lock_entered = nullptr;
    Clock::time_point write_time{};
    as5600_config_t configuration{};
    std::uint16_t configuration_word = 0;
    as5600_sample_t expected_sample{};
    as5600_result_t result = AS5600_ERROR_IO;
};

thread_local Call *current_call = nullptr;

Call *get_call(void *context)
{
    CHECK(current_call != nullptr);
    if (current_call == nullptr)
    {
        return nullptr;
    }
    CHECK(context == current_call->bus);
    return current_call;
}

void check_owner(const Call &call)
{
    CHECK(call.context_ok);
    CHECK(call.clocks > 0);
    CHECK(call.held);
    CHECK(call.bus->owner == std::this_thread::get_id());
    CHECK(call.bus->owner_operation == call.id);
}

void check_budget(Call &call, std::uint32_t timeout, bool transfer)
{
    CHECK(timeout > 0);
    CHECK(timeout <= UINT32_C(2147483647));
    CHECK(timeout <= call.budget);
    CHECK(timeout <= call.last_budget);
    call.last_budget = timeout;
    if (transfer && (call.first_transfer_budget == 0))
    {
        call.first_transfer_budget = timeout;
    }
}

class CallbackGuard
{
public:
    explicit CallbackGuard(Bus &bus) : bus_(bus)
    {
        CHECK(bus_.active_callbacks.fetch_add(1, std::memory_order_acq_rel) == 0);
    }
    ~CallbackGuard()
    {
        CHECK(bus_.active_callbacks.fetch_sub(1, std::memory_order_acq_rel) == 1);
    }
    CallbackGuard(const CallbackGuard &) = delete;
    CallbackGuard &operator=(const CallbackGuard &) = delete;

private:
    Bus &bus_;
};

as5600_result_t check_context(void *context)
{
    Call *call = get_call(context);
    if (call == nullptr)
    {
        return AS5600_ERROR_CONTEXT;
    }
    CHECK(call->contexts == 0);
    CHECK(call->clocks == 0);
    CHECK(call->locks == 0);
    CHECK(call->phase == 0);
    ++call->contexts;
    if (call->fault == Fault::context)
    {
        call->fault_triggered = true;
        return AS5600_ERROR_CONTEXT;
    }
    call->context_ok = true;
    return AS5600_OK;
}

std::uint32_t now_ms(void *context)
{
    Call *call = get_call(context);
    if (call == nullptr)
    {
        return 0;
    }
    CHECK(call->context_ok);
    const auto elapsed =
        std::chrono::duration_cast<Milliseconds>(Clock::now() - call->bus->epoch);
    const auto tick = static_cast<std::uint32_t>(elapsed.count());
    if (call->clocks == 0)
    {
        call->start_tick = tick;
    }
    ++call->clocks;
    call->last_tick = tick;
    return tick;
}

as5600_result_t lock_bus(void *context, std::uint32_t timeout)
{
    Call *call = get_call(context);
    if (call == nullptr)
    {
        return AS5600_ERROR_LOCK;
    }
    CHECK(call->context_ok);
    CHECK(call->clocks > 0);
    CHECK(call->locks == 0);
    CHECK(!call->held);
    ++call->locks;
    check_budget(*call, timeout, false);
    if (call->held)
    {
        return AS5600_ERROR_LOCK;
    }
    if (call->fault == Fault::lock)
    {
        call->fault_triggered = true;
        return AS5600_ERROR_BUSY;
    }
    if (call->lock_entered != nullptr)
    {
        std::lock_guard<std::mutex> guard(call->lock_entered->mutex);
        call->lock_entered->ready = 1;
        call->lock_entered->changed.notify_one();
    }
    bool acquired = call->bus->mutex.try_lock();
    if (!acquired)
    {
        call->bus->contended.fetch_add(1, std::memory_order_relaxed);
        acquired = call->bus->mutex.try_lock_for(Milliseconds(timeout));
    }
    if (!acquired)
    {
        return AS5600_ERROR_TIMEOUT;
    }
    std::lock_guard<std::mutex> guard(call->bus->audit);
    CHECK(call->bus->owner == std::thread::id{});
    CHECK(call->bus->owner_operation == 0);
    call->bus->owner = std::this_thread::get_id();
    call->bus->owner_operation = call->id;
    call->acquired = true;
    call->held = true;
    call->bus->acquired.fetch_add(1, std::memory_order_relaxed);
    return AS5600_OK;
}

as5600_result_t unlock_bus(void *context)
{
    Call *call = get_call(context);
    if (call == nullptr)
    {
        return AS5600_ERROR_UNLOCK;
    }
    if (call->fault == Fault::late_unlock)
    {
        call->fault_triggered = true;
        std::this_thread::sleep_for(Milliseconds(call->budget + 10));
    }
    {
        std::lock_guard<std::mutex> guard(call->bus->audit);
        check_owner(*call);
        CHECK(call->unlocks == 0);
        CHECK(call->bus->active_callbacks.load(std::memory_order_acquire) == 0);
        if (!call->held || (call->bus->owner != std::this_thread::get_id()) ||
            (call->bus->owner_operation != call->id))
        {
            return AS5600_ERROR_UNLOCK;
        }
        ++call->unlocks;
        call->bus->owner = std::thread::id{};
        call->bus->owner_operation = 0;
        call->held = false;
        call->bus->released.fetch_add(1, std::memory_order_relaxed);
    }
    call->bus->mutex.unlock();
    if ((call->fault == Fault::unlock) ||
        (call->fault == Fault::read_and_unlock))
    {
        call->release_fault_triggered = true;
        call->fault_triggered = true;
        return AS5600_ERROR_IO;
    }
    return AS5600_OK;
}

void put_word(std::uint8_t *data, std::uint16_t word)
{
    data[0] = static_cast<std::uint8_t>(word / 256);
    data[1] = static_cast<std::uint8_t>(word % 256);
}

as5600_result_t read_bus(
    void *context, std::uint8_t address, std::uint8_t reg,
    std::uint8_t *data, std::uint16_t length, std::uint32_t timeout)
{
    Call *call = get_call(context);
    if (call == nullptr)
    {
        return AS5600_ERROR_IO;
    }
    CallbackGuard callback(*call->bus);
    const unsigned phase = call->phase++;
    {
        std::lock_guard<std::mutex> guard(call->bus->audit);
        check_owner(*call);
        check_budget(*call, timeout, true);
        CHECK(address == 0x36);
        CHECK(data != nullptr);
        if ((data == nullptr) || (length == 0) || (length > 2))
        {
            CHECK(false);
            return AS5600_ERROR_IO;
        }
        if (call->operation == Operation::sample)
        {
            static constexpr std::array<std::uint8_t, 6> registers = {
                0x0b, 0x0c, 0x0e, 0x1a, 0x1b, 0x0b
            };
            static constexpr std::array<std::uint16_t, 6> lengths = {
                1, 2, 2, 1, 2, 1
            };
            CHECK(phase < registers.size());
            if (phase >= registers.size())
            {
                return AS5600_ERROR_IO;
            }
            CHECK(reg == registers[phase]);
            CHECK(length == lengths[phase]);
            if ((reg != registers[phase]) || (length != lengths[phase]))
            {
                return AS5600_ERROR_IO;
            }
            switch (phase)
            {
                case 0: data[0] = 0x24; break;
                case 1:
                    put_word(data, static_cast<std::uint16_t>(
                        call->expected_sample.raw_angle | 0xf000));
                    break;
                case 2:
                    put_word(data, static_cast<std::uint16_t>(
                        call->expected_sample.angle | 0xa000));
                    break;
                case 3: data[0] = call->expected_sample.diagnostics.agc; break;
                case 4:
                    put_word(data, static_cast<std::uint16_t>(
                        call->expected_sample.diagnostics.magnitude | 0xb000));
                    break;
                default: data[0] = 0xa1; break;
            }
            if ((call->fault == Fault::final_status) && (phase == 5))
            {
                call->fault_triggered = true;
                data[0] = 0x81;
            }
        }
        else
        {
            CHECK(reg == 0x07);
            CHECK(length == 2);
            CHECK((phase == 0) || (phase == 3));
            if ((reg != 0x07) || (length != 2))
            {
                return AS5600_ERROR_IO;
            }
            auto word = call->bus->configuration;
            if (phase == 0)
            {
                call->unchanged = word == call->configuration_word;
            }
            else
            {
                CHECK(call->settled);
                CHECK(Clock::now() - call->write_time >= Milliseconds(1));
                if (call->fault == Fault::verify)
                {
                    call->fault_triggered = true;
                    word ^= 0x8000;
                }
            }
            put_word(data, word);
        }
        if (((call->fault == Fault::first_read) && (phase == 0)) ||
            ((call->fault == Fault::read_and_unlock) && (phase == 0)))
        {
            call->fault_triggered = true;
            data[0] = 0xee;
            return AS5600_ERROR_IO;
        }
        if ((call->fault == Fault::third_read) && (phase == 2))
        {
            call->fault_triggered = true;
            data[0] = 0xee;
            return AS5600_ERROR_NACK;
        }
    }
    if (call->fault == Fault::late_read)
    {
        call->fault_triggered = true;
        std::this_thread::sleep_for(Milliseconds(timeout + 10));
    }
    else if (call->fault == Fault::budget_reads)
    {
        if (timeout <= 10)
        {
            call->fault_triggered = true;
            std::this_thread::sleep_for(Milliseconds(timeout));
            return AS5600_ERROR_TIMEOUT;
        }
        std::this_thread::sleep_for(Milliseconds(10));
    }
    std::this_thread::yield();
    return AS5600_OK;
}

as5600_result_t write_bus(
    void *context, std::uint8_t address, std::uint8_t reg,
    const std::uint8_t *data, std::uint16_t length, std::uint32_t timeout)
{
    Call *call = get_call(context);
    if (call == nullptr)
    {
        return AS5600_ERROR_IO;
    }
    CallbackGuard callback(*call->bus);
    {
        std::lock_guard<std::mutex> guard(call->bus->audit);
        check_owner(*call);
        check_budget(*call, timeout, true);
        CHECK(call->operation == Operation::configuration);
        CHECK(call->phase == 1);
        ++call->phase;
        CHECK(!call->unchanged);
        CHECK(address == 0x36);
        CHECK(reg == 0x07);
        CHECK(length == 2);
        CHECK(data != nullptr);
        if ((data == nullptr) || (length != 2) || (reg != 0x07))
        {
            return AS5600_ERROR_IO;
        }
        const auto word = static_cast<std::uint16_t>(
            static_cast<unsigned>(data[0]) * 256U + data[1]);
        CHECK(word == call->configuration_word);
        CHECK((word & 0xc000) == 0xc000);
        call->write_time = Clock::now();
        if (call->fault == Fault::write)
        {
            call->bus->configuration = static_cast<std::uint16_t>(
                (word & 0xff00) | (call->bus->configuration & 0x00ff));
            call->fault_triggered = true;
            return AS5600_ERROR_IO;
        }
        call->bus->configuration = word;
    }
    std::this_thread::yield();
    return AS5600_OK;
}

as5600_result_t delay_bus(
    void *context, std::uint32_t minimum, std::uint32_t timeout)
{
    Call *call = get_call(context);
    if (call == nullptr)
    {
        return AS5600_ERROR_IO;
    }
    CallbackGuard callback(*call->bus);
    {
        std::lock_guard<std::mutex> guard(call->bus->audit);
        check_owner(*call);
        check_budget(*call, timeout, true);
        CHECK(call->operation == Operation::configuration);
        CHECK(call->phase == 2);
        ++call->phase;
        CHECK(minimum >= 1);
        CHECK(minimum < timeout);
    }
    if (call->fault == Fault::delay)
    {
        call->fault_triggered = true;
        return AS5600_ERROR_IO;
    }
    const auto started = Clock::now();
    std::this_thread::sleep_for(Milliseconds(minimum));
    CHECK(Clock::now() - started >= Milliseconds(minimum));
    call->settled = true;
    return AS5600_OK;
}

Call make_call(Bus &bus, Operation operation, Fault fault,
               std::uint32_t budget = 5000)
{
    Call call;
    call.bus = &bus;
    call.operation = operation;
    call.fault = fault;
    call.id = bus.next_operation.fetch_add(1, std::memory_order_relaxed);
    call.budget = budget;
    call.last_budget = budget;
    const auto code = static_cast<unsigned>(call.id % 512);
    static constexpr std::array<as5600_power_t, 4> powers = {
        AS5600_POWER_NORMAL, AS5600_POWER_LOW_1,
        AS5600_POWER_LOW_2, AS5600_POWER_LOW_3
    };
    static constexpr std::array<as5600_hysteresis_t, 4> hysteresis = {
        AS5600_HYSTERESIS_OFF, AS5600_HYSTERESIS_1_LSB,
        AS5600_HYSTERESIS_2_LSB, AS5600_HYSTERESIS_3_LSB
    };
    static constexpr std::array<as5600_pwm_t, 4> pwm = {
        AS5600_PWM_115_HZ, AS5600_PWM_230_HZ,
        AS5600_PWM_460_HZ, AS5600_PWM_920_HZ
    };
    static constexpr std::array<as5600_slow_filter_t, 4> slow = {
        AS5600_SLOW_FILTER_16X, AS5600_SLOW_FILTER_8X,
        AS5600_SLOW_FILTER_4X, AS5600_SLOW_FILTER_2X
    };
    call.configuration.power = powers[code % 4];
    call.configuration.hysteresis = hysteresis[(code / 4) % 4];
    call.configuration.output = AS5600_OUTPUT_PWM;
    call.configuration.pwm_frequency = pwm[(code / 16) % 4];
    call.configuration.slow_filter = slow[(code / 64) % 4];
    call.configuration.fast_filter = AS5600_FAST_FILTER_10_LSB;
    call.configuration.watchdog = code >= 256;
    call.configuration_word = static_cast<std::uint16_t>(
        0xc000U + 0x1c00U + 0x20U + (code % 16) +
        (((code / 16) % 4) * 64) + (((code / 64) % 4) * 256) +
        ((code >= 256) ? 0x2000U : 0U));
    call.expected_sample.raw_angle = static_cast<std::uint16_t>(call.id % 4096);
    call.expected_sample.angle = static_cast<std::uint16_t>(
        (call.id * 7 + 19) % 4096);
    call.expected_sample.diagnostics.status = 0xa1;
    call.expected_sample.diagnostics.agc = static_cast<std::uint8_t>(
        (call.id * 11) % 256);
    call.expected_sample.diagnostics.magnitude = static_cast<std::uint16_t>(
        (call.id * 13 + 600) % 4096);
    return call;
}

as5600_result_t expected_result(const Call &call)
{
    if (call.contended_timeout)
    {
        return AS5600_ERROR_TIMEOUT;
    }
    switch (call.fault)
    {
        case Fault::none: return AS5600_OK;
        case Fault::context: return AS5600_ERROR_CONTEXT;
        case Fault::lock: return AS5600_ERROR_BUSY;
        case Fault::third_read: return AS5600_ERROR_NACK;
        case Fault::verify: return AS5600_ERROR_VERIFY;
        case Fault::final_status: return AS5600_ERROR_NO_MAGNET;
        case Fault::unlock:
        case Fault::read_and_unlock: return AS5600_ERROR_UNLOCK;
        case Fault::late_read:
        case Fault::late_unlock:
        case Fault::budget_reads: return AS5600_ERROR_TIMEOUT;
        default: return AS5600_ERROR_IO;
    }
}

void run_call(const as5600_t &device, Call &call)
{
    CHECK(current_call == nullptr);
    current_call = &call;
    as5600_sample_t sample;
    std::memset(&sample, 0xa5, sizeof(sample));
    std::array<unsigned char, sizeof(sample)> before{};
    std::memcpy(before.data(), &sample, sizeof(sample));
    std::array<unsigned char, sizeof(call.configuration)> configuration_before{};
    std::memcpy(configuration_before.data(), &call.configuration,
                sizeof(call.configuration));
    operations.fetch_add(1, std::memory_order_relaxed);
    call.result = (call.operation == Operation::sample) ?
        as5600_read_sample(&device, &sample, call.budget) :
        as5600_write_config(&device, &call.configuration, call.budget);
    const auto expected = expected_result(call);
    CHECK(call.result == expected);
    if (call.result != expected)
    {
        std::lock_guard<std::mutex> guard(reporting_mutex);
        std::fprintf(stderr, "operation=%llu fault=%d result=%d expected=%d\n",
                     static_cast<unsigned long long>(call.id),
                     static_cast<int>(call.fault), static_cast<int>(call.result),
                     static_cast<int>(expected));
    }
    CHECK(std::memcmp(configuration_before.data(), &call.configuration,
                      sizeof(call.configuration)) == 0);
    CHECK(call.contexts == 1);
    CHECK(!call.held);
    CHECK(call.unlocks == (call.acquired ? 1U : 0U));
    if (call.fault == Fault::context)
    {
        CHECK(call.clocks == 0);
        CHECK(call.locks == 0);
        CHECK(call.phase == 0);
    }
    else
    {
        CHECK(call.locks == 1);
    }
    if ((call.fault == Fault::lock) || call.contended_timeout)
    {
        CHECK(!call.acquired);
        CHECK(call.phase == 0);
        CHECK(call.unlocks == 0);
    }
    if ((call.fault != Fault::none) && (call.fault != Fault::budget_reads))
    {
        CHECK(call.fault_triggered);
    }
    if (call.fault == Fault::read_and_unlock)
    {
        CHECK(call.release_fault_triggered);
        CHECK(call.phase == 1);
    }
    if (call.result != AS5600_OK)
    {
        CHECK(std::memcmp(before.data(), &sample, sizeof(sample)) == 0);
    }
    else
    {
        CHECK(call.acquired);
        CHECK(static_cast<std::uint32_t>(call.last_tick - call.start_tick) <
              call.budget);
        if (call.operation == Operation::sample)
        {
            CHECK(call.phase == 6);
            CHECK(sample.raw_angle == call.expected_sample.raw_angle);
            CHECK(sample.angle == call.expected_sample.angle);
            CHECK(sample.diagnostics.status == call.expected_sample.diagnostics.status);
            CHECK(sample.diagnostics.agc == call.expected_sample.diagnostics.agc);
            CHECK(sample.diagnostics.magnitude ==
                  call.expected_sample.diagnostics.magnitude);
        }
        else
        {
            CHECK(call.phase == (call.unchanged ? 1U : 4U));
        }
    }
    current_call = nullptr;
}

void check_unlocked(Bus &bus)
{
    const bool acquired = bus.mutex.try_lock_for(Milliseconds(200));
    CHECK(acquired);
    if (acquired)
    {
        std::lock_guard<std::mutex> guard(bus.audit);
        CHECK(bus.owner == std::thread::id{});
        CHECK(bus.owner_operation == 0);
        CHECK(bus.active_callbacks.load(std::memory_order_acquire) == 0);
        bus.mutex.unlock();
    }
    CHECK(bus.acquired.load(std::memory_order_relaxed) ==
          bus.released.load(std::memory_order_relaxed));
}

void run_workers(Bus &bus, const std::array<as5600_t, 4> &devices)
{
    constexpr unsigned worker_count = 4;
    constexpr unsigned iterations = 64;
    static constexpr std::array<Fault, 16> faults = {
        Fault::none, Fault::none, Fault::third_read, Fault::write,
        Fault::final_status, Fault::verify, Fault::lock, Fault::delay,
        Fault::none, Fault::first_read, Fault::context, Fault::none,
        Fault::none, Fault::unlock, Fault::first_read, Fault::none
    };
    Gate start;
    std::vector<std::thread> workers;
    for (unsigned worker = 0; worker < worker_count; ++worker)
    {
        workers.emplace_back([&, worker]
        {
            {
                std::unique_lock<std::mutex> guard(start.mutex);
                ++start.ready;
                start.changed.notify_all();
                start.changed.wait(guard, [&] { return start.released; });
            }
            for (unsigned iteration = 0; iteration < iterations; ++iteration)
            {
                if (failures.load(std::memory_order_relaxed) != 0)
                {
                    break;
                }
                const auto operation = ((iteration % 2) == 0) ?
                    Operation::sample : Operation::configuration;
                const auto fault = faults[iteration % faults.size()];
                auto call = make_call(bus, operation, fault);
                /* Every thread alternates between all four shared-bus handles. */
                const auto &device = devices[(worker + iteration) % devices.size()];
                run_call(device, call);
                if (fault != Fault::none)
                {
                    auto recovery = make_call(bus, Operation::sample, Fault::none);
                    run_call(device, recovery);
                }
            }
        });
    }
    {
        std::unique_lock<std::mutex> guard(start.mutex);
        start.changed.wait(guard, [&] { return start.ready == worker_count; });
        start.released = true;
        start.changed.notify_all();
    }
    for (auto &worker : workers)
    {
        worker.join();
    }
    check_unlocked(bus);
}

void run_deadline_cases(Bus &bus, const as5600_t &device)
{
    {
        /* Holding until join guarantees genuine lock timeout, not a race. */
        std::unique_lock<std::timed_mutex> holder(bus.mutex, std::defer_lock);
        CHECK(holder.try_lock_for(Milliseconds(200)));
        if (!holder.owns_lock())
        {
            return;
        }
        auto call = make_call(bus, Operation::sample, Fault::none, 40);
        call.contended_timeout = true;
        std::thread waiter([&] { run_call(device, call); });
        waiter.join();
        CHECK(call.first_transfer_budget == 0);
    }
    check_unlocked(bus);
    {
        std::unique_lock<std::timed_mutex> holder(bus.mutex, std::defer_lock);
        CHECK(holder.try_lock_for(Milliseconds(200)));
        if (!holder.owns_lock())
        {
            return;
        }
        Gate entered;
        auto call = make_call(bus, Operation::sample, Fault::none, 2000);
        call.lock_entered = &entered;
        std::thread waiter([&] { run_call(device, call); });
        {
            std::unique_lock<std::mutex> guard(entered.mutex);
            CHECK(entered.changed.wait_for(
                guard, std::chrono::seconds(5), [&] { return entered.ready != 0; }));
        }
        std::this_thread::sleep_for(Milliseconds(40));
        holder.unlock();
        waiter.join();
        CHECK(call.first_transfer_budget > 0);
        CHECK(call.first_transfer_budget <= call.budget - 20);
    }
    for (const auto fault : {Fault::late_read, Fault::late_unlock,
                             Fault::budget_reads, Fault::read_and_unlock})
    {
        auto call = make_call(bus, Operation::sample, fault,
                              (fault == Fault::budget_reads) ? 35U : 60U);
        run_call(device, call);
        if (fault == Fault::late_read)
        {
            CHECK(call.phase == 1);
        }
        if (fault == Fault::late_unlock)
        {
            CHECK(call.phase == 6);
        }
        if (fault == Fault::budget_reads)
        {
            CHECK(call.phase < 6);
            CHECK(call.last_budget <= call.first_transfer_budget);
        }
        check_unlocked(bus);
        auto recovery = make_call(bus, Operation::sample, Fault::none);
        run_call(device, recovery);
    }
    CHECK(bus.contended.load(std::memory_order_relaxed) > 0);
    check_unlocked(bus);
}
} // namespace

int main()
{
    const auto started = Clock::now();
    Bus bus;
    as5600_bus_t callbacks{};
    callbacks.context = &bus;
    callbacks.read = read_bus;
    callbacks.write = write_bus;
    callbacks.now_ms = now_ms;
    callbacks.check_context = check_context;
    callbacks.delay_ms = delay_bus;
    callbacks.lock = lock_bus;
    callbacks.unlock = unlock_bus;
    std::array<as5600_t, 4> devices{};
    for (auto &device : devices)
    {
        CHECK(as5600_init(&device, &callbacks) == AS5600_OK);
        CHECK(device.bus.context == &bus);
        CHECK(device.bus.lock == lock_bus);
        CHECK(device.bus.unlock == unlock_bus);
    }
    CHECK(operations.load(std::memory_order_relaxed) == 0);
    run_workers(bus, devices);
    run_deadline_cases(bus, devices[0]);
    const auto elapsed =
        std::chrono::duration_cast<Milliseconds>(Clock::now() - started).count();
    std::printf("concurrency: 4 workers, 4 handles, %llu operations, "
                "%llu assertions, %llu failures, %llu acquisitions/releases, "
                "%llu contentions, %lld ms\n",
                static_cast<unsigned long long>(operations.load()),
                static_cast<unsigned long long>(assertions.load()),
                static_cast<unsigned long long>(failures.load()),
                static_cast<unsigned long long>(bus.acquired.load()),
                static_cast<unsigned long long>(bus.contended.load()),
                static_cast<long long>(elapsed));
    return (failures.load() == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}
