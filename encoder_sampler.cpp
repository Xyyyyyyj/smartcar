#include "encoder_sampler.hpp"

static inline timespec timespec_add_ms(timespec t, int32 ms)
{
    t.tv_sec += ms / 1000;
    t.tv_nsec += (long)(ms % 1000) * 1000000L;
    if(t.tv_nsec >= 1000000000L)
    {
        t.tv_sec += 1;
        t.tv_nsec -= 1000000000L;
    }
    return t;
}

static inline uint32 timespec_diff_us(const timespec &a, const timespec &b)
{
    // return (a - b) in microseconds, saturate to uint32
    int64 sec = (int64)a.tv_sec - (int64)b.tv_sec;
    int64 nsec = (int64)a.tv_nsec - (int64)b.tv_nsec;
    int64 us = sec * 1000000LL + nsec / 1000LL;
    if(us < 0) us = 0;
    if(us > 0xFFFFFFFFLL) us = 0xFFFFFFFFLL;
    return (uint32)us;
}

std::atomic<int16> g_enc_delta_left{0};
std::atomic<int16> g_enc_delta_right{0};
std::atomic<uint32> g_enc_dt_us{0};

static zf_driver_encoder g_encoder_left(ZF_ENCODER_QUAD_1);
static zf_driver_encoder g_encoder_right(ZF_ENCODER_QUAD_2);

void *encoder_sampler::thread_entry(void *arg)
{
    auto *self = static_cast<encoder_sampler *>(arg);
    prctl(PR_SET_NAME, "enc_sampler");

    timespec next{};
    clock_gettime(CLOCK_MONOTONIC, &next);
    timespec last = next;

    // 对齐起始：先清零一次，避免第一次读到“历史累计值”
    g_encoder_left.clear_count();
    g_encoder_right.clear_count();

    while(!self->exit_flag.load(std::memory_order_relaxed))
    {
        timespec now{};
        clock_gettime(CLOCK_MONOTONIC, &now);
        const uint32 dt_us = timespec_diff_us(now, last);
        last = now;

        const int16 dl = g_encoder_left.get_count();
        const int16 dr = g_encoder_right.get_count();
        g_encoder_left.clear_count();
        g_encoder_right.clear_count();

        g_enc_delta_left.store(dl, std::memory_order_relaxed);
        g_enc_delta_right.store(dr, std::memory_order_relaxed);
        g_enc_dt_us.store(dt_us, std::memory_order_relaxed);

        next = timespec_add_ms(next, SPEED_SAMPLE_MS);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, nullptr);
    }
    return nullptr;
}

void encoder_sampler::start()
{
    pthread_create(&tid, nullptr, thread_entry, this);
}

void encoder_sampler::stop()
{
    exit_flag.store(true, std::memory_order_relaxed);
    if(tid)
    {
        pthread_join(tid, nullptr);
        tid = {};
    }
}

