#pragma once

#include "zf_common_headfile.hpp"
#include <atomic>
#include <time.h>

// 速度采样周期（单位 ms）
static constexpr int32 SPEED_SAMPLE_MS = 10;

extern std::atomic<int16> g_enc_delta_left;
extern std::atomic<int16> g_enc_delta_right;
extern std::atomic<uint32> g_enc_dt_us;

struct encoder_sampler
{
    std::atomic<bool> exit_flag{false};
    pthread_t tid{};

    static void *thread_entry(void *arg);

    void start();
    void stop();
};

