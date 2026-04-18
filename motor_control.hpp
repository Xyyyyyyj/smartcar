/**
 * 电机控制相关封装：双电机驱动 + 简单工具函数。
 */
#pragma once

#include "zf_common_headfile.hpp"

// 电机驱动（左右双电机固定方向）
// 驱动：2xPWM + 2xDIR；运行时 DIR 固定为 0，仅 PWM 占空比调速
#define MOTOR_PWM_1_PATH ZF_PWM_MOTOR_1
#define MOTOR_PWM_2_PATH ZF_PWM_MOTOR_2
#define MOTOR_DIR_1_PATH ZF_GPIO_MOTOR_1
#define MOTOR_DIR_2_PATH ZF_GPIO_MOTOR_2

static inline int32 clamp_int32(int32 v, int32 lo, int32 hi)
{
    if(v < lo) return lo;
    if(v > hi) return hi;
    return v;
}

static inline int16 clamp_int16(int32 v, int16 lo, int16 hi)
{
    if(v < (int32)lo) return lo;
    if(v > (int32)hi) return hi;
    return (int16)v;
}

struct two_motor_driver
{
    zf_driver_gpio dir_1;
    zf_driver_gpio dir_2;
    zf_driver_pwm  pwm_1;
    zf_driver_pwm  pwm_2;

    pwm_info pwm_1_info;
    pwm_info pwm_2_info;

    two_motor_driver();

    void init(void);
    void stop(void);

    uint32 duty_max_left(void) const;
    uint32 duty_max_right(void) const;

    // duty >= 0: DIR=0, PWM 输出 duty
    // duty <  0: DIR=1, PWM 输出 |duty|
    void set_speed_duty(int32 duty_left, int32 duty_right);

private:
    static inline uint16 duty_to_hw(uint32 duty, uint32 hw_max);
};
