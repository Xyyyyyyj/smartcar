/**
 * two_motor_driver 实现：左右 PWM；DIR 固定低电平 0，仅用占空比调速。
 */
#include "motor_control.hpp"

two_motor_driver::two_motor_driver()
    : dir_1(MOTOR_DIR_1_PATH)
    , dir_2(MOTOR_DIR_2_PATH)
    , pwm_1(MOTOR_PWM_1_PATH)
    , pwm_2(MOTOR_PWM_2_PATH)
    , pwm_1_info{}
    , pwm_2_info{}
{
}

void two_motor_driver::init(void)
{
    dir_1.set_level(0);
    dir_2.set_level(0);
    pwm_1.get_dev_info(&pwm_1_info);
    pwm_2.get_dev_info(&pwm_2_info);
}

void two_motor_driver::stop(void)
{
    pwm_1.set_duty(0);
    pwm_2.set_duty(0);
}

uint32 two_motor_driver::duty_max_left(void) const
{
    return pwm_1_info.duty_max;
}

uint32 two_motor_driver::duty_max_right(void) const
{
    return pwm_2_info.duty_max;
}

uint16 two_motor_driver::duty_to_hw(uint32 duty, uint32 hw_max)
{
    if(hw_max == 0)
    {
        return 0;
    }
    if(duty > hw_max)
    {
        duty = hw_max;
    }
    if(duty > 65535u)
    {
        duty = 65535u;
    }
    return (uint16)duty;
}

void two_motor_driver::set_speed_duty(int32 duty_left, int32 duty_right)
{
    const uint32 max_l = pwm_1_info.duty_max;
    const uint32 max_r = pwm_2_info.duty_max;

    // 左电机
    if(duty_left >= 0)
    {
        dir_1.set_level(0);
        pwm_1.set_duty(duty_to_hw((uint32)duty_left, max_l));
    }
    else
    {
        // 反转：DIR=1，PWM 输出绝对值占空比
        const uint32 duty_abs = (uint32)(-(int64)duty_left);
        dir_1.set_level(1);
        pwm_1.set_duty(duty_to_hw(duty_abs, max_l));
    }

    // 右电机
    if(duty_right >= 0)
    {
        dir_2.set_level(0);
        pwm_2.set_duty(duty_to_hw((uint32)duty_right, max_r));
    }
    else
    {
        const uint32 duty_abs = (uint32)(-(int64)duty_right);
        dir_2.set_level(1);
        pwm_2.set_duty(duty_to_hw(duty_abs, max_r));
    }
}
