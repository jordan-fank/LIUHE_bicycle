/*
 * servo_app.c
 *
 *  Created on: 2025年11月11日
 *      Author: suiyungui
 */

#include "servo_app.h"

void servo_set(uint32_t duty)
{
    if(duty <= l_max)
        duty = l_max;
    if(duty >= r_max)
        duty = r_max;
    pwm_set_duty(ATOM1_CH1_P33_9, duty);
}

//-90-90旋转
void servo_test(void)
{
    static uint16_t servo_motor_duty = mid;
    static uint8_t servo_motor_dir = 0;
    if (servo_motor_dir)
    {
        servo_motor_duty += 10;
        if (servo_motor_duty >= r_max)
        {
            servo_motor_dir = 0x00;
        }
    }
    else
    {
        servo_motor_duty -= 10;
        if (servo_motor_duty <= l_max)
        {
            servo_motor_dir = 0x01;
        }
    }
    servo_set(servo_motor_duty);
}

// ============================================================================
// 新增：舵机初始化函数
// ============================================================================
void servo_init(void)
{
    // 舵机归中，等待到位
    servo_set(mid);
    

    #if (CURRENT_SERVO_TYPE == SERVO_TYPE_MG996R)
        // MG996R 需要较长预热时间
        system_delay_ms(2000);
    #elif (CURRENT_SERVO_TYPE == SERVO_TYPE_BDS300)
        // BDS300 响应更快
        system_delay_ms(500);
    #endif
}
