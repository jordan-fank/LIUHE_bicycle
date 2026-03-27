/*
 * servo_app.c
 *
 *  Created on: 2025年11月11日
 *      Author: suiyungui
 */

#include "servo_app.h"

/* ==================== NEW: 舵机参数改为全局变量 ====================
   原来的左右极限 / 中值宏定义改成全局变量，便于在 IPS 页面显示和调节。
   默认值按当前舵机类型给出。
*/
#if (CURRENT_SERVO_TYPE == SERVO_TYPE_MG996R)
volatile uint32_t g_servo_left_limit = 250U;     // 原 l_max，对应 500us
volatile uint32_t g_servo_mid_duty = 750U;       // 原 mid，对应 1500us
volatile uint32_t g_servo_right_limit = 1250U;   // 原 r_max，对应 2500us
#elif (CURRENT_SERVO_TYPE == SERVO_TYPE_BDS300)
volatile uint32_t g_servo_left_limit = 250U;     // 按当前工程默认值初始化，可后续在 IPS 中调节
volatile uint32_t g_servo_mid_duty = 750U;
volatile uint32_t g_servo_right_limit = 1250U;
#else
volatile uint32_t g_servo_left_limit = 250U;
volatile uint32_t g_servo_mid_duty = 750U;
volatile uint32_t g_servo_right_limit = 1250U;
#endif




void servo_set(uint32_t duty)
{
    if(duty <= g_servo_left_limit)
        duty = g_servo_left_limit;
    if(duty >= g_servo_right_limit)
        duty = g_servo_right_limit;
    pwm_set_duty(ATOM1_CH1_P33_9, duty);
}

//-90-90旋转
void servo_test(void)
{
    static uint16_t servo_motor_duty = 0;
    static uint8_t servo_motor_dir = 0;
    if (0U == servo_motor_duty)
    {
        servo_motor_duty = (uint16_t)g_servo_mid_duty;
    }
    if (servo_motor_dir)
    {
        servo_motor_duty += 10;
        if (servo_motor_duty >= g_servo_right_limit)
        {
            servo_motor_dir = 0x00;
        }
    }
    else
    {
        servo_motor_duty -= 10;
        if (servo_motor_duty <= g_servo_left_limit)
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
    servo_set(g_servo_mid_duty);
    

    #if (CURRENT_SERVO_TYPE == SERVO_TYPE_MG996R)
        // MG996R 需要较长预热时间
        system_delay_ms(2000);
    #elif (CURRENT_SERVO_TYPE == SERVO_TYPE_BDS300)
        // BDS300 响应更快
        system_delay_ms(500);
    #endif
}
