/*
 * motor_app.c
 *
 *  Created on: 2025年11月12日
 *      Author: suiyungui
 *  Modified: 改为无刷FOC驱动接口
 */

#include "motor_app.h"

#pragma section all "cpu0_dsram"

volatile float real_speed = 0.0f;
volatile float real_pulse = 0.0f;


#pragma section all restore

// 初始化
void motor_init(void)
{
    small_driver_uart_init();
}

// 设置占空比
void motor_set(int32_t duty)
{
    // 限幅 -10000 ~ 10000
    if(duty > 5000) duty = 5000;
    if(duty < -5000) duty = -5000;

    small_driver_set_duty((int16)duty, (int16)duty);
}

// 获取速度
int16 motor_get_speed(void)
{
    return motor_value.receive_left_speed_data;
}


/**
 * @brief  将编码器脉冲转换为真实车速(km/h)
 * @param  pulses_per_20ms: 编码器脉冲数/20ms
 * @return 车速(km/h)
 */
float motor_pulses_to_m_s(float pulses_per_20ms)
{
    // 1. 计算每秒脉冲数
    float pulses_per_sec = pulses_per_20ms * 50.0f;  // 20ms→1s, ×50
    
    // 2. 计算电机转速(RPS)
    float motor_rps = pulses_per_sec / ENCODER_PPR;
    
    // 3. 计算车轮转速(RPS)
    float wheel_rps = motor_rps / MOTOR_GEAR_RATIO;
    
    // 4. 计算车轮周长(m)
    float wheel_circumference = 3.14159f * (WHEEL_DIAMETER_MM / 1000.0f);
    
    // 5. 计算车速(m/s)
    float speed_m_s = wheel_rps * wheel_circumference;
    

    
    return speed_m_s;
}


void motor_calculate(void)
{
        
        real_speed = motor_pulses_to_m_s(real_pulse);    //获取真实的速度

        real_target_speed = motor_pulses_to_m_s(target_speed);



        
    /*=================速度范围测试===========================*/
    // static uint32_t debug_counter = 0;
    // static int16 max_speed = 0, min_speed = 0;
    //   // 记录极值
    // if(real_pulse > max_speed) max_speed = real_pulse;
    // if(real_pulse < min_speed) min_speed = real_pulse;
    
    // // 每100 次 (2秒)打印一次
    // if(++debug_counter >= 100) {
    //     printf("%d, %d,%d\r\n", 
    //            min_speed, max_speed, current);
    //     debug_counter = 0;
    // }
}
