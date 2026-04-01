/*
 * 文件: motor_app.c
 * 功能: 电机应用层封装，提供占空比控制、RPM读取和车速换算接口
 * 作者: 闫锦
 * 日期: 2026-03-31
 */

#include "motor_app.h"

#pragma section all "cpu0_dsram"

volatile float motor_speed_rpm = 0.0f;              //电机转速
volatile float motor_speed_m_s = 0.0f;              //电机实际线速度
volatile float target_motor_speed_m_s = 0.0f;       //电机目标线速度


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





 /*============================统一获取电机速度接口====================*/

// 获取原始速度反馈（单位：RPM，保留驱动原始符号）
int16 motor_get_raw_rpm(void)
{
    return motor_value.receive_left_speed_data;
}

/**
 * @brief  获取按车体方向统一后的电机转速
 * @return 电机转速（RPM），约定“车辆前进为正”
 */
float motor_get_vehicle_rpm(void)
{
    return MOTOR_FORWARD_SIGN * (float)motor_get_raw_rpm();
}


/**
 * @brief  将电机转速 RPM 转换为真实车速 m/s
 * @param  rpm: 电机转速（转/分）
 * @return 线速度（m/s），保留方向
 */
float motor_rpm_to_m_s(float rpm)
{
    float motor_rps = rpm / 60.0f;
    float wheel_rps = motor_rps / MOTOR_GEAR_RATIO;
    float wheel_circumference = 3.14159f * MOTOR_WHEEL_DIAMETER_M;
    float speed_m_s = wheel_rps * wheel_circumference;

    return speed_m_s;
}

/**
 * @brief  获取当前线速度（m/s），保留方向
 * @return 线速度（m/s）
 */
float motor_get_vehicle_speed_m_s(void)
{
    return motor_rpm_to_m_s(motor_get_vehicle_rpm());
}

/**
 * @brief  获取当前速度大小（m/s），恒正
 * @return 速度大小（m/s）
 * @note   当前 LQR 只按速度大小做增益调度，不区分前进/后退方向。
 */
float motor_get_vehicle_speed_abs_m_s(void)
{
    float speed_m_s = motor_get_vehicle_speed_m_s();

    if (speed_m_s < 0.0f)
    {
        speed_m_s = -speed_m_s;
    }

    return speed_m_s;
}
