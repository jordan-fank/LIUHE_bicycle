/*
 * lqr_balance.c
 *
 * LQR平衡控制应用层实现
 *
 * Created on: 2025
 * Author: suiyungui
 */

#include "lqr_balance.h"
#include "WP_Math.h"
#include <math.h>

// ==================== 全局变量 ====================
LQR_Controller_t lqr_ctrl;
float lqr_expect_phi = 0.0f;        // 期望侧倾角（度）

// 轮子参数（可运行时调整）
static float wheel_radius = LQR_WHEEL_RADIUS;
static float gear_ratio = LQR_GEAR_RATIO;

// ==================== 函数实现 ====================

/**
 * @brief  初始化LQR平衡控制
 */
void lqr_balance_init(void)
{
    // 初始化LQR控制器
    lqr_init(&lqr_ctrl);

    // 设置物理参数（后续根据实测调整）
    lqr_set_physical_params(&lqr_ctrl, 0.12f, 0.21f, 0.02f);

    // 设置限幅
    lqr_set_limits(&lqr_ctrl,
                   LQR_DELTA_MECHANICAL_MAX,    // 最大转向角
                   8.0f,                        // 最大转向角速度
                   0.3f);                       // 最小速度

    // 初始化期望角度
    lqr_expect_phi = 0.0f;

    // 默认使能
    lqr_enable(&lqr_ctrl, 1);
}

/**
 * @brief  从电机RPM估计车速
 * @return 速度（m/s），恒正
 */
float lqr_get_velocity(void)
{
    float rpm, omega, velocity;

    // 获取电机转速（RPM）
    rpm = (float)motor_get_speed();

    // 转换为线速度：v = rpm * 2*pi*r / 60 / gear_ratio
    omega = rpm * 2.0f * M_PI_F / 60.0f / gear_ratio;
    velocity = omega * wheel_radius;

    // 取绝对值（速度恒正）
    if (velocity < 0.0f) {
        velocity = -velocity;
    }

    return velocity;
}

/**
 * @brief  转向角转换为舵机PWM
 * @param  delta: 转向角（rad），正值=右转
 * @return 舵机PWM值
 */
uint32 lqr_delta_to_servo_pwm(float delta)
{
    float ratio, pwm;

    // 线性映射：delta -> PWM
    // delta = 0 -> mid
    // delta = +max -> r_max (右转)
    // delta = -max -> l_max (左转)
    ratio = delta / LQR_DELTA_MECHANICAL_MAX;

    // 限幅
    if (ratio > 1.0f) ratio = 1.0f;
    if (ratio < -1.0f) ratio = -1.0f;

    // 映射到PWM（左右分段，适应不对称舵机）
    if (ratio >= 0)
        pwm = (float)LQR_SERVO_MID + ratio * (float)(LQR_SERVO_RIGHT - LQR_SERVO_MID);
    else
        pwm = (float)LQR_SERVO_MID + ratio * (float)(LQR_SERVO_MID - LQR_SERVO_LEFT);

    // 边界保护
    if (pwm > (float)LQR_SERVO_RIGHT) pwm = (float)LQR_SERVO_RIGHT;
    if (pwm < (float)LQR_SERVO_LEFT) pwm = (float)LQR_SERVO_LEFT;

    return (uint32)pwm;
}

/**
 * @brief  LQR平衡控制主函数
 * @note   在5ms中断中调用
 */
void lqr_balance_control(void)
{
    float velocity;
    float phi, phi_dot;
    float delta_cmd;
    uint32 servo_pwm;
    int8_t ret;

    // 1. 获取当前速度
    velocity = lqr_get_velocity();

    // 2. 更新增益（根据速度插值）
    ret = lqr_update_gain(&lqr_ctrl, velocity);

    // 2.5 计算配套的目标转向角（物理套餐）
    // 公式：tan(phi) = v^2 / (g*L) * delta
    // 反推：delta = tan(phi) * g * L / v^2
    {
        float L = lqr_ctrl.params.L;
        float g = lqr_ctrl.params.g;
        float target_phi_rad = lqr_expect_phi * DEG_TO_RAD;

        if (velocity > 0.5f) {
            lqr_ctrl.target_delta = tanf(target_phi_rad) * g * L / (velocity * velocity);
            // 限幅保护
            if (lqr_ctrl.target_delta > LQR_DELTA_MECHANICAL_MAX) {
                lqr_ctrl.target_delta = LQR_DELTA_MECHANICAL_MAX;
            }
            if (lqr_ctrl.target_delta < -LQR_DELTA_MECHANICAL_MAX) {
                lqr_ctrl.target_delta = -LQR_DELTA_MECHANICAL_MAX;
            }
        } else {
            lqr_ctrl.target_delta = 0.0f;  // 低速时不允许压弯
        }
    }

    // 3. 获取IMU数据并转换单位
    // pitch_kalman: 度 -> rad（减去期望角度）
    // gyro_y_rate: 度/s -> rad/s
    // 注意：IMU定义左倾为正，LQR模型假设右倾为正，需取反
    phi = -(pitch_kalman - lqr_expect_phi) * DEG_TO_RAD;
    phi_dot = -gyro_y_rate * DEG_TO_RAD;

    // 4. 低速保护：速度过低时缓慢回中
    if (ret < 0) {
        // 低速时缓慢回中（衰减系数0.95）
        lqr_ctrl.delta_cmd *= 0.95f;
    } else {
        // 5. 执行LQR计算
        lqr_compute(&lqr_ctrl, phi, phi_dot);
    }

    // 6. 获取转向角命令
    delta_cmd = lqr_ctrl.delta_cmd;

    // 7. 转换为舵机PWM并输出
    servo_pwm = lqr_delta_to_servo_pwm(delta_cmd);
    servo_set(servo_pwm);
}

/**
 * @brief  设置期望侧倾角（用于转弯）
 * @param  phi_deg: 期望侧倾角（度）
 */
void lqr_set_expect_phi(float phi_deg)
{
    lqr_expect_phi = phi_deg;
}

/**
 * @brief  运行时调整物理参数
 */
void lqr_tune_params(float h, float L)
{
    lqr_set_physical_params(&lqr_ctrl, h, L, lqr_ctrl.params.b);
}

/**
 * @brief  运行时调整轮子参数
 */
void lqr_tune_wheel_params(float radius, float ratio)
{
    wheel_radius = radius;
    gear_ratio = ratio;
}

/**
 * @brief  获取调试信息
 */
void lqr_get_debug_info(float *phi, float *velocity, float *delta_deg)
{
    if (phi != NULL) {
        *phi = pitch_kalman;
    }
    if (velocity != NULL) {
        *velocity = lqr_ctrl.current_velocity;
    }
    if (delta_deg != NULL) {
        *delta_deg = lqr_ctrl.delta_cmd * RAD_TO_DEG;
    }
}
