/*
 * lqr_balance.h
 *
 * LQR平衡控制应用层
 * 集成IMU、舵机、电机，实现转向平衡控制
 *
 * Created on: 2025
 * Author: suiyungui
 */

#ifndef CODE_APP_LQR_BALANCE_H_
#define CODE_APP_LQR_BALANCE_H_

#include "lqr_driver.h"
#include "zf_common_headfile.h"

#define LQR_SERVO_MID           mid      // 舵机中位PWM
#define LQR_SERVO_LEFT          l_max    // 左极限
#define LQR_SERVO_RIGHT         r_max    // 右极限

// 角度转弧度
#define DEG_RAD(x)    ((x) * 3.14159f / 180.0f)

// 机械转向角范围（直接填角度，自动转弧度）
#define LQR_DELTA_MECHANICAL_MAX    DEG_RAD(23.0f)   // 最大机械转向角

// ==================== 轮子参数（速度估计用）====================
#define LQR_WHEEL_RADIUS        0.035f   // 轮子半径（m），需实测
#define LQR_GEAR_RATIO          1.0f    // 减速比



// ==================== 外部变量声明 ====================

extern LQR_Controller_t lqr_ctrl;       // LQR控制器实例
extern float lqr_expect_phi;            // 期望侧倾角（用于转弯控制，度）

// ==================== 函数声明 ====================

/**
 * @brief  初始化LQR平衡控制
 */
void lqr_balance_init(void);

/**
 * @brief  LQR平衡控制主函数
 * @note   在5ms中断中调用，替代原balance_control()
 */
void lqr_balance_control(void);

/**
 * @brief  设置期望侧倾角（用于转弯）
 * @param  phi_deg: 期望侧倾角（度）
 */
void lqr_set_expect_phi(float phi_deg);

/**
 * @brief  获取当前速度估计
 * @return 速度（m/s）
 */
float lqr_get_velocity(void);

/**
 * @brief  将转向角转换为舵机PWM
 * @param  delta: 转向角（rad），正值=右转
 * @return 舵机PWM值
 */
uint32 lqr_delta_to_servo_pwm(float delta);

/**
 * @brief  运行时调整物理参数
 * @param  h: 质心高度（m）
 * @param  L: 轴距（m）
 */
void lqr_tune_params(float h, float L);

/**
 * @brief  运行时调整轮子参数
 * @param  radius: 轮子半径（m）
 * @param  gear_ratio: 减速比
 */
void lqr_tune_wheel_params(float radius, float gear_ratio);

/**
 * @brief  获取调试信息
 * @param  phi: 输出当前侧倾角（度）
 * @param  velocity: 输出当前速度（m/s）
 * @param  delta_deg: 输出当前转向角命令（度）
 */
void lqr_get_debug_info(float *phi, float *velocity, float *delta_deg);

#endif /* CODE_APP_LQR_BALANCE_H_ */
