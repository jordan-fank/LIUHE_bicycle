/*
 * 文件: lqr_balance.h
 * 功能: 声明 LQR 平衡控制应用层接口和调试接口
 * 作者: 闫锦
 * 日期: 2026-03-31
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

// 机械转向角范围（直接填角度，自动转弧度）--需要实测
#define LQR_DELTA_MECHANICAL_MAX    DEG_RAD(23.0f)   // 最大机械转向角






// ==================== 外部变量声明 ====================
extern LQR_Controller_t lqr_ctrl;       // LQR控制器实例
extern float lqr_expect_phi;            // 期望侧倾角（用于转弯控制，度）

// ==================== 函数声明 ====================

/**
 * @brief  初始化LQR平衡控制
 */
void lqr_balance_init(void);

/* ==================== NEW: LQR使能接口 ====================
   用法和原 PID 方案保持一致：
   1. 初始化阶段先关闭
   2. 等 IMU 零位捕获完成后再打开
*/
void lqr_balance_set_enable(uint8_t enable);

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
 * @brief  获取当前用于 LQR 调度的速度大小
 * @return 速度大小（m/s）
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
 * @brief  获取调试信息
 * @param  phi: 输出当前侧倾角（度）
 * @param  velocity: 输出当前速度（m/s）
 * @param  delta_deg: 输出当前转向角命令（度）
 */
void lqr_get_debug_info(float *phi, float *velocity, float *delta_deg);

#endif /* CODE_APP_LQR_BALANCE_H_ */
