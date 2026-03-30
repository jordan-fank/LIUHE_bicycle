/*
 * motor_pid.c
 *
 *  Created on: 2025年11月12日
 *      Author: suiyungui
 */
#include "motor_pid.h"
#include <math.h>

#pragma section all "cpu0_dsram"

volatile float g_motor_kp = 1.5f;          // 原 MOTOR_KP
volatile float g_motor_ki = 0.4f;          // 原 MOTOR_KI
volatile float g_motor_kd = 0.8f;          // 原 MOTOR_KD
volatile float g_motor_output_limit = 5000.0f;   // 原 MOTOR_LIMIT



PID_T motor_pid;                    // 速度PID结构体
float target_speed = 600.0f;          // 目标速度（脉冲/20ms）
float real_target_speed = 0.0f;

SpeedAssist_t speed_assist;         // Speed assist controller



#pragma section all restore



/*********************************************************************************************************************
 * 函数实现
 ********************************************************************************************************************/

/**
 * @brief       初始化电机速度PID
 * @details     配置PID参数并初始化结构体
 * @return      void
 */
void motor_pid_init(void)
{
    // 初始化增量式PID
    // 参数: PID指针, kp, ki, kd, 目标值(初始0), 输出限幅
    pid_init(&motor_pid, g_motor_kp, g_motor_ki, g_motor_kd, 0.0f, g_motor_output_limit);

    // // 初始化目标速度为0
    // target_speed = 600.0f;
}

/**
 * @brief       电机速度闭环控制函数
 * @details     核心控制逻辑，执行以下步骤:
 *              1. 读取当前速度（编码器值）
 *              2. 设置目标速度到PID
 *              3. 计算增量式PID输出
 *              4. 输出到电机PWM
 *
 *
 * @return      void
 */
void motor_control(void)
{
    // 1. 获取当前速度（编码器值，单位: 脉冲/20ms）
    float current_speed = -(float)motor_get_speed();

    real_pulse = current_speed;                         //获取真实的脉冲数

    // 2. 设置目标速度
    pid_set_target(&motor_pid, target_speed);

    // 3. 计算增量式PID输出
    float pid_output = pid_calculate_incremental(&motor_pid, current_speed);

//    JustFloat_Test_three(target_speed,current_speed,pid_output);
    // 4. 输出到电机（motor_set会自动限幅到±4000）
    motor_set((int32_t)pid_output);




}

/**
 * @brief       设置目标速度
 * @param       speed   目标速度（单位: 编码器脉冲/20ms）
 *                      正值 = 正转
 *                      负值 = 反转
 *                      0 = 停止
 * @return      void
 */
void motor_set_speed(float speed)
{
    target_speed = speed;
}

/**
 * @brief       运行时动态调整PID参数
 * @param       kp      比例系数（建议范围: 0.5 ~ 5.0）
 * @param       ki      积分系数（建议范围: 0.0 ~ 0.5）
 * @param       kd      微分系数（建议范围: 0.0 ~ 2.0）
 * @return      void
 * @note        可通过无线串口实时调参
 */
void motor_set_params(float kp, float ki, float kd)
{
    pid_set_params(&motor_pid, kp, ki, kd);
}

/**
 * @brief       重置PID控制器状态
 * @details     清除历史误差、积分项等内部状态
 *              建议在启动、停止或切换控制模式时调用
 * @return      void
 */
void motor_pid_reset(void)
{
    pid_reset(&motor_pid);
}
