/*
 * motor_app.h
 *
 *  Created on: 2025年11月12日
 *      Author: suiyungui
 *  Modified: 改为无刷FOC驱动接口
 */

#ifndef CODE_APP_MOTOR_APP_H_
#define CODE_APP_MOTOR_APP_H_

#include "zf_common_headfile.h"
// small_driver_uart_control.h 已在 zf_common_headfile.h 中包含



#define ENCODER_PPR           4096.0f    // 编码器分辨率 (示例值，需查手册)
#define MOTOR_GEAR_RATIO      33.0f      // 电机减速比(示例值)
#define WHEEL_DIAMETER_MM     180.0f     // 车轮直径mm(示例值，需实测)

extern volatile float real_speed;
extern volatile float real_pulse;

// 函数声明（封装驱动层，保持接口兼容）
void motor_init(void);              // 初始化（调用small_driver_uart_init）
void motor_set(int32_t duty);       // 设置占空比（-10000~10000）
int16 motor_get_speed(void);        // 获取速度（返回FOC反馈的RPM）


float motor_pulses_to_m_s(float pulses_per_20ms);
void motor_calculate(void);

#endif /* CODE_APP_MOTOR_APP_H_ */
