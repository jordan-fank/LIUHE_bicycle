/*
 * pid_app.h
 *
 *  Created on: 2025年11月12日
 *      Author: suiyungui
 */

#ifndef CODE_APP_PID_APP_H_
#define CODE_APP_PID_APP_H_

/* 仅引入本头文件声明所需的最小依赖，避免和 zf_common_headfile.h 形成循环包含 */
#include <stdint.h>
#include "pid_driver.h"
#include "zf_common_headfile.h"

// ==================== 全局变量声明 ====================

extern PID_T balance_pid;        // 平衡控制PID结构体
extern float expect_angle;       // 期望倾角（用于转弯控制）

/* ==================== NEW: 平衡控制参数全局变量 ====================
   这些变量替代了原来的 PID 宏定义，供控制逻辑和 IPS 参数页面共同使用。
*/
extern volatile float g_balance_kp;
extern volatile float g_balance_ki;
extern volatile float g_balance_kd;
extern volatile float g_balance_output_limit;
extern volatile float g_balance_integral_limit;




// ==================== 函数声明 ====================
void balance_pid_init(void);                          // 初始化平衡PID
void balance_control(void);                           // 核心平衡控制函数（10ms中断调用）
void balance_set_expect_angle(float angle);           // 设置期望倾角
void balance_set_params(float kp, float ki, float kd); // 运行时调整PID参数



/* ==================== NEW: 平衡控制使能开关 ====================
   用途：
   1. 上电阶段先关闭舵机平衡控制
   2. 等 IMU 完成零偏校准、机械零位捕获后再打开控制
   3. 避免零位捕获阶段舵机动作干扰 IMU 静态姿态
*/
void balance_control_set_enable(uint8_t enable);

#endif /* CODE_APP_PID_APP_H_ */
