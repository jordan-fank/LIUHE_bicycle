#ifndef CODE_APP_PID_APP_H_
#define CODE_APP_PID_APP_H_

/* 仅引入本头文件声明所需的最小依赖，避免和 zf_common_headfile.h 形成循环包含 */
#include <stdint.h>
#include "pid_driver.h"
#include "zf_common_headfile.h"

// ==================== 全局变量声明 ====================

extern PID_T balance_pid;        // 平衡控制PID结构体
extern volatile float expect_angle;       // 期望倾角（用于转弯控制）

/* [新增] 舵机平衡控制模式
   SIMPLE_PD : 原始位置式 P+I+D（D项直接用陀螺，当前默认）
   CASCADE   : 串级双环——外环角度→内环角速率，适合中高速平衡
   LOW_SPEED : 纯角度环 P+I，去掉陀螺项，适合题目二低速绕八字 */
#define SERVO_CTRL_MODE_SIMPLE_PD   0u
#define SERVO_CTRL_MODE_CASCADE     1u
#define SERVO_CTRL_MODE_LOW_SPEED   2u

/* 舵机转向方向系数：
   +1：保持当前输出方向
   -1：整体反向（适用于摇臂/拉杆安装后发现左右打舵相反） */
#define SERVO_STEER_DIR_NORMAL      (1)
#define SERVO_STEER_DIR_REVERSED    (-1)

extern volatile uint8_t g_servo_control_mode;
extern volatile int8_t  g_servo_steer_dir;

/* [新增] 串级内环（角速率环）参数，仅在 CASCADE 模式下生效
   内环输入：目标角速率 - 当前陀螺角速率（deg/s）
   内环输出：舵机偏置（duty counts）                        */
extern volatile float g_balance_inner_kp;
extern volatile float g_balance_inner_ki;

/* ==================== NEW: 平衡控制参数全局变量 ====================
   这些变量替代了原来的 PID 宏定义，供控制逻辑和 IPS 参数页面共同使用。
*/
extern volatile float g_balance_kp;
extern volatile float g_balance_ki;
extern volatile float g_balance_kd;
extern volatile float g_balance_output_limit;
extern volatile float g_balance_integral_limit;
extern volatile float pwm_angle;
extern volatile float g_balance_pid_output;

// ==================== 函数声明 ====================
void balance_pid_init(void);                           // 初始化平衡PID
void balance_control(void);                            // 核心平衡控制函数（5ms中断调用）
void balance_set_expect_angle(float angle);            // 设置期望倾角
void balance_set_params(float kp, float ki, float kd); // 运行时调整PID参数

/* ==================== NEW: 平衡控制使能开关 ====================
   用途：
   1. 上电阶段先关闭舵机平衡控制
   2. 等 IMU 完成零偏校准、机械零位捕获后再打开控制
   3. 避免零位捕获阶段舵机动作干扰 IMU 静态姿态
*/
void balance_control_set_enable(uint8_t enable);

#endif /* CODE_APP_PID_APP_H_ */
