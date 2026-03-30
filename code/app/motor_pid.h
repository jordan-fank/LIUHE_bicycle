#ifndef CODE_APP_MOTOR_PID_H_
#define CODE_APP_MOTOR_PID_H_

#include "zf_common_headfile.h"

// ==================== 速度辅助平衡 (源自逐飞方案) ====================
// 公式：ideal_speed = base_speed + abs(roll_angle) * assist_gain
// 当车辆倾斜时，增加速度以产生更大的离心力

typedef struct {
    float base_speed;       // 基础目标速度 (RPM)
    float assist_gain;      // 每度倾斜的速度增益 (RPM/度)
    float max_boost;        // 最大速度增益 (RPM)
    uint8_t enabled;        // 使能标志
} SpeedAssist_t;

extern SpeedAssist_t speed_assist;
extern float target_speed;         // 目标速度 (单位：脉冲/20ms)
extern float real_target_speed;

void motor_pid_init(void);                             // 电机 PID 初始化
void motor_control(void);                              // 电机控制
void motor_set_speed(float speed);                     // 设置目标速度
void motor_set_params(float kp, float ki, float kd);   // 设置 PID 参数
void motor_pid_reset(void);                            // 重置 PID 状态

// 速度辅助函数
void speed_assist_init(float base_speed, float assist_gain, float max_boost);  // 速度辅助初始化
void speed_assist_enable(uint8_t enable);     // 速度辅助使能
void speed_assist_set_base(float base_speed); // 设置基础速度
float speed_assist_calculate(float roll_angle_deg);  // 计算速度辅助 (根据横滚角)


/* ==================== NEW: 电机 PID 参数全局变量 ====================
   这些变量替代原来的 MOTOR_KP / MOTOR_KI / MOTOR_KD / MOTOR_LIMIT 宏定义，
   供电机控制逻辑和后续 IPS 参数页面共同使用。
*/
extern volatile float g_motor_kp;
extern volatile float g_motor_ki;
extern volatile float g_motor_kd;
extern volatile float g_motor_output_limit;


#endif // CODE_APP_MOTOR_PID_H_

