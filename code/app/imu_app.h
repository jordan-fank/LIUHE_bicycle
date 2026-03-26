/*
 * imu.h
 *
 *  Created on: 2025年11月14日
 *      Author: suiyungui
 */

#ifndef CODE_IMU_H_
#define CODE_IMU_H_

#include "zf_common_headfile.h"

// 统一时基
#define IMU_PERIOD_MS   (5U)
#define IMU_DT_S        ((float)IMU_PERIOD_MS * 0.001f)  //IMU_DT_S时基用于积分，和采样周期一致


// IMU 原始姿态输出
extern float roll_kalman;
extern float pitch_kalman;
extern float yaw_kalman;
extern float gyro_x_rate;
extern float gyro_y_rate;

/* ==================== NEW: 机械零位补偿后的控制角度 ====================
   这两个变量是新增的。

   含义：
   1. roll_kalman / pitch_kalman / yaw_kalman 仍然表示 IMU 原始姿态输出
   2. roll_ctrl_angle / pitch_ctrl_angle 是控制专用角度，等于“原始姿态角 - 机械零位”

   为什么这里只处理 roll / pitch，不处理 yaw / gyro：
   1. yaw_kalman：
      当前工程没有磁力计，yaw 只是相对偏航角，不是绝对航向角，
      一般不作为“机械装配零位补偿”的核心对象
   2. gyro_x_rate / gyro_y_rate：
      它们是角速度，不是姿态角；
      其零偏已经在 imu_calibrate_gyro() / imu_calibrate_gyro_temp() 中完成校准，
      不应该再按“姿态角零位”的思路补偿一次
*/
extern float roll_ctrl_angle;
extern float pitch_ctrl_angle;

/* ==================== TEST ONLY: IMU采样/丢帧统计 ==================== */
typedef struct
{
    uint32_t sample_count;
    uint32_t process_count;
    uint32_t drop_count;
} imu_diag_stat_t;

extern volatile imu_diag_stat_t g_imu_diag_stat;
/* ==================== TEST ONLY: IMU采样/丢帧统计 ==================== */

void imu_sample_isr(void);
void imu_proc(void);
void imu_all_init(void);
void imu_calibrate_gyro(void);          // 陀螺仪零偏校准
void imu_calibrate_gyro_temp(void);     // 校准的测试值

/* ==================== NEW: 机械零位补偿接口 ====================
   imu_set_control_zero():
   - 手动指定机械零位对应的原始姿态角
   - 例如静止时测得 roll_kalman=4.27, pitch_kalman=0.38
     则调用 imu_set_control_zero(4.27f, 0.38f);

   imu_capture_control_zero():
   - 不手填数值，直接把“当前姿态”记录为机械零位
   - 适合把车摆到目标机械零位后调用一次
*/
void imu_set_control_zero(float roll_zero_deg, float pitch_zero_deg);
void imu_capture_control_zero(void);

void imu_test(void);




void zero_compensation(void);    // 零位补偿

#endif /* CODE_IMU_H_ */
