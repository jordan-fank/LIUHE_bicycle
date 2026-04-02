/*
 * 文件: motor_app.h
 * 功能: 声明电机应用层接口、速度语义约定和车轮参数
 * 作者: 闫锦
 * 日期: 2026-03-31
 */

#ifndef CODE_APP_MOTOR_APP_H_
#define CODE_APP_MOTOR_APP_H_

#include "zf_common_headfile.h"
// small_driver_uart_control.h 已在 zf_common_headfile.h 中包含



/* ==================== 电机/车轮参数 ====================
   当前项目已经确认：
   1. motor_get_raw_rpm() 返回的是电机转速 RPM
   2. 本车为轮驱一体，减速比 = 1
   3. 车轮直径 = 64 mm
*/
#define MOTOR_GEAR_RATIO      1.0f                             // 减速比 = 1
#define MOTOR_WHEEL_DIAMETER_M 0.064f                          // 车轮直径（m）
#define MOTOR_WHEEL_RADIUS_M   (MOTOR_WHEEL_DIAMETER_M * 0.5f) // 车轮半径（m）


/* 车体方向符号约定：
   1. motor_get_raw_rpm() 保留 FOC 驱动原始 RPM 符号
   2. 上层控制统一约定“车辆前进为正”
   3. 当前工程按既有控制效果先取 -1

   注意：这里不是驱动文档直接给出的结论，而是项目级方向映射。
   换电机接线、换轮子安装方向、换车体装配后，都必须重新实测确认。
*/
#define MOTOR_FORWARD_SIGN    (-1.0f)



extern volatile float motor_speed_rpm;          // 当前车体方向下的电机转速（RPM，前进为正）
extern volatile float motor_speed_m_s;          // 当前车体方向下的线速度（m/s，前进为正）
extern volatile float target_motor_speed_m_s;   // 目标线速度（m/s，带方向）



// 函数声明（封装驱动层）
void motor_init(void);              // 初始化（调用small_driver_uart_init）
void motor_set(int32_t duty);       // 设置占空比（-10000~10000）


int16 motor_get_raw_rpm(void);              // 获取原始 FOC 反馈转速（RPM，保留驱动原始符号）
float motor_get_vehicle_rpm(void);          // 获取按车体方向统一后的 RPM（前进为正）
float motor_rpm_to_m_s(float rpm);          // RPM -> m/s，保留方向
float motor_get_vehicle_speed_m_s(void);    // 读取当前线速度（m/s，前进为正/后退为负）
float motor_get_vehicle_speed_abs_m_s(void);// 读取当前速度大小（m/s，恒正）

#endif /* CODE_APP_MOTOR_APP_H_ */
