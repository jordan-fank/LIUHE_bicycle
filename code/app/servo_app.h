/*
 * servo_app.h
 *
 *  Created on: 2025年11月11日
 *      Author: suiyungui
 */

#ifndef CODE_APP_SERVO_APP_H_
#define CODE_APP_SERVO_APP_H_

#include "zf_common_headfile.h"

// ==================== 舵机类型选择 ====================
// 可选值:
//   - SERVO_TYPE_MG996R   (50Hz, 调试用)
//   - SERVO_TYPE_BDS300   (330Hz, 比赛用)
// ===================================================
#define SERVO_TYPE_MG996R     0
#define SERVO_TYPE_BDS300     1

// ============ 在这里选择舵机类型 ============
#define CURRENT_SERVO_TYPE    SERVO_TYPE_MG996R
// ===========================================


#if (CURRENT_SERVO_TYPE == SERVO_TYPE_MG996R)
    // MG996R 参数 (50Hz, 周期 20ms)
    // 脉宽范围：500μs ~ 2500μs，中位 1500μs
    #define SERVO_FREQ         50
    #define SERVO_MIN_US       500     // 最小脉宽 500μs
    #define SERVO_MID_US       1500    // 中位脉宽 1500μs
    #define SERVO_MAX_US       2500    // 最大脉宽 2500μs
    
    // 换算成占空比 (PWM_DUTY_MAX=10000, 周期 20ms)
    // 公式：占空比 = (脉宽 / 周期) × PWM_DUTY_MAX
    #define l_max  250     // 对应 500μs
    #define mid    750     // 对应1500μs
    #define r_max  1250    // 对应2500μs
    
    // 编译提示：当前使用 MG996R 舵机 (50Hz)
#elif (CURRENT_SERVO_TYPE == SERVO_TYPE_BDS300)
    // BDS300 参数 (330Hz, 周期 3.03ms)
    // 脉宽范围需根据手册调整，此处使用默认值
    #define SERVO_FREQ         330
    #define SERVO_MIN_US       1000    // 约 1240μs
    #define SERVO_MID_US       1500    // 约 1520μs
    #define SERVO_MAX_US       2000    // 约 1820μs
    
    // #define l_max  4108    // 左极限
    // #define mid    5016    // 中位
    // #define r_max  6000    // 右极限

    #define l_max  3300    // 对应 1000μs
    #define mid    4950    // 对应 1500μs
    #define r_max  6600    // 对应 2000μs
    
    // 编译提示：当前使用 BDS300 舵机 (330Hz)


#else
    #error "请选择正确的舵机类型！"
#endif


void servo_set(uint32_t duty);
void servo_test(void);
void servo_init(void);  // 新增初始化函数

#endif /* CODE_APP_SERVO_APP_H_ */
