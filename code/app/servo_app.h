
#ifndef CODE_APP_SERVO_APP_H_
#define CODE_APP_SERVO_APP_H_

#include "zf_common_headfile.h"


#define SERVO_TYPE_MG996R     0
#define SERVO_TYPE_BDS300     1


// ==================== 舵机类型选择 ====================
// 一键切换舵机类型
//   - SERVO_TYPE_MG996R   (50Hz, 调试用)
//   - SERVO_TYPE_BDS300   (330Hz, 比赛用)
// ===================================================
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
#define l_max  (g_servo_left_limit)
#define mid    (g_servo_mid_duty)
#define r_max  (g_servo_right_limit)


    
    // 编译提示：当前使用 MG996R 舵机 (50Hz)
#elif (CURRENT_SERVO_TYPE == SERVO_TYPE_BDS300)
    // BDS300 参数 (330Hz, 周期 3.03ms)
    // 舵机硬件支持：500-2500μs (0°-180°), 50-333Hz可调
    // 实际应用限制：自行车机械限位 45°-135° → 1000-2000μs
    // 注意：极限位置由车架机械结构决定，不是舵机电气限制
    
    #define SERVO_FREQ         330
    #define SERVO_MIN_US       1000    // 机械限位 45°
    #define SERVO_MID_US       1500    // 中位 90°
    #define SERVO_MAX_US       2000    // 机械限位 135°
    
    // #define l_max  4108    // 500μs @ 330Hz → 0°(硬件极限，不建议使用)
    // #define mid    5016    // 1500μs @ 330Hz → 90°
    // #define r_max  6000    // 2500μs @ 330Hz → 180°(硬件极限，不建议使用)
    //左极限占空比 = 1000*330/1000*10000

    #define l_max  3300    // 1000μs @ 330Hz → 45°(机械限位)
    #define mid    4950    // 1500μs @ 330Hz → 90°
    #define r_max  6600    // 2000μs @ 330Hz → 135°(机械限位)
    
    // 编译提示：当前使用 BDS300 舵机 (330Hz, 机械限位 45°-135°)


#else
    #error "请选择正确的舵机类型！"
#endif




// ==================== 角度转换宏(通用公式) ====================
// 公式来源：逐飞舵机示例代码
// 占空比 → 角度：angle = 90 × (duty × (1000/freq) / PWM_DUTY_MAX - 0.5)
// 角度 → 占空比：duty = PWM_DUTY_MAX / (1000/freq) × (0.5 + angle/90)
#define SERVO_DUTY_TO_ANGLE(duty)     (90.0f * ((duty) * (1000.0f / (float)SERVO_FREQ) / (float)PWM_DUTY_MAX - 0.5f))
#define SERVO_ANGLE_TO_DUTY(angle)    ((uint32_t)((float)PWM_DUTY_MAX / (1000.0f / (float)SERVO_FREQ) * (0.5f + (float)(angle) / 90.0f)))


void servo_set(uint32_t duty);
void servo_test(void);
void servo_init(void);  // 新增初始化函数

/* ==================== NEW: 舵机参数全局变量 ====================
   这组变量替代原来的 l_max / mid / r_max 宏定义思路，供控制逻辑和 IPS 页面共用。
   实际默认值在 servo_app.c 中根据当前舵机类型初始化。
*/
extern volatile uint32_t g_servo_left_limit;
extern volatile uint32_t g_servo_mid_duty;
extern volatile uint32_t g_servo_right_limit;

#endif /* CODE_APP_SERVO_APP_H_ */
