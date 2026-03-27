/**
 * 
 * 添加页面操作：
 * 1.注册页面                  ->  ips_app——config.h
 * 2.extern外部需要显示的变量   ->  ips_app——config.h
 * 3.注册参数表                ->  ips_app——config.h
 * 4.注册参数列表的数组         ->  ips_app.c
 * 5.注册参数列表的长度         ->  ips_app.c
 * 6.注册标题                  ->  ips_app.c
 */

#ifndef IPS_APP_CONFIG_H
#define IPS_APP_CONFIG_H

#include "zf_common_headfile.h"






//==============================================================================
// 布局常量（屏幕 240x320，8x16 字体）
//==============================================================================
#define SCREEN_W        240     // 屏幕宽度
#define SCREEN_H        320     // 屏幕高度
#define TITLE_H         16      // 标题栏高度（= CHAR_H，精确1行字符高）
#define SEP_H           1       // 分隔线高度
#define ROW_H           16      // 每行参数高度（= CHAR_H，精确1行字符高）
#define ROW_START_Y     (TITLE_H + SEP_H)       // 第一行参数起始Y坐标，紧接分隔线
#define ROW_LEFT        4       // 左边距（像素）
#define ROW_RIGHT       4       // 右边距（像素）

// 选中指示符 "->" 占 2 字符 = 16px
#define INDICATOR_W     (2 * 8)


// 只读行数值缓存，用于判断是否真正变化（最多支持12行只读参数）
#define MAX_RDONLY_ROWS 12

//==============================================================================
// 颜色（白底黑字）
//==============================================================================
#define CLR_BG      RGB565_WHITE
#define CLR_FG      RGB565_BLACK




//==============================================================================
// 页面枚举（添加新页面时在 PAGE_COUNT 前插入）---第一步
//==============================================================================
typedef enum
{
    PAGE_MOTOR = 0,     // 电机 PID 参数
    PAGE_SERVO,         // 舵机 PID 参数
    PAGE_BATTERY,       // 电池信息（只读）
    PAGE_IMU,           // IMU（只读）
    PAGE_TEST,           // 用于测试（只读）
    PAGE_COUNT          // 页面总数，不要删除
} page_e;






//==============================================================================
// 参数行结构体
//==============================================================================
typedef struct
{
    const char *label;  // 显示标签，如 "Kp:"
    void       *value;  // 指向实际变量（float* 或 int32_t*）
    float       step;   // 每次按键的步长
    float       min;    // 最小值
    float       max;    // 最大值
    uint8       rdonly; // 1=只读，不响应增减按键
    uint8       dec;    // 小数显示位数：0=整数(int32_t)，1=x.x，2=x.xx
    uint8       is_int; // 1=value是int32_t*，0=value是float*
} param_t;




// 填参数表的两个快捷宏：
//   PARAM_F = 浮点参数   {标签, &float变量, 步长, 最小, 最大, 只读, 小数位}
//   PARAM_I = 整数参数   {标签, &int32_t变量, 步长(整数), 最小, 最大, 只读}
#define PARAM_F(label, pvar, step, min, max, rdonly, dec) \
    {label, (void*)(pvar), step, min, max, rdonly, dec, 0}

#define PARAM_I(label, pvar, step, min, max, rdonly) \
    {label, (void*)(pvar), step, min, max, rdonly, 0, 1}






//==============================================================================
// 全局变量声明（定义在 ips_app.c 中）---第二步
//==============================================================================

// 电机参数
extern float g_motor_kp;
extern float g_motor_ki;
extern float g_motor_kd;



// 电池 / 状态（只读显示）
extern float g_bat_voltage;
extern float g_vehicle_speed;

// 低电量报警阈值
#define BAT_LOW_V   10.5f


extern float speed;
extern int32_t temp;
extern float hum;


extern float roll_kalman;
extern float pitch_kalman;
extern float yaw_kalman;

extern float gyro_x_rate;
extern float gyro_y_rate;

extern uint8_t gyro_calibrated;

extern volatile float pwm_angle;






//==============================================================================
// 各页面参数表
// 用 PARAM_F(...) 填浮点参数，用 PARAM_I(...) 填整数参数      --第三步
//   PARAM_F(标签, &float变量, 步长, 最小, 最大, 只读(1=只读), 小数位(小数位数))
//   PARAM_I(标签, &int32_t变量, 步长, 最小, 最大, 只读(1=只读))
//==============================================================================


// 电机页面
#define MOTOR_PARAM_LIST \
    PARAM_F("Kp:", &g_motor_kp, 0.1f,  0.0f, 100.0f, 0, 2), \
    PARAM_F("Ki:", &g_motor_ki, 0.01f, 0.0f,  10.0f, 0, 2), \
    PARAM_F("Kd:", &g_motor_kd, 0.1f,  0.0f, 100.0f, 0, 2)

// 舵机页面
#define SERVO_PARAM_LIST \
    PARAM_F("Kp:",     &g_balance_kp,     0.1f,  0.0f,  100.0f, 0, 2), \
    PARAM_F("Ki:",     &g_balance_ki,     0.01f, 0.0f,   10.0f, 0, 2), \
    PARAM_F("Kd:",     &g_balance_kd,     0.1f,  0.0f,  100.0f, 0, 2), \
    PARAM_F("output_limit:",    &g_balance_output_limit,     1.0f,  0.0f,  300.0f, 0, 2), \
    PARAM_F("integral_limit:",  &g_balance_integral_limit,   1.0f,  0.0f,  300.0f, 0, 2), \
    PARAM_I("left_limit:", &g_servo_left_limit,   10,  0, 10000, 0), \
    PARAM_I("mid_duty:", &g_servo_mid_duty,     10,  0, 10000, 0), \
    PARAM_I("right_limit:", &g_servo_right_limit,  10,  0, 10000, 0), \
    PARAM_F("pwm_angle:",&pwm_angle,   0,  0, 0, 1, 2)

// 电池页面
#define BATTERY_PARAM_LIST \
    PARAM_F("Voltage:", &g_bat_voltage,   0, 0, 0, 1, 2), \
    PARAM_F("Speed:",   &g_vehicle_speed, 0, 0, 0, 1, 1)

// 传感器页面
#define IMU_PARAM_LIST \
        PARAM_F("roll:", &roll_kalman, 0,  0, 0, 1, 2), \
        PARAM_F("pitch:",&pitch_kalman,0,  0, 0, 1, 2), \
        PARAM_F("yaw:", &yaw_kalman,   0,  0, 0, 1, 2), \
        PARAM_F("gyro_x:", &gyro_x_rate,   0,  0, 0, 1, 2), \
        PARAM_F("gyro_y:", &gyro_y_rate,   0,  0, 0, 1, 2), \
        PARAM_I("calibrated:", &gyro_calibrated,   0,  0, 0, 1),\
        PARAM_F("roll_ctrl:",&roll_ctrl_angle,0,  0, 0, 1, 2), \
        PARAM_F("pitch_ctrl:",&pitch_ctrl_angle,   0,  0, 0, 1, 2), \
        PARAM_F("pwm_angle:",&pwm_angle,   0,  0, 0, 1, 2), \
        PARAM_I("drop_count:", &g_imu_diag_stat.drop_count,   0,  0, 0, 1)
        

//测试界面
#define TEST_PARAM_LIST PARAM_I("drop_count:", &g_imu_diag_stat.drop_count,   0,  0, 0, 1)
//    PARAM_I("last:",&g_scheduler_stat.last_us ,   0,  0, 0, 1), \
//    PARAM_I("max:", &g_scheduler_stat.max_us,   0,  0, 0, 1), \




#endif /* IPS_APP_CONFIG_H */
