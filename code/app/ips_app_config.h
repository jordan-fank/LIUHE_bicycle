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
#include "nav_app.h"
#include "subject1_app.h"
#include "subject2_app.h"
#include "subject3_app.h"

//==============================================================================
// 布局常量（屏幕 240x320，8x16 字体）
//==============================================================================
#define SCREEN_W        240
#define SCREEN_H        320
#define TITLE_H         16
#define SEP_H           1
#define ROW_H           16
#define ROW_START_Y     (TITLE_H + SEP_H)
#define ROW_LEFT        4
#define ROW_RIGHT       4
#define INDICATOR_W     (2 * 8)

// 只读行数值缓存，用于判断是否真正变化（最多支持16行只读参数）
#define MAX_RDONLY_ROWS 16

//==============================================================================
// 颜色（白底黑字）
//==============================================================================
#define CLR_BG      RGB565_WHITE
#define CLR_FG      RGB565_BLACK

//==============================================================================
// 页面枚举（添加新页面时在 PAGE_COUNT 前插入）
//==============================================================================
typedef enum
{
    PAGE_HOME = 0,
    PAGE_MOTOR,
    PAGE_SERVO,
    PAGE_GPS,
    PAGE_IMU,
    PAGE_TEST,
    PAGE_SUBJ1,
    PAGE_SUBJ2,
    PAGE_SUBJ3,
    PAGE_COUNT
} page_e;

//==============================================================================
// 参数行结构体
//==============================================================================
typedef struct
{
    const char *label;
    void       *value;
    float       step;
    float       min;
    float       max;
    uint8       rdonly;
    uint8       dec;
    uint8       is_int;
} param_t;

//==============================================================================
// 参数表快捷宏
//==============================================================================
#define PARAM_F(label, pvar, step, min, max, rdonly, dec) \
    {label, (void*)(pvar), step, min, max, rdonly, dec, 0}

#define PARAM_I(label, pvar, step, min, max, rdonly) \
    {label, (void*)(pvar), step, min, max, rdonly, 0, 1}

#define PARAM_U32(label, pvar, step, min, max, rdonly) \
    {label, (void*)(pvar), step, min, max, rdonly, 0, 2}

#define PARAM_U8(label, pvar, step, min, max, rdonly) \
    {label, (void*)(pvar), step, min, max, rdonly, 0, 3}

#define PARAM_D(label, pvar, step, min, max, rdonly, dec) \
    {label, (void*)(pvar), step, min, max, rdonly, dec, 4}

//==============================================================================
// 外部变量声明
//==============================================================================
extern float g_bat_voltage;
extern float g_vehicle_speed;
extern float speed;
extern int32_t temp;
extern float hum;

extern float roll_kalman;
extern float pitch_kalman;
extern float yaw_kalman;
extern float gyro_x_rate;
extern float gyro_y_rate;
extern uint32_t gyro_calibrated;
extern volatile uint32_t g_ctrl_5ms_last_us;
extern volatile uint32_t g_ctrl_5ms_max_us;
extern volatile uint32_t g_ctrl_5ms_min_us;
extern volatile uint32_t g_ctrl_5ms_run_count;
extern volatile float pwm_angle;

extern uint8_t g_active_subject;
extern uint8_t g_home_init_imu_ok;
extern uint8_t g_home_init_gps_ok;
extern uint8_t g_home_init_motor_ok;
extern uint8_t g_home_init_wifi_ok;
extern uint8_t g_home_init_nav_ok;
extern uint8_t g_home_flash_loaded;

//==============================================================================
// 各页面参数表
//==============================================================================
#define HOME_PARAM_LIST \
    PARAM_U8 ("Active:",   &g_active_subject,        0,      0,      0,      1), \
    PARAM_U8 ("GPS:",      &gnss.state,              0,      0,      0,      1), \
    PARAM_I  ("Nav:",      &g_nav_state,             0,      0,      0,      1), \
    PARAM_U8 ("Ctrl:",     &g_servo_control_mode,    0,      0,      0,      1), \
    PARAM_F  ("HdgErr:",   &g_nav_heading_error,     0,      0,      0,      1, 1), \
    PARAM_F  ("Motor:",    &motor_speed_rpm,         0,      0,      0,      1, 1), \
    PARAM_U32("5msMax:",   &g_ctrl_5ms_max_us,       0,      0,      0,      1), \
    PARAM_F  ("Yaw:",      &yaw_kalman,              0,      0,      0,      1, 1), \
    PARAM_U8 ("IMUinit:",  &g_home_init_imu_ok,      0,      0,      0,      1), \
    PARAM_U8 ("GPSinit:",  &g_home_init_gps_ok,      0,      0,      0,      1), \
    PARAM_U8 ("MotInit:",  &g_home_init_motor_ok,    0,      0,      0,      1), \
    PARAM_U8 ("WiFiInit:", &g_home_init_wifi_ok,     0,      0,      0,      1), \
    PARAM_U8 ("NavInit:",  &g_home_init_nav_ok,      0,      0,      0,      1), \
    PARAM_U8 ("CfgLd:",    &g_home_flash_loaded,     0,      0,      0,      1)

#define MOTOR_PARAM_LIST \
    PARAM_F("Kp:",           &g_motor_kp,             0.1f,   0.0f,     100.0f, 0, 2), \
    PARAM_F("Ki:",           &g_motor_ki,             0.01f,  0.0f,      10.0f, 0, 2), \
    PARAM_F("Kd:",           &g_motor_kd,             0.1f,   0.0f,     100.0f, 0, 2), \
    PARAM_F("pid_output:",   &g_motor_pid_output,     0,      0,           0,   1, 2), \
    PARAM_F("output_limit:", &g_motor_output_limit,   10.0f,  0.0f,   10000.0f, 0, 2), \
    PARAM_F("target_rpm:",   &target_motor_rpm,       10.0f, -1500.0f, 1500.0f, 0, 2), \
    PARAM_F("target_m_s:",   &target_motor_speed_m_s, 0,      0,           0,   1, 2), \
    PARAM_F("real_rpm:",     &motor_speed_rpm,        0,      0,           0,   1, 2), \
    PARAM_F("real_m_s:",     &motor_speed_m_s,        0,      0,           0,   1, 2)

#define SERVO_PARAM_LIST \
    PARAM_F("Kp:",             &g_balance_kp,             0.1f,   0.0f,   100.0f, 0, 2), \
    PARAM_F("Ki:",             &g_balance_ki,             0.01f,  0.0f,    10.0f, 0, 2), \
    PARAM_F("Kd:",             &g_balance_kd,             0.1f,   0.0f,   100.0f, 0, 2), \
    PARAM_F("output_limit:",   &g_balance_output_limit,   1.0f,   0.0f,  1000.0f, 0, 2), \
    PARAM_F("integral_limit:", &g_balance_integral_limit, 1.0f,   0.0f,   300.0f, 0, 2), \
    PARAM_I("left_limit:",     &g_servo_left_limit,       10,     0,      10000,  0), \
    PARAM_I("mid_duty:",       &g_servo_mid_duty,         10,     0,      10000,  0), \
    PARAM_I("right_limit:",    &g_servo_right_limit,      10,     0,      10000,  0), \
    PARAM_F("pwm_angle:",      &pwm_angle,                0,      0,          0,  1, 2)

#define GPS_PARAM_LIST \
    PARAM_U8("second:",      &gnss.time.second,        0, 0, 0, 1), \
    PARAM_U8("state:",       &gnss.state,              0, 0, 0, 1), \
    PARAM_U8("satellite:",   &gnss.satellite_used,     0, 0, 0, 1), \
    PARAM_D( "latitude:",    &g_gps_latitude_display,  0, 0, 0, 1, 6), \
    PARAM_D( "longitude:",   &g_gps_longitude_display, 0, 0, 0, 1, 6), \
    PARAM_F( "speed:",       &gnss.speed,              0, 0, 0, 1, 1), \
    PARAM_F( "direction:",   &gnss.direction,          0, 0, 0, 1, 1), \
    PARAM_F( "height:",      &gnss.height,             0, 0, 0, 1, 1)

#define IMU_PARAM_LIST \
    PARAM_F("roll:",       &roll_kalman,                 0, 0, 0, 1, 2), \
    PARAM_F("pitch:",      &pitch_kalman,                0, 0, 0, 1, 2), \
    PARAM_F("yaw:",        &yaw_kalman,                  0, 0, 0, 1, 2), \
    PARAM_F("gyro_x:",     &gyro_x_rate,                 0, 0, 0, 1, 2), \
    PARAM_F("gyro_y:",     &gyro_y_rate,                 0, 0, 0, 1, 2), \
    PARAM_I("calibrated:", &gyro_calibrated,             0, 0, 0, 1), \
    PARAM_F("roll_ctrl:",  &roll_ctrl_angle,             0, 0, 0, 1, 2), \
    PARAM_F("pitch_ctrl:", &pitch_ctrl_angle,            0, 0, 0, 1, 2), \
    PARAM_F("pwm_angle:",  &pwm_angle,                   0, 0, 0, 1, 2), \
    PARAM_I("drop_count:", &g_imu_diag_stat.drop_count,  0, 0, 0, 1)

#define TEST_PARAM_LIST \
    PARAM_I("drop_count:", &g_imu_diag_stat.drop_count, 0, 0, 0, 1), \
    PARAM_U32("5ms_last:", &g_ctrl_5ms_last_us,         0, 0, 0, 1), \
    PARAM_U32("5ms_max:",  &g_ctrl_5ms_max_us,          0, 0, 0, 1), \
    PARAM_U32("5ms_min:",  &g_ctrl_5ms_min_us,          0, 0, 0, 1), \
    PARAM_U32("5ms_cnt:",  &g_ctrl_5ms_run_count,       0, 0, 0, 1)

#define SUBJ1_PARAM_LIST \
    PARAM_I ("State:",    &g_subject1_state,        0,      0,      0,      1), \
    PARAM_F ("Dist:",     &g_nav_dist_to_wp,        0,      0,      0,      1, 2), \
    PARAM_F ("HdgErr:",   &g_nav_heading_error,     0,      0,      0,      1, 1), \
    PARAM_F ("HIGH:",     &g_s1_high_rpm,           10.0f,  0.0f,   2000.0f, 0, 1), \
    PARAM_F ("MID:",      &g_s1_mid_rpm,            10.0f,  0.0f,   2000.0f, 0, 1), \
    PARAM_F ("TURN:",     &g_s1_turn_rpm,           10.0f,  0.0f,   2000.0f, 0, 1), \
    PARAM_F ("BrakeDst:", &g_s1_pre_brake_dist,     0.1f,   0.5f,   20.0f,   0, 1), \
    PARAM_F ("FinDst:",   &g_s1_finish_brake_dist,  0.1f,   0.5f,   20.0f,   0, 1), \
    PARAM_F ("Resume:",   &g_s1_resume_thresh,      1.0f,   1.0f,   90.0f,   0, 1), \
    PARAM_F ("Accel:",    &g_s1_accel_step,         5.0f,   0.0f,   300.0f,  0, 1)

#define SUBJ2_PARAM_LIST \
    PARAM_I ("State:",    &g_subject2_state,        0,      0,      0,      1), \
    PARAM_F ("HdgErr:",   &g_nav_heading_error,     0,      0,      0,      1, 1), \
    PARAM_F ("IMUhdg:",   &g_nav_imu_heading,       0,      0,      0,      1, 1), \
    PARAM_F ("Dist:",     &g_nav_distance_m,        0,      0,      0,      1, 2), \
    PARAM_F ("RPM:",      &g_s2_rpm,                10.0f,  50.0f,  800.0f,  0, 1), \
    PARAM_F ("Radius:",   &g_s2_turn_radius,        0.05f,  0.3f,   1.5f,    0, 2)

#define SUBJ3_PARAM_LIST \
    PARAM_I ("State:",    &g_subject3_state,        0,      0,      0,      1), \
    PARAM_F ("Dist:",     &g_nav_dist_to_wp,        0,      0,      0,      1, 2), \
    PARAM_F ("HdgErr:",   &g_nav_heading_error,     0,      0,      0,      1, 1), \
    PARAM_F ("HIGH:",     &g_s3_go_rpm,             10.0f,  0.0f,   2000.0f, 0, 1), \
    PARAM_F ("MID:",      &g_s3_mid_rpm,            10.0f,  0.0f,   2000.0f, 0, 1), \
    PARAM_F ("TURN:",     &g_s3_turn_rpm,           10.0f,  0.0f,   2000.0f, 0, 1), \
    PARAM_F ("BrakeDst:", &g_s3_pre_brake_dist,     0.1f,   0.5f,   20.0f,   0, 1), \
    PARAM_F ("FinDst:",   &g_s3_finish_brake_dist,  0.1f,   0.5f,   20.0f,   0, 1), \
    PARAM_F ("Resume:",   &g_s3_resume_thresh,      1.0f,   1.0f,   90.0f,   0, 1), \
    PARAM_F ("Accel:",    &g_s3_accel_step,         5.0f,   0.0f,   300.0f,  0, 1)

#endif /* IPS_APP_CONFIG_H */
