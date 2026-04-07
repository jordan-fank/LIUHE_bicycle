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

/*======================开机主菜单=========================*/
#define HOME_PARAM_LIST \
    PARAM_U8 ("Active:",   &g_active_subject,        1,      0,      3,      0), /* 当前激活模式，可调；0=手动/SIMPLE_PD，1/2/3=科目1/2/3，并自动同步 Ctrl */ \
    PARAM_U8 ("Ctrl:",     &g_servo_control_mode,    0,      0,      0,      1), /* 舵机控制模式编号，只读；Active=0 对应模式0，Active=1/3 对应 CASCADE，Active=2 对应 LOW_SPEED */ \
    PARAM_U8 ("GPS:",      &gnss.state,              0,      0,      0,      1), /* GNSS 当前定位状态，只读；用于判断是否满足勘线和导航的基本定位条件 */ \
    PARAM_I  ("Nav:",      &g_nav_state,             0,      0,      0,      1), /* 导航状态机当前状态，只读；用于观察是否空闲、导航中或已到达目标 */ \
    PARAM_F  ("HdgErr:",   &g_nav_heading_error,     0,      0,      0,      1, 1), /* 当前导航航向误差，单位度，只读；用于判断车头与目标方向的偏差大小 */ \
    PARAM_F  ("Motor:",    &motor_speed_rpm,         0,      0,      0,      1, 1), /* 电机实时转速反馈，单位 RPM，只读；用于确认动力输出是否达到预期 */ \
    PARAM_U32("5msMax:",   &g_ctrl_5ms_max_us,       0,      0,      0,      1), /* 5ms 控制周期最大执行耗时，单位 us，只读；用于排查实时性抖动 */ \
    PARAM_F  ("Yaw:",      &yaw_kalman,              0,      0,      0,      1, 1), /* IMU 当前偏航角估计值，单位度，只读；用于观察整车航向变化是否连续 */ \
    PARAM_U8 ("IMUinit:",  &g_home_init_imu_ok,      0,      0,      0,      1), /* IMU 初始化成功标志，只读；上电自检时用于确认姿态链已建立 */ \
    PARAM_U8 ("GPSinit:",  &g_home_init_gps_ok,      0,      0,      0,      1), /* GPS 初始化成功标志，只读；用于确认 GNSS 串口与模块工作正常 */ \
    PARAM_U8 ("MotInit:",  &g_home_init_motor_ok,    0,      0,      0,      1), /* 电机模块初始化成功标志，只读；用于确认驱动链已准备完成 */ \
    PARAM_U8 ("WiFiInit:", &g_home_init_wifi_ok,     0,      0,      0,      1), /* 无线调试链路初始化标志，只读；用于确认上位机通信功能是否可用 */ \
    PARAM_U8 ("NavInit:",  &g_home_init_nav_ok,      0,      0,      0,      1), /* 导航模块初始化标志，只读；用于确认导航缓存和状态机已正确建立 */ \
    PARAM_U8 ("CfgLd:",    &g_home_flash_loaded,     0,      0,      0,      1)  /* Flash 配置加载标志，只读；用于确认上电后参数已从持久化区恢复 */

/*======================电机页面相关参数=========================*/
#define MOTOR_PARAM_LIST \
    PARAM_F("Kp:",           &g_motor_kp,             0.1f,   0.0f,     100.0f, 0, 2), /* 电机速度环比例系数，可调；越大响应越快，但过大易振荡 */ \
    PARAM_F("Ki:",           &g_motor_ki,             0.01f,  0.0f,      10.0f, 0, 2), /* 电机速度环积分系数，可调；用于消除稳态误差，过大易积累饱和 */ \
    PARAM_F("Kd:",           &g_motor_kd,             0.1f,   0.0f,     100.0f, 0, 2), /* 电机速度环微分系数，可调；用于抑制转速变化过程中的超调 */ \
    PARAM_F("pid_output:",   &g_motor_pid_output,     0,      0,           0,   1, 2), /* 电机 PID 实时输出值，只读；用于观察控制器当前给出的驱动量 */ \
    PARAM_F("output_limit:", &g_motor_output_limit,   10.0f,  0.0f,   10000.0f, 0, 2), /* 电机 PID 输出限幅，可调；用于限制最大驱动强度 */ \
    PARAM_F("target_rpm:",   &target_motor_rpm,       10.0f, -1500.0f, 1500.0f, 0, 2), /* 电机目标转速，单位 RPM，可调；用于直接验证速度环跟踪能力 */ \
    PARAM_F("target_m_s:",   &target_motor_speed_m_s, 0,      0,           0,   1, 2), /* 目标线速度，单位 m/s，只读；由目标转速换算得到，便于直观查看设定值 */ \
    PARAM_F("real_rpm:",     &motor_speed_rpm,        0,      0,           0,   1, 2), /* 实际电机转速，单位 RPM，只读；用于对比目标与反馈误差 */ \
    PARAM_F("real_m_s:",     &motor_speed_m_s,        0,      0,           0,   1, 2)  /* 实际线速度，单位 m/s，只读；用于观察整车真实速度 */

/*======================舵机页面相关参数=========================*/
#define SERVO_PARAM_LIST \
    PARAM_F("Kp:",             &g_balance_kp,             0.1f,   0.0f,   100.0f, 0, 2), /* 平衡/转向比例系数，可调；决定误差转成舵量的直接力度 */ \
    PARAM_F("Ki:",             &g_balance_ki,             0.01f,  0.0f,    10.0f, 0, 2), /* 平衡/转向积分系数，可调；用于补偿长期偏差，但过大易产生拖尾 */ \
    PARAM_F("Kd:",             &g_balance_kd,             0.1f,   0.0f,   100.0f, 0, 2), /* 平衡/转向微分系数，可调；用于抑制姿态变化过快时的超调 */ \
    PARAM_F("output_limit:",   &g_balance_output_limit,   1.0f,   0.0f,  1000.0f, 0, 2), /* 平衡控制输出限幅，可调；用于约束最大控制强度 */ \
    PARAM_F("integral_limit:", &g_balance_integral_limit, 1.0f,   0.0f,   300.0f, 0, 2), /* 平衡控制积分限幅，可调；防止积分项长时间堆积 */ \
    PARAM_I("left_limit:",     &g_servo_left_limit,       10,     0,      10000,  0), /* 舵机左打机械极限占空比，可调；用于标定最大左转边界 */ \
    PARAM_I("mid_duty:",       &g_servo_mid_duty,         10,     0,      10000,  0), /* 舵机中位占空比，可调；用于校正机械零位和直行中心 */ \
    PARAM_I("right_limit:",    &g_servo_right_limit,      10,     0,      10000,  0), /* 舵机右打机械极限占空比，可调；用于标定最大右转边界 */ \
    PARAM_F("pwm_angle:",      &pwm_angle,                0,      0,          0,  1, 2)  /* 当前 PWM 对应等效转角，只读；用于观察输出是否接近限位 */


/*======================GPS页面相关参数=========================*/
#define GPS_PARAM_LIST \
    PARAM_U8("second:",      &gnss.time.second,        0, 0, 0, 1), /* GNSS 时间秒字段，只读；用于确认定位帧是否持续刷新 */ \
    PARAM_U8("state:",       &gnss.state,              0, 0, 0, 1), /* GNSS 定位状态，只读；用于判断是否具备导航和勘线条件 */ \
    PARAM_U8("satellite:",   &gnss.satellite_used,     0, 0, 0, 1), /* 当前参与解算的卫星数量，只读；用于评估定位质量 */ \
    PARAM_D( "latitude:",    &g_gps_latitude_display,  0, 0, 0, 1, 6), /* 当前纬度显示值，只读；保留 6 位小数便于核对位置变化 */ \
    PARAM_D( "longitude:",   &g_gps_longitude_display, 0, 0, 0, 1, 6), /* 当前经度显示值，只读；保留 6 位小数便于核对位置变化 */ \
    PARAM_F( "speed:",       &gnss.speed,              0, 0, 0, 1, 1), /* GNSS 解算速度，单位 m/s，只读；用于与电机估算速度互相验证 */ \
    PARAM_F( "direction:",   &gnss.direction,          0, 0, 0, 1, 1), /* GNSS 航向角，单位度，只读；用于观察卫星解算的运动方向 */ \
    PARAM_F( "height:",      &gnss.height,             0, 0, 0, 1, 1)  /* GNSS 海拔高度，单位 m，只读；主要用于诊断数据完整性 */


/*======================IMU页面相关参数=========================*/
#define IMU_PARAM_LIST \
    PARAM_F("roll:",       &roll_kalman,                 0, 0, 0, 1, 2), /* 横滚角估计值，单位度，只读；用于观察车体左右倾斜姿态 */ \
    PARAM_F("pitch:",      &pitch_kalman,                0, 0, 0, 1, 2), /* 俯仰角估计值，单位度，只读；用于观察前后俯仰姿态 */ \
    PARAM_F("yaw:",        &yaw_kalman,                  0, 0, 0, 1, 2), /* 偏航角估计值，单位度，只读；用于观察航向累计变化 */ \
    PARAM_F("gyro_x:",     &gyro_x_rate,                 0, 0, 0, 1, 2), /* X 轴角速度，单位 deg/s，只读；用于观察横滚动态变化 */ \
    PARAM_F("gyro_y:",     &gyro_y_rate,                 0, 0, 0, 1, 2), /* Y 轴角速度，单位 deg/s，只读；用于观察俯仰动态变化 */ \
    PARAM_I("calibrated:", &gyro_calibrated,             0, 0, 0, 1), /* IMU 校准完成标志，只读；用于确认零偏补偿是否已经完成 */ \
    PARAM_F("roll_ctrl:",  &roll_ctrl_angle,             0, 0, 0, 1, 2), /* 控制用横滚角，单位度，只读；这是扣除机械零位后的真实控制输入 */ \
    PARAM_F("pitch_ctrl:", &pitch_ctrl_angle,            0, 0, 0, 1, 2), /* 控制用俯仰角，单位度，只读；用于观察进入控制环前的姿态量 */ \
    PARAM_F("pwm_angle:",  &pwm_angle,                   0, 0, 0, 1, 2), /* 当前舵机 PWM 映射角度，只读；用于观察姿态误差最终转成的舵量 */ \
    PARAM_I("drop_count:", &g_imu_diag_stat.drop_count,  0, 0, 0, 1)  /* IMU 数据丢包计数，只读；用于排查采样链是否异常 */



/*======================测试相关参数=========================*/
#define TEST_PARAM_LIST \
    PARAM_I("drop_count:", &g_imu_diag_stat.drop_count, 0, 0, 0, 1), /* IMU 丢包计数，只读；测试页用于快速检查采样稳定性 */ \
    PARAM_U32("5ms_last:", &g_ctrl_5ms_last_us,         0, 0, 0, 1), /* 最近一次 5ms 控制任务执行时间，单位 us，只读 */ \
    PARAM_U32("5ms_max:",  &g_ctrl_5ms_max_us,          0, 0, 0, 1), /* 5ms 控制任务最大执行时间，单位 us，只读；用于抓最坏时延 */ \
    PARAM_U32("5ms_min:",  &g_ctrl_5ms_min_us,          0, 0, 0, 1), /* 5ms 控制任务最小执行时间，单位 us，只读；用于观察时延波动范围 */ \
    PARAM_U32("5ms_cnt:",  &g_ctrl_5ms_run_count,       0, 0, 0, 1)  /* 5ms 控制任务累计执行次数，只读；用于确认调度是否持续运行 */


/*======================科目1相关参数=========================*/

#define SUBJ1_PARAM_LIST \
    PARAM_I ("State:",    &g_subject1_state,        0,      0,      0,      1), /* 科目1状态机当前状态，只读；用于确认待机、去程、减速、掉头或返程阶段 */ \
    PARAM_I ("Step:",     &s_survey_step,           0,      0,      0,      1), /* 科目1堪线步骤，堪线数==2即堪线完成 */ \
    PARAM_F ("Dist:",     &g_nav_dist_to_wp,        0,      0,      0,      1, 2), /* 当前距目标路点距离，单位 m，只读；用于观察是否进入制动区 */ \
    PARAM_F ("HdgErr:",   &g_nav_heading_error,     0,      0,      0,      1, 1), /* 当前航向误差，单位度，只读；掉头阶段重点看该值是否回到阈值内 */ \
    PARAM_F ("HIGH:",     &g_s1_high_rpm,           10.0f,  0.0f,   2000.0f, 0, 1), /* 科目1直线路段最高目标转速，单位 RPM，可调 */ \
    PARAM_F ("MID:",      &g_s1_mid_rpm,            10.0f,  0.0f,   2000.0f, 0, 1), /* 科目1预减速和终点制动的中间目标转速，单位 RPM，可调 */ \
    PARAM_F ("TURN:",     &g_s1_turn_rpm,           10.0f,  0.0f,   2000.0f, 0, 1), /* 科目1掉头阶段低速目标转速，单位 RPM，可调 */ \
    PARAM_F ("BrakeDst:", &g_s1_pre_brake_dist,     0.1f,   0.5f,   20.0f,   0, 1), /* 去程最后路点前预减速距离，单位 m，可调；距离越大越早开始降速 */ \
    PARAM_F ("FinDst:",   &g_s1_finish_brake_dist,  0.1f,   0.5f,   20.0f,   0, 1), /* 回程接近起点时的终点制动距离，单位 m，可调 */ \
    PARAM_F ("Resume:",   &g_s1_resume_thresh,      1.0f,   1.0f,   90.0f,   0, 1), /* 掉头后恢复加速的航向误差阈值，单位度，可调 */ \
    PARAM_F ("Accel:",    &g_s1_accel_step,         5.0f,   0.0f,   300.0f,  0, 1)  /* 回程阶段每 100ms 的加速步进，单位 RPM，可调 */


/*======================科目2相关参数=========================*/
#define SUBJ2_PARAM_LIST \
    PARAM_I ("State:",    &g_subject2_state,        0,      0,      0,      1), /* 科目2状态机当前状态，只读；用于确认八字是否已启动或结束 */ \
    PARAM_F ("HdgErr:",   &g_nav_heading_error,     0,      0,      0,      1, 1), /* IMU 导航当前航向误差，单位度，只读；用于观察每段转向跟踪效果 */ \
    PARAM_F ("IMUhdg:",   &g_nav_imu_heading,       0,      0,      0,      1, 1), /* 当前 IMU 累计航向，单位度，只读；用于核对八字左右环绕是否正确 */ \
    PARAM_F ("Dist:",     &g_nav_distance_m,        0,      0,      0,      1, 2), /* 当前 IMU 累计里程，单位 m，只读；用于核对段长切换是否准确 */ \
    PARAM_F ("RPM:",      &g_s2_rpm,                10.0f,  50.0f,  800.0f,  0, 1), /* 科目2恒定运行转速，单位 RPM，可调 */ \
    PARAM_F ("Radius:",   &g_s2_turn_radius,        0.05f,  0.3f,   1.5f,    0, 2)  /* 八字等效转弯半径，单位 m，可调；会直接影响圆弧段长度 */


/*======================科目3相关参数=========================*/
#define SUBJ3_PARAM_LIST \
    PARAM_I ("State:",    &g_subject3_state,        0,      0,      0,      1), /* 科目3状态机当前状态，只读；用于确认去程、减速、掉头或返程阶段 */ \
    PARAM_I ("Step:",     &s3_survey_step,          0,      0,      0,      1), /* 科目3目前堪线路点数 */ \
    PARAM_F ("Dist:",     &g_nav_dist_to_wp,        0,      0,      0,      1, 2), /* 当前距正在导航路点的距离，单位 m，只读；用于判断何时触发最后路点减速 */ \
    PARAM_F ("HdgErr:",   &g_nav_heading_error,     0,      0,      0,      1, 1), /* 当前航向误差，单位度，只读；用于掉头完成判定与返程对准观察 */ \
    PARAM_F ("HIGH:",     &g_s3_go_rpm,             10.0f,  0.0f,   2000.0f, 0, 1), /* 科目3巡航最高转速，单位 RPM，可调 */ \
    PARAM_F ("MID:",      &g_s3_mid_rpm,            10.0f,  0.0f,   2000.0f, 0, 1), /* 科目3预减速与终点制动时使用的中速转速，单位 RPM，可调 */ \
    PARAM_F ("TURN:",     &g_s3_turn_rpm,           10.0f,  0.0f,   2000.0f, 0, 1), /* 科目3掉头阶段低速转速，单位 RPM，可调 */ \
    PARAM_F ("BrakeDst:", &g_s3_pre_brake_dist,     0.1f,   0.5f,   20.0f,   0, 1), /* 科目3最后去程路点前预减速距离，单位 m，可调 */ \
    PARAM_F ("FinDst:",   &g_s3_finish_brake_dist,  0.1f,   0.5f,   20.0f,   0, 1), /* 科目3返程接近起点时的终点制动距离，单位 m，可调 */ \
    PARAM_F ("Resume:",   &g_s3_resume_thresh,      1.0f,   1.0f,   90.0f,   0, 1), /* 科目3掉头后恢复加速的航向误差阈值，单位度，可调 */ \
    PARAM_F ("Accel:",    &g_s3_accel_step,         5.0f,   0.0f,   300.0f,  0, 1)  /* 科目3返程每 100ms 的加速步进，单位 RPM，可调 */

#endif /* IPS_APP_CONFIG_H */
