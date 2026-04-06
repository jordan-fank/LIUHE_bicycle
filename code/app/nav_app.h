/*
 * 文件: nav_app.h
 * 功能: GPS/IMU 融合导航应用层接口声明
 *
 *   题目一/三（高速直行 / 地形障碍）：记录勘线 GPS 路点，比赛时自动跟踪
 *   题目二（低速绕8字）：预设 IMU 航向段序列，陀螺仪偏航角跟踪
 *
 *   输出：g_nav_heading_error（航向误差，度）供控制层使用
 *     PID 模式：expect_angle   = g_nav_heading_error * g_nav_heading_gain;
 *     LQR 模式：lqr_expect_phi = g_nav_heading_error * g_nav_heading_gain;
 *
 * 作者: 闫锦
 * 日期: 2026-04-01
 */

#ifndef CODE_APP_NAV_APP_H_
#define CODE_APP_NAV_APP_H_

#include "zf_common_headfile.h"

/* ==================== 参数配置 ==================== */

/* 路点容量上限 */
#define NAV_MAX_WAYPOINTS       20U
#define NAV_MAX_IMU_SEGMENTS    10U

/* GPS 路点默认到达半径（m）——比赛场地较小，建议 1.5~3.0 m */
#define NAV_ACCEPT_RADIUS_DEFAULT_M  2.0f

/* 航向误差 → 期望侧倾角增益（初始值，可运行时通过无线调参覆盖）
   物理含义：heading_error = 10°，则 expect_angle ≈ 10 * 0.3 = 3°
   实际调参时根据车辆转向响应修正此值                              */
#define NAV_HEADING_GAIN_DEFAULT     0.3f

/* GPS 速度可信阈值（km/h）
   低于此值时 gnss.direction 抖动大，改用 IMU 偏航角作为当前航向参考  */
#define NAV_GPS_SPEED_MIN_KMH        2.0f

/* 导航任务调度周期（与 scheduler.c 注册的周期一致）*/
#define NAV_TASK_PERIOD_MS      100U
#define NAV_TASK_DT_S           0.1f    /* 任务 dt（s），用于里程积分 */


/* ==================== 数据结构 ==================== */

typedef enum
{
    NAV_MODE_IDLE         = 0,  /* 未激活 */
    NAV_MODE_HEADING_IMU  = 1,  /* IMU 航向段跟踪（题目二）*/
    NAV_MODE_GPS_WAYPOINT = 2,  /* GPS 路点导航（题目一、三）*/
} nav_mode_t;

typedef enum
{
    NAV_STATE_IDLE       = 0,   /* 空闲，无输出 */
    NAV_STATE_READY      = 1,   /* 路点/段已加载，等待启动 */
    NAV_STATE_NAVIGATING = 2,   /* 导航中，持续更新 g_nav_heading_error */
    NAV_STATE_DONE       = 3,   /* 已完成（末路点到达或末段走完）*/
} nav_state_t;

/* GPS 路点：记录勘线时的绝对坐标 */
typedef struct
{
    double lat;             /* 纬度（度）*/
    double lon;             /* 经度（度）*/
    float  target_speed;    /* 本段目标速度（m/s）*/
    float  accept_radius;   /* 到达判定半径（m）*/
} nav_waypoint_t;

/* IMU 航向段：预设一段路线的目标航向和里程 */
typedef struct
{
    float target_heading;   /* 目标航向（度，相对于 nav_reset_imu_heading() 时的零点）*/
    float segment_dist_m;   /* 本段预定里程（m），超过后切换到下一段 */
    float target_speed;     /* 本段目标速度（m/s）*/
} nav_imu_segment_t;


/* ==================== 输出变量（供控制层 extern 读取）==================== */

/* [主输出] 航向误差（度）：正 = 需要右转，负 = 需要左转
   控制层通过此变量驱动转向：
     expect_angle / lqr_expect_phi = g_nav_heading_error * g_nav_heading_gain  */
extern volatile float        g_nav_heading_error;

/* [主输出] 当前段目标速度（m/s），可由外层速度环使用 */
extern volatile float        g_nav_target_speed;

/* IMU 积分航向（度，相对于上次 nav_reset_imu_heading() 时的偏航角变化量）*/
extern volatile float        g_nav_imu_heading;

/* 当前 IMU 段已行驶里程（m）*/
extern volatile float        g_nav_distance_m;

/* [新增 科目1分段速度] 当前目标路点的实时距离（m）
   由 nav_task_gps() 每 100ms 更新，供 subject1_app 触发距离制动逻辑使用。
   非 GPS 导航状态时保持最后值，subject1_app 应在 NAVIGATING 状态下使用。 */
extern volatile float        g_nav_dist_to_wp;

/* 导航状态和模式 */
extern volatile nav_state_t  g_nav_state;
extern volatile nav_mode_t   g_nav_mode;

/* 航向增益（可通过无线调参在线修改）*/
extern volatile float        g_nav_heading_gain;


/* ==================== API ==================== */

/* ---- 基础 ---- */
void    nav_app_init    (void);   /* 初始化，在 cpu0_main.c 中调用 */
void    nav_app_task    (void);   /* 导航任务，100ms 周期，注册到 scheduler */
void    nav_stop        (void);   /* 停止导航，清零输出，回到 IDLE 状态 */

/* ---- GPS 路点模式 ---- */
/* 勘线时调用：记录当前 GPS 位置为参考起点（用于比赛时漂移修正）*/
void    nav_set_ref_start   (void);

/* 勘线时调用：将当前 GPS 位置记录为一个路点 */
void    nav_record_waypoint (float target_speed_m_s, float accept_radius_m);

/* 清空所有已记录路点 */
void    nav_clear_waypoints (void);

/* 比赛时调用：计算漂移修正量，启动 GPS 路点导航 */
void    nav_start_gps       (void);

/* [修复 GPS回程坐标系BUG] 保留已有漂移修正量，仅重置路点索引并启动导航
 * 用于科目1回程：若重新调用 nav_start_gps()，会以掉头区坐标重算漂移，
 * 导致修正量包含整段赛道距离，车辆永远无法到达起点。
 * 本函数跳过漂移计算，复用 GO 阶段在起点算出的正确漂移量。 */
void    nav_start_gps_keep_drift(void);

/* 查询当前是否已具备 GPS 勘线数据（参考起点 + 至少 1 个路点） */
uint8_t nav_has_ref_start   (void);
uint8_t nav_get_waypoint_count(void);

/* [新增 科目1 GPS回程] 将已记录的参考起点（ref_start）追加为下一个导航路点
   用于科目1去程完成后，动态添加"返回起点"路点，实现全程GPS导航 */
void    nav_add_waypoint_at_ref_start(float radius_m);

/* [新增 科目3] 将当前路点数组反转为回程路径：
   去程 [WP0..WPN-1(掉头)] → 回程 [WPN-2..WP0, ref_start]
   跳过最后一个路点（掉头点，已到达），反转中间路点，追加参考起点为终点 */
void    nav_prepare_return_path(float return_wp_radius);

/* [新增 科目3] 查询当前正在导航的路点索引（0-based）*/
uint8_t nav_get_current_wp_index(void);

/* [新增 Flash持久化] 导出 GPS 私有状态供 config_flash 保存到 Flash */
void    nav_export_gps_config(uint8_t *has_ref_out, double *ref_lat_out, double *ref_lon_out,
                               uint8_t *wp_count_out, nav_waypoint_t *waypoints_out);

/* [新增 Flash持久化] 从 Flash 恢复时导入 GPS 私有状态（nav_app_init 之后调用）*/
void    nav_import_gps_config(uint8_t has_ref, double ref_lat, double ref_lon,
                               uint8_t wp_count, const nav_waypoint_t *waypoints_in);

/* ---- IMU 航向模式 ---- */
/* 将当前 yaw_kalman 设为航向零点（调用 nav_start_imu 时自动执行，也可手动重置）*/
void    nav_reset_imu_heading   (void);

/* 添加一段 IMU 航向段（按调用顺序执行）*/
void    nav_add_imu_segment     (float target_heading_deg,
                                 float segment_dist_m,
                                 float target_speed_m_s);

/* 清空所有 IMU 段 */
void    nav_clear_imu_segments  (void);

/* 比赛时调用：重置航向零点，启动 IMU 段导航 */
void    nav_start_imu           (void);

#endif /* CODE_APP_NAV_APP_H_ */
