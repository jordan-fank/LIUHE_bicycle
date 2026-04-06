/*
 * 文件: nav_app.c
 * 功能: GPS/IMU 融合导航应用层实现
 *
 * 依赖（只读，不修改）：
 *   gnss                 — 逐飞 GNSS 库，提供 gnss.latitude/longitude/direction/speed/state
 *   get_two_points_distance / get_two_points_azimuth — 逐飞库已实现，直接复用
 *   yaw_kalman           — IMU 应用层（imu_app.h）
 *   motor_speed_m_s      — 电机应用层（motor_app.h），用于里程积分
 *
 * 使用流程：
 *   [GPS 路点模式 — 题目一/三]
 *   1. 勘线：车停在起点 → nav_set_ref_start()
 *   2. 勘线：沿路行驶，在关键转折点调 nav_record_waypoint(speed, radius)
 *   3. 比赛：车放在起点 → nav_start_gps()（自动计算 GPS 漂移并修正）
 *   4. 控制层循环读 g_nav_heading_error，乘以 g_nav_heading_gain 得到 expect_angle
 *
 *   [IMU 航向模式 — 题目二绕八字]
 *   1. 调 nav_add_imu_segment(heading, dist, speed) 依次添加每段预期航向和里程
 *   2. 车就位后调 nav_start_imu()（自动重置偏航参考）
 *   3. 控制层读 g_nav_heading_error 驱动转向
 *
 * 注意：
 *   - nav_app_task() 仅做航向误差计算和状态转移，不直接写 expect_angle，
 *     保持与控制层解耦，两套控制算法（PID/LQR）均可复用此输出。
 *   - IMU yaw 无磁力计，长期漂移不可避免。IMU 模式适合 1~2 分钟内的短程应用。
 *
 * 作者: 闫锦
 * 日期: 2026-04-01
 */

#include "nav_app.h"
#include "imu_app.h"
#include "motor_app.h"

/* ================================================================
 * 输出变量定义
 * ================================================================ */
volatile float       g_nav_heading_error = 0.0f;   /* 航向误差（度，正=右转）*/
volatile float       g_nav_target_speed  = 0.0f;   /* 当前段目标速度（m/s）*/
volatile float       g_nav_imu_heading   = 0.0f;   /* IMU 积分航向（度）*/
volatile float       g_nav_distance_m    = 0.0f;   /* 当前段已行里程（m）*/
volatile nav_state_t g_nav_state         = NAV_STATE_IDLE;
volatile nav_mode_t  g_nav_mode          = NAV_MODE_IDLE;
volatile float       g_nav_heading_gain  = NAV_HEADING_GAIN_DEFAULT;
/* [新增 科目1分段速度] 当前目标路点距离，供 subject1_app 距离触发减速 */
volatile float       g_nav_dist_to_wp    = 0.0f;


/* ================================================================
 * 内部状态
 * ================================================================ */

/* --- GPS 路点 --- */
static nav_waypoint_t s_waypoints[NAV_MAX_WAYPOINTS];
static uint8_t        s_wp_count   = 0u;   /* 已记录路点数 */
static uint8_t        s_wp_current = 0u;   /* 当前目标路点索引 */

/* GPS 漂移修正
   原理：
     勘线时在同一物理起点记录 GPS 坐标（参考起点）。
     比赛时再次站在同一物理起点，计算当次 GPS 与参考起点的偏差，
     作为全局漂移量。由于比赛场地较小（<100m），可视漂移量空间均匀。
*/
static double  s_ref_start_lat = 0.0;   /* 参考起点纬度（勘线记录）*/
static double  s_ref_start_lon = 0.0;   /* 参考起点经度（勘线记录）*/
static uint8_t s_has_ref       = 0u;    /* 参考起点是否已记录 */
static double  s_drift_lat     = 0.0;   /* 纬度漂移量（比赛时计算）*/
static double  s_drift_lon     = 0.0;   /* 经度漂移量（比赛时计算）*/

/* --- IMU 航向段 --- */
static nav_imu_segment_t s_imu_segments[NAV_MAX_IMU_SEGMENTS];
static uint8_t           s_seg_count   = 0u;
static uint8_t           s_seg_current = 0u;

/* IMU 航向参考：调用 nav_reset_imu_heading() 时保存的 yaw_kalman 快照
   当前 IMU 相对航向 = yaw_kalman - s_yaw_ref                         */
static float s_yaw_ref = 0.0f;

/* [修复 M6] 梯形积分：保存上一次速度，减小 100ms 采样下的里程积分误差 */
static float s_last_speed_m_s = 0.0f;

/* [修复 H2] GPS→IMU 航向平滑过渡：
   高速时持续记录最后可信的 GPS 航向和对应的 yaw_kalman 快照，
   低速时用 last_gps_heading + (yaw_now - yaw_at_last_gps) 近似绝对航向。 */
static float s_last_gps_heading = 0.0f;
static float s_yaw_at_last_gps  = 0.0f;


/* ================================================================
 * 内部工具函数
 * ================================================================ */

/*
 * 角度包裹：将任意角度归一化到 (-180, +180]
 * 避免跨 0°/360° 边界时航向误差出现大幅跳变
 * [修复 M2] 边界改为 <= -180 以保证对称性
 */
static float nav_wrap_angle(float deg)
{
    while (deg >  180.0f)  { deg -= 360.0f; }
    while (deg <= -180.0f) { deg += 360.0f; }
    return deg;
}

/*
 * 获取漂移修正后的当前 GPS 坐标
 * 修正公式：corrected = gnss_reported - drift
 * drift 在 nav_start_gps() 时一次性计算（比赛起点 - 勘线参考起点）
 *
 * [修复 C1] gnss.latitude/longitude 是 double（64位），TC264D 是 32 位 MCU，
 *           读写 double 需要 2 次 32 位内存操作，UART3 RX ISR 可在中间更新 gnss，
 *           导致读到"新高32位 + 旧低32位"的混合值（坐标跳变数十公里）。
 *           因此所有读取 gnss 的位置必须加临界区保护。
 */
static void nav_get_corrected_pos(double *lat_out, double *lon_out)
{
    double lat, lon;
    uint32 irq_state = interrupt_global_disable();
    lat = gnss.latitude;
    lon = gnss.longitude;
    interrupt_global_enable(irq_state);
    *lat_out = lat - s_drift_lat;
    *lon_out = lon - s_drift_lon;
}


/* ================================================================
 * GPS 路点导航内部任务
 * ================================================================ */
static void nav_task_gps(void)
{
    double curr_lat, curr_lon;
    double dist_m, bearing_deg;
    float  cur_heading;
    float  heading_err;

    /* [修复 C1] 快照 gnss 关键字段，避免后续使用时被 ISR 撕裂
       gnss.state / speed / direction 是 float/uint8（32位以内，原子安全），
       但 state 检查与 lat/lon 读取必须来自同一时刻，否则可能"state 有效但坐标是旧帧"。 */
    uint8_t snap_state;
    float   snap_speed, snap_direction;
    {
        uint32 irq_state = interrupt_global_disable();
        snap_state     = gnss.state;
        snap_speed     = gnss.speed;
        snap_direction = gnss.direction;
        interrupt_global_enable(irq_state);
    }

    /* GPS 无效定位时跳过，保持上次误差不变，避免输出抖动 */
    if (snap_state == 0)
    {
        return;
    }

    nav_get_corrected_pos(&curr_lat, &curr_lon);

    /* 调用逐飞库函数 —— 无需自己实现 Haversine */
    dist_m     = get_two_points_distance(curr_lat, curr_lon,
                                         s_waypoints[s_wp_current].lat,
                                         s_waypoints[s_wp_current].lon);
    /* [新增 科目1分段速度] 实时暴露路点距离，供 subject1_app 触发预减速 */
    g_nav_dist_to_wp = (float)dist_m;
    bearing_deg = get_two_points_azimuth(curr_lat, curr_lon,
                                         s_waypoints[s_wp_current].lat,
                                         s_waypoints[s_wp_current].lon);

    /* [修复 H2] GPS→IMU 航向切换
       原实现：低速时直接用 yaw_kalman（相对偏航），但 gnss.direction 是绝对北向，
       两者坐标系不同，切换瞬间航向跳变。
       修复：高速时记录最后可信 GPS 航向和对应 yaw 快照，
       低速时用"最后可信航向 + IMU 变化量"，保证切换时连续。         */
    if (snap_speed >= NAV_GPS_SPEED_MIN_KMH)
    {
        cur_heading = snap_direction;
        /* 记录当前可信航向及其对应的 IMU yaw 快照 */
        s_last_gps_heading = snap_direction;
        s_yaw_at_last_gps  = yaw_kalman;
    }
    else
    {
        /* 绝对基准 + IMU 相对变化量 = 近似绝对航向（短期有效） */
        float yaw_delta = yaw_kalman - s_yaw_at_last_gps;
        cur_heading = s_last_gps_heading + yaw_delta;
    }

    /* 航向误差：目标方位 - 当前航向，包裹到 ±180° */
    heading_err = nav_wrap_angle((float)bearing_deg - cur_heading);

    g_nav_heading_error = heading_err;
    g_nav_target_speed  = s_waypoints[s_wp_current].target_speed;

    /* 同步更新 IMU 相对航向 */
    g_nav_imu_heading = yaw_kalman - s_yaw_ref;

    /* 判断是否到达当前路点 */
    if (dist_m < (double)s_waypoints[s_wp_current].accept_radius)
    {
        s_wp_current++;

        if (s_wp_current >= s_wp_count)
        {
            /* 所有路点已到达，导航完成 */
            g_nav_state         = NAV_STATE_DONE;
            g_nav_heading_error = 0.0f;
            g_nav_target_speed  = 0.0f;
            printf("[NAV] GPS navigation done\r\n");
        }
        else
        {
            printf("[NAV] -> Waypoint %u/%u\r\n", s_wp_current + 1u, s_wp_count);
        }
    }
}


/* ================================================================
 * IMU 航向段导航内部任务
 * ================================================================ */
static void nav_task_imu(void)
{
    float heading_err;

    /* 更新 IMU 积分航向（相对偏航变化量）*/
    g_nav_imu_heading = yaw_kalman - s_yaw_ref;

    /* [修复 M6] 里程梯形积分：(v_now + v_last) / 2 × dt
       相比矩形积分，加减速段误差减半（100ms 采样 100m 误差从 ~2m 降至 ~1m）*/
    g_nav_distance_m += (motor_speed_m_s + s_last_speed_m_s) * 0.5f * NAV_TASK_DT_S;
    s_last_speed_m_s = motor_speed_m_s;

    /* 航向误差：目标航向 - 当前相对偏航，包裹到 ±180° */
    heading_err = nav_wrap_angle(
        s_imu_segments[s_seg_current].target_heading - g_nav_imu_heading
    );

    g_nav_heading_error = heading_err;
    g_nav_target_speed  = s_imu_segments[s_seg_current].target_speed;

    /* 判断当前段是否完成（按里程切换）*/
    if (g_nav_distance_m >= s_imu_segments[s_seg_current].segment_dist_m)
    {
        s_seg_current++;
        g_nav_distance_m = 0.0f;   /* 重置段内里程 */

        if (s_seg_current >= s_seg_count)
        {
            /* 所有段完成 */
            g_nav_state         = NAV_STATE_DONE;
            g_nav_heading_error = 0.0f;
            g_nav_target_speed  = 0.0f;
            printf("[NAV] IMU navigation done\r\n");
        }
        else
        {
            printf("[NAV] IMU -> seg %u/%u target=%.1f deg\r\n",
                   s_seg_current + 1u, s_seg_count,
                   s_imu_segments[s_seg_current].target_heading);
        }
    }
}


/* ================================================================
 * 公开 API 实现
 * ================================================================ */

void nav_app_init(void)
{
    s_wp_count    = 0u;
    s_wp_current  = 0u;
    s_seg_count   = 0u;
    s_seg_current = 0u;
    s_has_ref     = 0u;
    s_drift_lat   = 0.0;
    s_drift_lon   = 0.0;
    s_yaw_ref            = 0.0f;
    s_last_gps_heading   = 0.0f;
    s_yaw_at_last_gps    = 0.0f;
    s_last_speed_m_s     = 0.0f;

    g_nav_heading_error = 0.0f;
    g_nav_target_speed  = 0.0f;
    g_nav_imu_heading   = 0.0f;
    g_nav_distance_m    = 0.0f;
    g_nav_dist_to_wp    = 0.0f;   /* [新增 科目1分段速度] */
    g_nav_state         = NAV_STATE_IDLE;
    g_nav_mode          = NAV_MODE_IDLE;
    g_nav_heading_gain  = NAV_HEADING_GAIN_DEFAULT;
}

void nav_app_task(void)
{
    if (g_nav_state != NAV_STATE_NAVIGATING)
    {
        return;  /* 非导航状态不做任何计算，零开销 */
    }

    if (g_nav_mode == NAV_MODE_GPS_WAYPOINT)
    {
        nav_task_gps();
    }
    else if (g_nav_mode == NAV_MODE_HEADING_IMU)
    {
        nav_task_imu();
    }
}

void nav_stop(void)
{
    g_nav_state         = NAV_STATE_IDLE;
    g_nav_mode          = NAV_MODE_IDLE;
    g_nav_heading_error = 0.0f;
    g_nav_target_speed  = 0.0f;
}


/* ---- GPS 路点模式 ---- */

void nav_set_ref_start(void)
{
    if (gnss.state == 0)
    {
        printf("[NAV] GPS invalid, ref start NOT recorded\r\n");
        return;
    }
    /* [修复 C1] 临界区保护 gnss double 读取 */
    {
        uint32 irq_state = interrupt_global_disable();
        s_ref_start_lat = gnss.latitude;
        s_ref_start_lon = gnss.longitude;
        interrupt_global_enable(irq_state);
    }
    s_has_ref       = 1u;
    printf("[NAV] Ref start recorded: lat=%.7f lon=%.7f\r\n",
           s_ref_start_lat, s_ref_start_lon);
}

void nav_record_waypoint(float target_speed_m_s, float accept_radius_m)
{
    if (gnss.state == 0)
    {
        printf("[NAV] GPS invalid, waypoint NOT recorded\r\n");
        return;
    }
    if (s_wp_count >= NAV_MAX_WAYPOINTS)
    {
        printf("[NAV] Waypoint buffer full (%u max)\r\n", NAV_MAX_WAYPOINTS);
        return;
    }

    /* [修复 C1] 临界区保护 gnss double 读取 */
    {
        uint32 irq_state = interrupt_global_disable();
        s_waypoints[s_wp_count].lat = gnss.latitude;
        s_waypoints[s_wp_count].lon = gnss.longitude;
        interrupt_global_enable(irq_state);
    }
    s_waypoints[s_wp_count].target_speed = target_speed_m_s;
    s_waypoints[s_wp_count].accept_radius =
        (accept_radius_m > 0.1f) ? accept_radius_m : NAV_ACCEPT_RADIUS_DEFAULT_M;
    s_wp_count++;
    g_nav_state = NAV_STATE_READY;

    printf("[NAV] WP#%u recorded: lat=%.7f lon=%.7f spd=%.1fm/s r=%.1fm\r\n",
           s_wp_count,
           s_waypoints[s_wp_count - 1u].lat,
           s_waypoints[s_wp_count - 1u].lon,
           target_speed_m_s,
           s_waypoints[s_wp_count - 1u].accept_radius);
}

void nav_clear_waypoints(void)
{
    s_wp_count   = 0u;
    s_wp_current = 0u;
    if (g_nav_mode == NAV_MODE_GPS_WAYPOINT)
    {
        nav_stop();
    }
}

void nav_start_gps(void)
{
    if (s_wp_count == 0u)
    {
        printf("[NAV] No waypoints, GPS start aborted\r\n");
        return;
    }
    if (gnss.state == 0)
    {
        printf("[NAV] GPS invalid, GPS start aborted\r\n");
        return;
    }

    /* 计算 GPS 漂移修正量 */
    if (s_has_ref)
    {
        /* [修复 C1] 临界区保护 gnss double 读取 */
        double curr_lat, curr_lon;
        {
            uint32 irq_state = interrupt_global_disable();
            curr_lat = gnss.latitude;
            curr_lon = gnss.longitude;
            interrupt_global_enable(irq_state);
        }
        s_drift_lat = curr_lat - s_ref_start_lat;
        s_drift_lon = curr_lon - s_ref_start_lon;
        printf("[NAV] Drift correction: dlat=%.8f dlon=%.8f\r\n",
               s_drift_lat, s_drift_lon);
    }
    else
    {
        s_drift_lat = 0.0;
        s_drift_lon = 0.0;
        printf("[NAV] No ref start, drift correction disabled\r\n");
    }

    /* 重置 IMU 航向参考（供低速段 fallback 使用）*/
    s_yaw_ref = yaw_kalman;
    g_nav_imu_heading = 0.0f;

    s_wp_current = 0u;
    g_nav_mode   = NAV_MODE_GPS_WAYPOINT;
    g_nav_state  = NAV_STATE_NAVIGATING;
    printf("[NAV] GPS navigation started, %u waypoints\r\n", s_wp_count);
}

uint8_t nav_has_ref_start(void)
{
    return s_has_ref;
}

uint8_t nav_get_waypoint_count(void)
{
    return s_wp_count;
}


/* [修复 GPS回程坐标系BUG] 保留漂移修正量，仅重置路点索引并启动导航
 *
 * 问题根因：
 *   nav_start_gps() 每次调用都以"当前所在位置"重算漂移：
 *     drift = current_gnss - s_ref_start
 *   GO阶段在起点调用：drift ≈ 1m（GPS当日系统误差，正确）
 *   RETURN阶段在掉头区调用：drift ≈ 赛道长度+1m（错误！）
 *   → 修正后的当前坐标在起点时偏差 ≈ 赛道长度，永远无法到达目标。
 *
 * 修复方案：
 *   回程路点已通过 nav_add_waypoint_at_ref_start() 设为勘线起点坐标，
 *   GO阶段的漂移修正量已经正确，直接沿用即可。
 *   本函数仅重置路点索引和 IMU 参考，不碰 s_drift_lat/s_drift_lon。
 */
void nav_start_gps_keep_drift(void)
{
    if (s_wp_count == 0u)
    {
        printf("[NAV] nav_start_gps_keep_drift: no waypoints\r\n");
        return;
    }
    if (gnss.state == 0)
    {
        printf("[NAV] nav_start_gps_keep_drift: GPS invalid\r\n");
        return;
    }

    /* 漂移修正量 s_drift_lat/lon 保持 GO 阶段算出的值，不重算 */

    /* 重置 IMU 航向参考（供低速段 GPS→IMU 融合使用）*/
    s_yaw_ref         = yaw_kalman;
    g_nav_imu_heading = 0.0f;

    /* 重置路点索引，从第0个路点开始导航 */
    s_wp_current = 0u;
    g_nav_mode   = NAV_MODE_GPS_WAYPOINT;
    g_nav_state  = NAV_STATE_NAVIGATING;
    printf("[NAV] GPS return started (drift kept): dlat=%.8f dlon=%.8f, wp=%u\r\n",
           s_drift_lat, s_drift_lon, s_wp_count);
}

/* [新增 科目1 GPS回程] 将参考起点追加为返回路点
 * 调用时机：GPS去程导航完成（g_nav_state==DONE）后，切换返回导航之前。
 * 内部直接使用 s_ref_start_lat/lon（调用 nav_set_ref_start 时保存），
 * 不读 gnss 结构体，无需临界区。
 */
void nav_add_waypoint_at_ref_start(float radius_m)
{
    if (!s_has_ref)
    {
        printf("[NAV] nav_add_waypoint_at_ref_start: ref_start not recorded\r\n");
        return;
    }
    if (s_wp_count >= NAV_MAX_WAYPOINTS)
    {
        printf("[NAV] nav_add_waypoint_at_ref_start: buffer full\r\n");
        return;
    }
    s_waypoints[s_wp_count].lat          = s_ref_start_lat;
    s_waypoints[s_wp_count].lon          = s_ref_start_lon;
    s_waypoints[s_wp_count].target_speed = 0.0f;  /* subject1 自管理速度 */
    s_waypoints[s_wp_count].accept_radius =
        (radius_m > 0.1f) ? radius_m : NAV_ACCEPT_RADIUS_DEFAULT_M;
    s_wp_count++;
    g_nav_state = NAV_STATE_READY;
    printf("[NAV] Return WP added at ref_start: lat=%.7f lon=%.7f r=%.1fm\r\n",
           s_ref_start_lat, s_ref_start_lon,
           s_waypoints[s_wp_count - 1u].accept_radius);
}

/* [新增 科目3] 将去程路点反转为回程路径
 *
 * 去程路点：[WP0, WP1, ..., WPN-1(掉头)]
 * 生成回程：[WPN-2, WPN-3, ..., WP0, ref_start]
 *
 * 原理：掉头点 WPN-1 已到达无需再导航；中间路点反序保证回程经过相同障碍间隙；
 * ref_start 追加为终点确保回到发车区。
 */
void nav_prepare_return_path(float return_wp_radius)
{
    nav_waypoint_t temp[NAV_MAX_WAYPOINTS];
    uint8_t go_count = s_wp_count;
    uint8_t i;

    if (go_count < 2u)
    {
        /* 只有掉头点（或0），直接导航回起点 */
        nav_clear_waypoints();
        nav_add_waypoint_at_ref_start(return_wp_radius);
        printf("[NAV] Return path: only ref_start (go_count=%u)\r\n", go_count);
        return;
    }

    /* 备份去程路点到临时数组 */
    for (i = 0u; i < go_count; i++)
    {
        temp[i] = s_waypoints[i];
    }

    /* 重建路点数组：跳过掉头点(go_count-1)，反转中间路点 */
    s_wp_count   = 0u;
    s_wp_current = 0u;

    for (i = 0u; i < go_count - 1u; i++)
    {
        uint8_t src = go_count - 2u - i;   /* go_count-2, go_count-3, ..., 0 */
        if (s_wp_count >= NAV_MAX_WAYPOINTS) break;
        s_waypoints[s_wp_count]               = temp[src];
        s_waypoints[s_wp_count].accept_radius = return_wp_radius;
        s_wp_count++;
    }

    /* 追加参考起点为终点 */
    if (s_has_ref && s_wp_count < NAV_MAX_WAYPOINTS)
    {
        s_waypoints[s_wp_count].lat          = s_ref_start_lat;
        s_waypoints[s_wp_count].lon          = s_ref_start_lon;
        s_waypoints[s_wp_count].target_speed = 0.0f;
        s_waypoints[s_wp_count].accept_radius = return_wp_radius;
        s_wp_count++;
    }

    g_nav_state = NAV_STATE_READY;
    printf("[NAV] Return path prepared: %u WPs (reversed + ref_start)\r\n", s_wp_count);
}

/* [新增 科目3] 查询当前路点索引 */
uint8_t nav_get_current_wp_index(void)
{
    return s_wp_current;
}

/* [新增 Flash持久化] 导出 GPS 私有状态，供 config_flash_save() 读取 */
void nav_export_gps_config(uint8_t *has_ref_out, double *ref_lat_out, double *ref_lon_out,
                            uint8_t *wp_count_out, nav_waypoint_t *waypoints_out)
{
    uint8_t i;
    *has_ref_out  = s_has_ref;
    *ref_lat_out  = s_ref_start_lat;
    *ref_lon_out  = s_ref_start_lon;
    *wp_count_out = s_wp_count;
    for (i = 0u; i < s_wp_count && i < NAV_MAX_WAYPOINTS; i++)
    {
        waypoints_out[i] = s_waypoints[i];
    }
}

/* [新增 Flash持久化] 从 Flash 恢复时导入 GPS 私有状态
 * 须在 nav_app_init() 之后调用，否则 init 会将此处写入的数据清零。
 */
void nav_import_gps_config(uint8_t has_ref, double ref_lat, double ref_lon,
                            uint8_t wp_count, const nav_waypoint_t *waypoints_in)
{
    uint8_t i;
    s_has_ref       = has_ref;
    s_ref_start_lat = ref_lat;
    s_ref_start_lon = ref_lon;
    s_wp_count      = (wp_count <= NAV_MAX_WAYPOINTS) ? wp_count : NAV_MAX_WAYPOINTS;
    s_wp_current    = 0u;
    for (i = 0u; i < s_wp_count; i++)
    {
        s_waypoints[i] = waypoints_in[i];
    }
    if (s_wp_count > 0u)
    {
        g_nav_state = NAV_STATE_READY;  /* 路点已就绪，等待 nav_start_gps() */
    }
    printf("[NAV] GPS config imported: has_ref=%u, wp_count=%u\r\n",
           (unsigned)has_ref, (unsigned)s_wp_count);
}

/* ---- IMU 航向模式 ---- */

void nav_reset_imu_heading(void)
{
    s_yaw_ref         = yaw_kalman;
    g_nav_imu_heading = 0.0f;
    g_nav_distance_m  = 0.0f;
    s_last_speed_m_s  = 0.0f;
    printf("[NAV] IMU heading reset, yaw_ref=%.2f deg\r\n", s_yaw_ref);
}

void nav_add_imu_segment(float target_heading_deg, float segment_dist_m, float target_speed_m_s)
{
    if (s_seg_count >= NAV_MAX_IMU_SEGMENTS)
    {
        printf("[NAV] IMU segment buffer full (%u max)\r\n", NAV_MAX_IMU_SEGMENTS);
        return;
    }
    s_imu_segments[s_seg_count].target_heading  = target_heading_deg;
    s_imu_segments[s_seg_count].segment_dist_m  = segment_dist_m;
    s_imu_segments[s_seg_count].target_speed    = target_speed_m_s;
    s_seg_count++;
    g_nav_state = NAV_STATE_READY;
}

void nav_clear_imu_segments(void)
{
    s_seg_count   = 0u;
    s_seg_current = 0u;
    if (g_nav_mode == NAV_MODE_HEADING_IMU)
    {
        nav_stop();
    }
}

void nav_start_imu(void)
{
    if (s_seg_count == 0u)
    {
        printf("[NAV] No IMU segments, IMU start aborted\r\n");
        return;
    }

    nav_reset_imu_heading();   /* 重置偏航参考和段里程 */

    s_seg_current = 0u;
    g_nav_mode    = NAV_MODE_HEADING_IMU;
    g_nav_state   = NAV_STATE_NAVIGATING;
    printf("[NAV] IMU navigation started, %u segments, first target=%.1f deg\r\n",
           s_seg_count, s_imu_segments[0].target_heading);
}
