/*
 * 文件: subject3_app.c
 * 功能: 科目3（颠簸路段往返）状态机实现
 *
 * 状态机：
 *   IDLE → GO(多路点) → GO_BRAKE → UTURN → RETURN(反向多路点) → DONE
 *   任意状态 + KEY3长按 → IDLE（紧急停止）
 *
 * 数据隔离：
 *   科目3路点存储在本模块内部数组（s3_waypoints[]），独立于 nav_app 和科目1。
 *   Flash 使用 page 10（科目1使用 page 11），互不干扰。
 *   启动比赛时通过 nav_import_gps_config() 将路点拷入 nav_app 运行。
 *
 * 作者: 闫锦
 * 日期: 2026-04-06
 */

#include "subject3_app.h"
#include "nav_app.h"
#include "motor_pid.h"
#include "servo_pid.h"
#include "lqr_balance.h"

#include <math.h>    /* fabsf */
#include <string.h>  /* memcpy */

/* ================================================================
 * 全局状态
 * ================================================================ */
/* 科目3状态机当前状态，用于区分待机、去程、减速、掉头、返程和完成。 */
subject3_state_t g_subject3_state = SUBJ3_STATE_IDLE;
/* 科目3多路点巡航阶段的最高目标转速。 */
volatile float g_s3_go_rpm            = SUBJ3_GO_RPM;
/* 科目3进入预减速区和终点制动区时使用的中速目标转速。 */
volatile float g_s3_mid_rpm           = SUBJ3_MID_RPM;
/* 科目3掉头阶段保持的低速目标转速。 */
volatile float g_s3_turn_rpm          = SUBJ3_TURN_RPM;
/* 最后一个去程路点前触发预减速的距离阈值，单位 m。 */
volatile float g_s3_pre_brake_dist    = SUBJ3_PRE_BRAKE_DIST_M;
/* 回到起点前触发终点制动的距离阈值，单位 m。 */
volatile float g_s3_finish_brake_dist = SUBJ3_FINISH_BRAKE_DIST_M;
/* 掉头完成后允许恢复加速的航向误差阈值，单位度。 */
volatile float g_s3_resume_thresh     = SUBJ3_RESUME_THRESH_DEG;
/* 返程阶段每个 100ms 调度周期的加速步进，单位 RPM。 */
volatile float g_s3_accel_step        = SUBJ3_ACCEL_STEP_RPM;

/* ================================================================
 * 内部路点存储（与 nav_app / 科目1 完全隔离）
 * ================================================================ */
/* 科目3是否已经记录有效的起点参考。 */
static uint8_t        s3_has_ref       = 0u;
/* 科目3专属起点参考纬度。 */
static double         s3_ref_lat       = 0.0;
/* 科目3专属起点参考经度。 */
static double         s3_ref_lon       = 0.0;
/* 科目3专属勘线路点数组，与 nav_app 工作路点和科目1路线相互隔离。 */
static nav_waypoint_t s3_waypoints[NAV_MAX_WAYPOINTS];
/* 当前已经保存的科目3有效路点数量。 */
static uint8_t        s3_wp_count      = 0u;

/* 勘线步骤：0=未开始, 1=ref_start已记录, >=2 表示已记录 (step-1) 个路点 */
uint8_t        s3_survey_step   = 0u;

/* ================================================================
 * 内部 Flash 存储结构（page 10）
 * ================================================================ */
typedef struct
{
    uint32_t magic;            /* 魔数，用于判断该页是否存放有效的科目3配置。 */
    uint8_t  has_ref;          /* 是否存在有效起点参考。 */
    uint8_t  wp_count;         /* 已保存的有效路点数量。 */
    uint8_t  _pad[2];          /* 对齐填充，保证后续字段按 32bit 边界存放。 */
    uint32_t ref_lat_raw[2];   /* 起点纬度的原始二进制镜像，double 通过 memcpy 拆成 2 个 uint32 保存。 */
    uint32_t ref_lon_raw[2];   /* 起点经度的原始二进制镜像。 */
    struct
    {
        uint32_t lat_raw[2];   /* 单个路点纬度的原始二进制镜像。 */
        uint32_t lon_raw[2];   /* 单个路点经度的原始二进制镜像。 */
        float    target_speed; /* 路点附带目标速度；科目3当前统一写 0，由状态机自主管速度。 */
        float    accept_radius;/* 路点到达判定半径，单位 m。 */
    } waypoints[NAV_MAX_WAYPOINTS];
    uint32_t checksum;         /* 除 magic 和自身外的累加校验值。 */
} subject3_flash_t;

/* ================================================================
 * Flash 工具函数
 * ================================================================ */

static uint32_t s3_compute_checksum(const subject3_flash_t *cfg)
{
    const uint32_t *p = (const uint32_t *)(const void *)cfg;
    uint32_t n   = sizeof(subject3_flash_t) / sizeof(uint32_t);
    uint32_t sum = 0u;
    uint32_t i;
    /* 跳过首个 magic 字，最后一个 checksum 字段也不参与累加。 */
    for (i = 1u; i < n - 1u; i++) sum += p[i];
    return sum;
}

static void s3_flash_save(void)
{
    subject3_flash_t cfg;
    uint8_t i;

    cfg.magic    = SUBJ3_FLASH_MAGIC;
    cfg.has_ref  = s3_has_ref;
    cfg.wp_count = s3_wp_count;
    cfg._pad[0]  = 0u; cfg._pad[1] = 0u;

    /* double 不能直接按整数页写入，先拷贝为原始 32bit 字镜像。 */
    memcpy(cfg.ref_lat_raw, &s3_ref_lat, sizeof(double));
    memcpy(cfg.ref_lon_raw, &s3_ref_lon, sizeof(double));

    for (i = 0u; i < s3_wp_count && i < NAV_MAX_WAYPOINTS; i++)
    {
        /* 把当前 RAM 中的每个路点编码到 Flash 镜像结构。 */
        memcpy(cfg.waypoints[i].lat_raw, &s3_waypoints[i].lat, sizeof(double));
        memcpy(cfg.waypoints[i].lon_raw, &s3_waypoints[i].lon, sizeof(double));
        cfg.waypoints[i].target_speed  = s3_waypoints[i].target_speed;
        cfg.waypoints[i].accept_radius = s3_waypoints[i].accept_radius;
    }
    /* 清零未使用的槽 */
    for (i = s3_wp_count; i < NAV_MAX_WAYPOINTS; i++)
    {
        cfg.waypoints[i].lat_raw[0] = 0u; cfg.waypoints[i].lat_raw[1] = 0u;
        cfg.waypoints[i].lon_raw[0] = 0u; cfg.waypoints[i].lon_raw[1] = 0u;
        cfg.waypoints[i].target_speed  = 0.0f;
        cfg.waypoints[i].accept_radius = 0.0f;
    }

    cfg.checksum = s3_compute_checksum(&cfg);

    /* 先清缓冲区再装载镜像，防止上次写入留下的无效数据污染当前页。 */
    flash_buffer_clear();
    memcpy(flash_union_buffer, &cfg, sizeof(subject3_flash_t));
    flash_erase_page(SUBJ3_FLASH_SECTOR, SUBJ3_FLASH_PAGE);
    flash_write_page_from_buffer(SUBJ3_FLASH_SECTOR, SUBJ3_FLASH_PAGE);

    printf("[SUBJ3-FLASH] Saved: has_ref=%u, wp=%u\r\n",
           (unsigned)s3_has_ref, (unsigned)s3_wp_count);
}

static uint8_t s3_flash_load(void)
{
    subject3_flash_t cfg;
    uint8_t i;

    /* 先把整页读到统一缓冲区，再解析成科目3本地结构。 */
    flash_read_page_to_buffer(SUBJ3_FLASH_SECTOR, SUBJ3_FLASH_PAGE);
    memcpy(&cfg, flash_union_buffer, sizeof(subject3_flash_t));

    if (cfg.magic != SUBJ3_FLASH_MAGIC)
    {
        printf("[SUBJ3-FLASH] No valid data (page %u)\r\n", SUBJ3_FLASH_PAGE);
        return 0u;
    }
    if (cfg.checksum != s3_compute_checksum(&cfg))
    {
        printf("[SUBJ3-FLASH] Checksum mismatch\r\n");
        return 0u;
    }

    s3_has_ref  = cfg.has_ref;
    s3_wp_count = (cfg.wp_count <= NAV_MAX_WAYPOINTS) ? cfg.wp_count : NAV_MAX_WAYPOINTS;

    /* 从 Flash 镜像恢复起点参考。 */
    memcpy(&s3_ref_lat, cfg.ref_lat_raw, sizeof(double));
    memcpy(&s3_ref_lon, cfg.ref_lon_raw, sizeof(double));

    for (i = 0u; i < s3_wp_count; i++)
    {
        /* 只恢复有效路点数量范围内的数据，未使用槽位保持忽略。 */
        memcpy(&s3_waypoints[i].lat, cfg.waypoints[i].lat_raw, sizeof(double));
        memcpy(&s3_waypoints[i].lon, cfg.waypoints[i].lon_raw, sizeof(double));
        s3_waypoints[i].target_speed  = cfg.waypoints[i].target_speed;
        s3_waypoints[i].accept_radius = cfg.waypoints[i].accept_radius;
    }

    printf("[SUBJ3-FLASH] Loaded: has_ref=%u, wp=%u\r\n",
           (unsigned)s3_has_ref, (unsigned)s3_wp_count);
    return 1u;
}

/* ================================================================
 * 内部辅助：将科目3路点拷入 nav_app 供导航使用
 * ================================================================ */
static void s3_load_waypoints_to_nav(void)
{
    /* 比赛启动前把科目3私有路线装入 nav_app 工作区供导航模块实际运行。 */
    nav_import_gps_config(s3_has_ref, s3_ref_lat, s3_ref_lon,
                          s3_wp_count, s3_waypoints);
}

/* ================================================================
 * 公开 API
 * ================================================================ */

void subject3_app_init(void)
{
    g_subject3_state = SUBJ3_STATE_IDLE;

    /* 从 Flash page 10 恢复科目3专属路点 */
    s3_flash_load();

    /* 推断勘线进度 */
    if (s3_has_ref && s3_wp_count > 0u)
    {
        /* 已有起点和至少一个路点，说明本次上电可以直接进入可启动状态。 */
        s3_survey_step = 1u + s3_wp_count;   /* ref + N 个路点 */
    }
    else if (s3_has_ref)
    {
        /* 只有起点没有路点，说明还处在勘线未完成状态。 */
        s3_survey_step = 1u;
    }
    else
    {
        /* Flash 中没有有效科目3路线时，从零开始勘线。 */
        s3_survey_step = 0u;
    }
}

/**
 * @brief KEY4 长按（科目3模式下）：多路点勘线
 * @details
 *   第1次：记录 ref_start（发车区 GPS）
 *   第2次起：每按一次记录一个路点（坡道/草地/锥桶间隙/掉头区）
 *   记录第2个路点起每次自动 Flash 保存
 */
void subject3_survey(void)
{
    if (g_subject3_state != SUBJ3_STATE_IDLE)
    {
        printf("[SUBJ3] Survey blocked: race running\r\n");
        return;
    }

    if (s3_survey_step == 0u)
    {
        /* 第1次：记录 ref_start */
        if (gnss.state == 0)
        {
            printf("[SUBJ3] Survey 1: GPS invalid\r\n");
            return;
        }
        {
            uint32 irq = interrupt_global_disable();
            /* 读 GNSS 双精度坐标时进入临界区，避免中断更新导致经纬度撕裂。 */
            s3_ref_lat = gnss.latitude;
            s3_ref_lon = gnss.longitude;
            interrupt_global_enable(irq);
        }
        s3_has_ref = 1u;
        s3_survey_step = 1u;
        printf("[SUBJ3] Survey: ref_start recorded (lat=%.7f lon=%.7f)\r\n",
               s3_ref_lat, s3_ref_lon);
    }
    else
    {
        /* 第2次起：记录路点 */
        if (gnss.state == 0)
        {
            printf("[SUBJ3] Survey WP: GPS invalid\r\n");
            return;
        }
        if (s3_wp_count >= NAV_MAX_WAYPOINTS)
        {
            printf("[SUBJ3] Survey WP: buffer full (%u max)\r\n", NAV_MAX_WAYPOINTS);
            return;
        }

        {
            uint32 irq = interrupt_global_disable();
            /* 路点经纬度同样在临界区中采样，保证一组坐标来自同一帧数据。 */
            s3_waypoints[s3_wp_count].lat = gnss.latitude;
            s3_waypoints[s3_wp_count].lon = gnss.longitude;
            interrupt_global_enable(irq);
        }
        s3_waypoints[s3_wp_count].target_speed  = 0.0f;  /* subject3 自管理速度 */
        s3_waypoints[s3_wp_count].accept_radius = SUBJ3_WP_RADIUS;
        s3_wp_count++;
        s3_survey_step = 1u + s3_wp_count;

        /* 自动保存（至少有 ref_start + 1 个路点后每次都存） */
        s3_flash_save();

        printf("[SUBJ3] Survey: WP#%u recorded (lat=%.7f lon=%.7f). Total=%u\r\n",
               s3_wp_count, s3_waypoints[s3_wp_count - 1u].lat,
               s3_waypoints[s3_wp_count - 1u].lon, s3_wp_count);
    }
}

/**
 * @brief KEY3 长按（科目3模式下）：启动
 */
void subject3_start(void)
{
    if (g_subject3_state != SUBJ3_STATE_IDLE && g_subject3_state != SUBJ3_STATE_DONE)
    {
        printf("[SUBJ3] Already running\r\n");
        return;
    }
    if (s3_wp_count < 1u || !s3_has_ref)
    {
        printf("[SUBJ3] Survey incomplete: ref=%u, wp=%u\r\n",
               (unsigned)s3_has_ref, (unsigned)s3_wp_count);
        return;
    }

    /* 将科目3专属路点拷入 nav_app（会覆盖科目1数据，比赛时只运行一个科目） */
    s3_load_waypoints_to_nav();

    /* 启动 GPS 导航（在起点计算漂移修正量）*/
    nav_start_gps();

    if (g_nav_state != NAV_STATE_NAVIGATING)
    {
        printf("[SUBJ3] Start FAILED: nav not running\r\n");
        return;
    }

    /* 起跑成功后先以巡航高速进入多路点去程。 */
    motor_set_target_rpm(g_s3_go_rpm);
    /* 统一启停框架：仅在科目3确实启动成功后放行电机输出。 */
    motor_output_set_enable(1u);
    g_subject3_state = SUBJ3_STATE_GO;
    printf("[SUBJ3] START! GO → %u waypoints @ %.0f RPM\r\n",
           s3_wp_count, g_s3_go_rpm);
}

/**
 * @brief 紧急停止
 */
void subject3_stop(void)
{
    /* 紧急停止只清运行状态，不擦除已经勘线并持久化的科目3路线。 */
    nav_stop();
    motor_set_target_rpm(0.0f);
    motor_output_set_enable(0u);
    balance_set_expect_angle(0.0f);
    lqr_set_expect_phi(0.0f);
    g_subject3_state = SUBJ3_STATE_IDLE;
    printf("[SUBJ3] STOP! Emergency.\r\n");
}

/**
 * @brief 科目3调度任务（100ms）
 *
 * GO       — 多路点导航，全速；仅在最后路点前触发 GO_BRAKE
 * GO_BRAKE — 线性减速到掉头速
 * UTURN    — 低速掉头，航向对齐后加速
 * RETURN   — 反向多路点导航，渐进加速，终点制动
 */
void subject3_task(void)
{
    switch (g_subject3_state)
    {
        /* -------------------------------------------------------- */
        case SUBJ3_STATE_GO:
        /* -------------------------------------------------------- */
        {
            /* 判断是否正在导航最后一个路点（掉头点） */
            uint8_t cur_idx   = nav_get_current_wp_index();
            uint8_t total_wps = nav_get_waypoint_count();

            if (cur_idx == total_wps - 1u)
            {
                /* 正在奔向最后路点（掉头点），检查是否进入预减速区 */
                if (g_nav_dist_to_wp < g_s3_pre_brake_dist && g_nav_dist_to_wp > 0.1f)
                {
                    /* 中间路点不减速，只有最后一个去程路点前才转入减速状态。 */
                    motor_set_target_rpm(g_s3_mid_rpm);
                    g_subject3_state = SUBJ3_STATE_GO_BRAKE;
                    printf("[SUBJ3] → GO_BRAKE @ dist=%.1fm (last WP)\r\n", g_nav_dist_to_wp);
                }
            }
            /* 中间路点自动由 nav_app 导航到下一个，无需 subject3 干预 */
            break;
        }

        /* -------------------------------------------------------- */
        case SUBJ3_STATE_GO_BRAKE:
        /* -------------------------------------------------------- */
        {
            /* 线性减速（与科目1相同逻辑）*/
            float frac = g_nav_dist_to_wp / g_s3_pre_brake_dist;
            if (frac < 0.0f) frac = 0.0f;
            if (frac > 1.0f) frac = 1.0f;
            /* 按剩余距离比例把速度从 MID 平滑收敛到 TURN，减少掉头点冲击。 */
            float rpm = g_s3_turn_rpm + (g_s3_mid_rpm - g_s3_turn_rpm) * frac;
            motor_set_target_rpm(rpm);

            /* 到达掉头路点 → 准备反向回程 */
            if (g_nav_state == NAV_STATE_DONE)
            {
                /* nav_prepare_return_path 将路点反转 + 追加 ref_start */
                nav_prepare_return_path(SUBJ3_WP_RADIUS);

                /* 保留去程漂移修正量启动回程导航 */
                nav_start_gps_keep_drift();

                if (g_nav_state != NAV_STATE_NAVIGATING)
                {
                    printf("[SUBJ3] GPS return FAILED\r\n");
                    subject3_stop();
                    break;
                }

                /* 回程导航准备完成后先维持低速掉头，等待航向重新对准返程方向。 */
                motor_set_target_rpm(g_s3_turn_rpm);
                g_subject3_state = SUBJ3_STATE_UTURN;
                printf("[SUBJ3] → UTURN @ %.0f RPM\r\n", g_s3_turn_rpm);
            }
            break;
        }

        /* -------------------------------------------------------- */
        case SUBJ3_STATE_UTURN:
        /* -------------------------------------------------------- */
            motor_set_target_rpm(g_s3_turn_rpm);

            if (fabsf(g_nav_heading_error) < g_s3_resume_thresh)
            {
                /* 航向回到允许范围后先以中速进入返程，再逐步加速回巡航速度。 */
                motor_set_target_rpm(g_s3_mid_rpm);
                g_subject3_state = SUBJ3_STATE_RETURN;
                printf("[SUBJ3] → RETURN, heading_err=%.1f°\r\n", g_nav_heading_error);
            }
            break;

        /* -------------------------------------------------------- */
        case SUBJ3_STATE_RETURN:
        /* -------------------------------------------------------- */
            /* 终点制动区 */
            if (g_nav_dist_to_wp < g_s3_finish_brake_dist && g_nav_dist_to_wp > 0.1f)
            {
                /* 检查是否在导航最后路点（ref_start） */
                uint8_t cur_idx   = nav_get_current_wp_index();
                uint8_t total_wps = nav_get_waypoint_count();
                if (cur_idx == total_wps - 1u)
                {
                    motor_set_target_rpm(g_s3_mid_rpm);
                }
            }
            else
            {
                /* 渐进加速 */
                float cur = (float)target_motor_rpm;
                if (cur < g_s3_go_rpm)
                {
                    /* 固定步进加速可减少掉头后瞬间满速造成的姿态扰动。 */
                    cur += g_s3_accel_step;
                    if (cur > g_s3_go_rpm) cur = g_s3_go_rpm;
                    motor_set_target_rpm(cur);
                }
            }

            /* 到达起点 → 停车 */
            if (g_nav_state == NAV_STATE_DONE)
            {
                /* 到达起点后停止导航和输出，但本模块保存的科目3路线仍继续保留。 */
                motor_set_target_rpm(0.0f);
                motor_output_set_enable(0u);
                nav_stop();
                balance_set_expect_angle(0.0f);
                lqr_set_expect_phi(0.0f);
                g_subject3_state = SUBJ3_STATE_DONE;
                printf("[SUBJ3] DONE! Return complete.\r\n");
            }
            break;

        case SUBJ3_STATE_IDLE:
        case SUBJ3_STATE_DONE:
        default:
            break;
    }
}
