/*
 * 文件: subject1_app.c
 * 功能: 科目1（高速直线往返）状态机实现
 *
 * 状态机（[升级] 5段分速控制）：
 *   IDLE
 *     ↓ KEY3长按
 *   GO（高速直行，GPS）
 *     ↓ g_nav_dist_to_wp < PRE_BRAKE_DIST
 *   GO_BRAKE（线性减速，GPS）
 *     ↓ g_nav_state == DONE（到达掉头路点）
 *   UTURN（低速掉头，GPS指向起点）
 *     ↓ |heading_error| < RESUME_THRESH
 *   RETURN（渐进加速→全速，终点制动，GPS）
 *     ↓ g_nav_state == DONE（到达起点）
 *   DONE（停车）
 *
 *   任意活跃状态 + KEY3长按 → IDLE（紧急停止）
 *
 * 升级说明（与旧版对比）：
 *   旧版：GO → RETURN，仅靠 heading_error 阈值两档切速（HIGH/TURN），粗暴
 *   新版：增加 GO_BRAKE（距离触发预减速）、UTURN（恒低速）、RETURN 渐进加速
 *         避免高速冲过路点、出弯突然满速两大失稳风险
 *
 * 作者: 闫锦
 * 日期: 2026-04-06
 */

#include "subject1_app.h"
#include "nav_app.h"
#include "motor_pid.h"
#include "servo_pid.h"
#include "lqr_balance.h"

#include "config_flash.h"                           //堪线完成自动进行路点保存

#include <math.h>   /* fabsf */

/* ================================================================
 * 科目1关键全局变量
 * ================================================================ */
subject1_state_t g_subject1_state = SUBJ1_STATE_IDLE;               //科目1目前，状态，分别为五个活跃状态
volatile float g_s1_high_rpm          = SUBJ1_HIGH_RPM;             //去程和回程直线段最高的目标速度
volatile float g_s1_mid_rpm           = SUBJ1_MID_RPM;              //去程预减速和回程直线段的最高目标速度
volatile float g_s1_turn_rpm          = SUBJ1_TURN_RPM;             //掉头阶段保持的低速目标转速
volatile float g_s1_pre_brake_dist    = SUBJ1_PRE_BRAKE_DIST_M;     //接近掉头鲁甸时触发预减速的距离阈值
volatile float g_s1_finish_brake_dist = SUBJ1_FINISH_BRAKE_DIST_M;  //接近发车区终点时触发预减速的的距离阈值
volatile float g_s1_resume_thresh     = SUBJ1_RESUME_THRESH_DEG;    //掉头结束回复加速的航向误差阈值/角度
volatile float g_s1_accel_step        = SUBJ1_ACCEL_STEP_RPM;       //返程阶段每个 100ms 周期提高的转速步进，单位 RPM

/* ================================================================
 * 内部变量s_survey_step
 * ================================================================ */

/* 勘线步骤计数（0=未勘线, 1=已记录起点, 2=已记录路点，可以起跑）*/
uint8_t s_survey_step = 0u;

/* 科目1原始勘线路线备份：
 * nav_app 在去程结束后会把工作路点切换成“返回起点”单路点。
 * 为了保证同一上电周期内可以重复启动科目1，这里单独保存一份
 * “勘线/Flash恢复后的原始去程路线”，每次 start/stop/done 时按需恢复。 
 * */

static uint8_t        s1_has_ref = 0u;                  //备份路线是否包含有效起点参考
static double         s1_ref_lat = 0.0;                 //备份路线中的起点参考纬度。
static double         s1_ref_lon = 0.0;                 //备份路线中的起点参考经度。 

/* 备份的原始去程路点数组，避免 nav_app 在回程阶段覆盖工作路线后丢失原始勘线。 */
static nav_waypoint_t s1_waypoints[NAV_MAX_WAYPOINTS];



static uint8_t        s1_wp_count = 0u;                 /* 当前备份路点数量。 */

static void subject1_clear_route_backup(void)
{
    /* 清空科目1本地原始路线备份，供重新勘线时重新建立。 */
    s1_has_ref  = 0u;
    s1_ref_lat  = 0.0;
    s1_ref_lon  = 0.0;
    s1_wp_count = 0u;
}

//导出之前的勘测路线
static void subject1_snapshot_route_from_nav(void)
{
    /* 从 nav_app 工作区导出当前路线，保存为可重复起跑的原始副本。 */
    nav_export_gps_config(&s1_has_ref, &s1_ref_lat, &s1_ref_lon,
                          &s1_wp_count, s1_waypoints);
}

static uint8_t subject1_restore_route_to_nav(void)
{
    if (!s1_has_ref || s1_wp_count == 0u)
    {
        /* 没有有效备份时拒绝恢复，防止把无效路线写回导航模块。 */
        return 0u;
    }

    /* 把原始勘线路线重新装回 nav_app，保证 stop/done 后还能再次按原路线启动。 */
    nav_import_gps_config(s1_has_ref, s1_ref_lat, s1_ref_lon,
                          s1_wp_count, s1_waypoints);
    return 1u;
}


/* ================================================================
 * API 实现
 * ================================================================ */

/**
 * @brief 初始化科目1状态
 * @note  在 cpu0_main.c 中 config_flash_load() 之后调用，
 *        Flash 恢复的 GPS 路点信息可被 nav_has_ref_start() / nav_get_waypoint_count() 检测到
 */
void subject1_app_init(void)
{
    g_subject1_state = SUBJ1_STATE_IDLE;
    /* 根据 Flash 恢复的 GPS 状态自动推断勘线进度，避免每次上电重新勘线 */
    if (nav_has_ref_start() && nav_get_waypoint_count() > 0u)
    {
        /* 既有起点又有掉头路点，说明可以直接启动科目1。 */
        s_survey_step = 2u;
        subject1_snapshot_route_from_nav();
    }
    else if (nav_has_ref_start())
    {
        /* 只有起点没有掉头点，说明勘线只完成了一半。 */
        s_survey_step = 1u;
        subject1_snapshot_route_from_nav();
    }
    else
    {
        /* 没有任何历史勘线数据时，清空本地备份和进度。 */
        s_survey_step = 0u;
        subject1_clear_route_backup();
    }
}

/**
 * @brief KEY4 长按触发：勘线操作
 * @details
 *   第1次：记录发车区 GPS 参考起点（nav_set_ref_start）
 *   第2次：记录掉头区路点（nav_record_waypoint）+ 自动 Flash 保存
 *   第3次起：提示已完成
 */
void subject1_survey(void)
{
    if (g_subject1_state != SUBJ1_STATE_IDLE && g_subject1_state != SUBJ1_STATE_DONE)
    {
        printf("[SUBJ1] Survey blocked: race is running, stop first\r\n");
        return;
    }

    /* DONE 也视为“已结束、不在运行”。
       进入重新勘线流程前先统一回到 IDLE，避免界面显示仍停留在 DONE。 */
    if (g_subject1_state == SUBJ1_STATE_DONE)
    {
        g_subject1_state = SUBJ1_STATE_IDLE;
    }

    /* 若当前已存在完整勘线路线，则新的第一次 KEY4 长按视为“重新勘线开始”：
       清空旧工作路点和本地备份，然后立即把当前位置记录为新的 ref_start。 */
    if (s_survey_step >= 2u)
    {
        nav_clear_waypoints();
        subject1_clear_route_backup();
        s_survey_step = 0u;
        printf("[SUBJ1] Re-survey started: old route cleared, recording new ref_start.\r\n");
    }

    //勘测起点
    if (s_survey_step == 0u)
    {
        /* 第一次勘线先记录起点参考，为后续路点建立统一局部坐标基准。 */
        nav_set_ref_start();
        if (nav_has_ref_start())
        {
            /* 起点成功记录后立即同步备份，防止后续工作路点变化覆盖该信息。 */
            subject1_snapshot_route_from_nav();
            s_survey_step = 1u;
            printf("[SUBJ1] Survey 1/2: ref_start recorded. Move to turn-around point.\r\n");
        }
        else
        {
            printf("[SUBJ1] Survey 1/2 FAILED: GPS invalid\r\n");
        }
    }

    //勘测拐点
    else if (s_survey_step == 1u)
    {
        uint8_t wp_before = nav_get_waypoint_count();
        /* 第二次勘线记录掉头点，这也是去程导航的唯一目标路点。 */
        nav_record_waypoint(0.0f, SUBJ1_WP_RADIUS);
        if (nav_get_waypoint_count() > wp_before)
        {
            /* 路点录入成功后立即备份完整路线，供后续重复启动恢复。 */
            subject1_snapshot_route_from_nav();
            s_survey_step = 2u;
            /* 勘线完成，立即保存 GPS 路点 + 全部 PID 参数到 Flash */
            config_flash_save();
            printf("[SUBJ1] Survey 2/2: waypoint recorded. Config saved to flash. Ready!\r\n");
        }
        else
        {
            printf("[SUBJ1] Survey 2/2 FAILED: waypoint not recorded (GPS invalid?)\r\n");
        }
    }
    else
    {
        printf("[SUBJ1] Survey done (step=%u/2). KEY3 long-press to start race.\r\n",
               (unsigned)s_survey_step);
    }
}

/**
 * @brief KEY3 长按触发：启动科目1
 * @details 需已完成勘线（s_survey_step == 2）
 */
void subject1_start(void)
{
    if (g_subject1_state != SUBJ1_STATE_IDLE && g_subject1_state != SUBJ1_STATE_DONE)
    {
        printf("[SUBJ1] Already running\r\n");
        return;
    }
    if (s_survey_step < 2u)
    {
        printf("[SUBJ1] Survey incomplete (%u/2). KEY4 long-press to survey.\r\n",
               (unsigned)s_survey_step);
        return;
    }
    if (!subject1_restore_route_to_nav())
    {
        printf("[SUBJ1] Start FAILED: route backup missing, survey again.\r\n");
        return;
    }

    /* 起跑前先恢复原始去程路线，再让 nav_app 进入 GPS 导航状态。 */
    nav_start_gps();

    if (g_nav_state != NAV_STATE_NAVIGATING)
    {
        printf("[SUBJ1] Start FAILED: nav not running (GPS valid?)\r\n");
        return;
    }

    /* 启动成功后先以高速进入去程。 */
    motor_set_target_rpm(g_s1_high_rpm);
    /* 统一启停框架：只有 start 成功后才放行电机输出。 */
    motor_output_set_enable(1u);
    g_subject1_state = SUBJ1_STATE_GO;
    printf("[SUBJ1] START! GO @ %.0f RPM\r\n", g_s1_high_rpm);
}

/**
 * @brief KEY3 长按触发（运行中）：紧急停止
 */
void subject1_stop(void)
{
    /* 先停导航，避免恢复路线时 nav_app 仍在处理旧运行状态。 */
    nav_stop();
    /* 紧急停止也恢复原始路线，保证同一上电周期内还能再次起跑。 */
    (void)subject1_restore_route_to_nav();
    /* 同时清零电机和姿态目标，避免停下后残留控制量。 */
    motor_set_target_rpm(0.0f);
    motor_output_set_enable(0u);
    balance_set_expect_angle(0.0f);
    lqr_set_expect_phi(0.0f);
    g_subject1_state = SUBJ1_STATE_IDLE;
    printf("[SUBJ1] STOP! Emergency.\r\n");
}

/**
 * @brief 科目1调度任务（100ms 周期）
 *
 * [升级] 分段速度控制：
 *   GO       — 高速直行，到预减速距离触发 GO_BRAKE
 *   GO_BRAKE — 按距离线性降速（MID→TURN），到达路点切 UTURN
 *   UTURN    — 恒低速掉头，航向对齐后切 RETURN
 *   RETURN   — 渐进加速，终点制动区降速，GPS到达起点后停车
 *
 * @note 航向误差 → balance_set_expect_angle 在 isr.c 5ms ISR 中统一注入，
 *       本任务只管速度和状态转移，不直接操作舵机。
 */
void subject1_task(void)
{
    switch (g_subject1_state)
    {
        /* -------------------------------------------------------- */
        case SUBJ1_STATE_GO:
        /* -------------------------------------------------------- */
            /* 触发预减速：距掉头路点进入制动区 */
            if (g_nav_dist_to_wp < g_s1_pre_brake_dist && g_nav_dist_to_wp > 0.1f)
            {
                /* 立即切到中速，避免继续以全速冲入制动区 */
                motor_set_target_rpm(g_s1_mid_rpm);
                /* 切到 GO_BRAKE 后，后续每个周期都会按剩余距离继续线性降速。 */
                g_subject1_state = SUBJ1_STATE_GO_BRAKE;
                printf("[SUBJ1] → GO_BRAKE @ dist=%.1fm\r\n", g_nav_dist_to_wp);
            }
            break;

        /* -------------------------------------------------------- */
        case SUBJ1_STATE_GO_BRAKE:
        /* -------------------------------------------------------- */
            /* 线性减速：距路点越近转速越低
               rpm = TURN + (MID - TURN) × (dist / PRE_BRAKE_DIST)
               dist → 0 时 rpm → TURN_RPM                          */
            {
                float frac = g_nav_dist_to_wp / g_s1_pre_brake_dist;
                if (frac < 0.0f) { frac = 0.0f; }
                if (frac > 1.0f) { frac = 1.0f; }
                /* 按剩余距离比例把速度从 MID 连续降到 TURN，避免硬切到低速。 */
                float rpm = g_s1_turn_rpm + (g_s1_mid_rpm - g_s1_turn_rpm) * frac;
                motor_set_target_rpm(rpm);
            }

            /* 到达掉头路点 → 切换 GPS 回程（保留 GO 阶段漂移修正）*/
            if (g_nav_state == NAV_STATE_DONE)
            {
                /* 去程完成后清空工作路点，并改成“返回起点”这一条回程目标。 */
                nav_clear_waypoints();
                nav_add_waypoint_at_ref_start(SUBJ1_WP_RADIUS);
                /* [关键] keep_drift：保留 GO 阶段在起点算出的漂移修正量
                   若此处重新调 nav_start_gps()，会以掉头区位置重算漂移，
                   导致修正后的起点坐标偏差≈赛道长度，车辆永远无法到达。 */
                nav_start_gps_keep_drift();

                if (g_nav_state != NAV_STATE_NAVIGATING)
                {
                    printf("[SUBJ1] GPS return FAILED, emergency stop\r\n");
                    subject1_stop();
                    break;
                }

                /* 回程导航建立后先低速掉头，等待车头重新对准起点方向。 */
                motor_set_target_rpm(g_s1_turn_rpm);
                g_subject1_state = SUBJ1_STATE_UTURN;
                printf("[SUBJ1] → UTURN @ %.0f RPM\r\n", g_s1_turn_rpm);
            }
            break;

        /* -------------------------------------------------------- */
        case SUBJ1_STATE_UTURN:
        /* -------------------------------------------------------- */
            /* 恒低速掉头，等待 GPS heading_error 对准起点方向 */
            motor_set_target_rpm(g_s1_turn_rpm);

            if (fabsf(g_nav_heading_error) < g_s1_resume_thresh)
            {
                /* 航向已对准，出弯开始加速 */
                motor_set_target_rpm(g_s1_mid_rpm);
                /* 先以中速进入 RETURN，再由 RETURN 状态逐步推回高速。 */
                g_subject1_state = SUBJ1_STATE_RETURN;
                printf("[SUBJ1] → RETURN, heading_err=%.1f°\r\n", g_nav_heading_error);
            }
            break;

        /* -------------------------------------------------------- */
        case SUBJ1_STATE_RETURN:
        /* -------------------------------------------------------- */
            /* 终点制动区（距起点 < FINISH_BRAKE_DIST）：降中速准备停车 */
            if (g_nav_dist_to_wp < g_s1_finish_brake_dist && g_nav_dist_to_wp > 0.1f)
            {
                motor_set_target_rpm(g_s1_mid_rpm);
            }
            else
            {
                /* 非制动区：每 100ms 加速一步，直到高速 */
                float cur = (float)target_motor_rpm;
                if (cur < g_s1_high_rpm)
                {
                    /* 固定步进加速可减小掉头结束后突然满速带来的姿态冲击。 */
                    cur += g_s1_accel_step;
                    if (cur > g_s1_high_rpm) { cur = g_s1_high_rpm; }
                    motor_set_target_rpm(cur);
                }
            }

            /* GPS 到达起点 → 停车 */
            if (g_nav_state == NAV_STATE_DONE)
            {
                /* 回到起点后恢复原始路线，为下一次 start 继续使用最初勘线路径。 */
                motor_set_target_rpm(0.0f);
                motor_output_set_enable(0u);
                nav_stop();
                (void)subject1_restore_route_to_nav();
                balance_set_expect_angle(0.0f);
                lqr_set_expect_phi(0.0f);
                g_subject1_state = SUBJ1_STATE_DONE;
                printf("[SUBJ1] DONE! Return complete.\r\n");
            }
            break;

        case SUBJ1_STATE_IDLE:
        case SUBJ1_STATE_DONE:
        default:
            break;
    }
}
