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
/* [Flash持久化] 勘线完成后自动保存路点到 Flash */
#include "config_flash.h"

#include <math.h>   /* fabsf */

/* ================================================================
 * 全局状态
 * ================================================================ */
subject1_state_t g_subject1_state = SUBJ1_STATE_IDLE;
volatile float g_s1_high_rpm          = SUBJ1_HIGH_RPM;
volatile float g_s1_mid_rpm           = SUBJ1_MID_RPM;
volatile float g_s1_turn_rpm          = SUBJ1_TURN_RPM;
volatile float g_s1_pre_brake_dist    = SUBJ1_PRE_BRAKE_DIST_M;
volatile float g_s1_finish_brake_dist = SUBJ1_FINISH_BRAKE_DIST_M;
volatile float g_s1_resume_thresh     = SUBJ1_RESUME_THRESH_DEG;
volatile float g_s1_accel_step        = SUBJ1_ACCEL_STEP_RPM;

/* ================================================================
 * 内部变量
 * ================================================================ */

/* 勘线步骤计数（0=未勘线, 1=已记录起点, 2=已记录路点，可以起跑）*/
static uint8_t s_survey_step = 0u;


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
        s_survey_step = 2u;
    }
    else if (nav_has_ref_start())
    {
        s_survey_step = 1u;
    }
    else
    {
        s_survey_step = 0u;
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
    if (g_subject1_state != SUBJ1_STATE_IDLE)
    {
        printf("[SUBJ1] Survey blocked: race is running, stop first\r\n");
        return;
    }

    if (s_survey_step == 0u)
    {
        nav_set_ref_start();
        if (nav_has_ref_start())
        {
            s_survey_step = 1u;
            printf("[SUBJ1] Survey 1/2: ref_start recorded. Move to turn-around point.\r\n");
        }
        else
        {
            printf("[SUBJ1] Survey 1/2 FAILED: GPS invalid\r\n");
        }
    }
    else if (s_survey_step == 1u)
    {
        uint8_t wp_before = nav_get_waypoint_count();
        nav_record_waypoint(0.0f, SUBJ1_WP_RADIUS);
        if (nav_get_waypoint_count() > wp_before)
        {
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

    nav_start_gps();

    if (g_nav_state != NAV_STATE_NAVIGATING)
    {
        printf("[SUBJ1] Start FAILED: nav not running (GPS valid?)\r\n");
        return;
    }

    motor_set_target_rpm(g_s1_high_rpm);
    g_subject1_state = SUBJ1_STATE_GO;
    printf("[SUBJ1] START! GO @ %.0f RPM\r\n", g_s1_high_rpm);
}

/**
 * @brief KEY3 长按触发（运行中）：紧急停止
 */
void subject1_stop(void)
{
    nav_stop();
    motor_set_target_rpm(0.0f);
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
                float rpm = g_s1_turn_rpm + (g_s1_mid_rpm - g_s1_turn_rpm) * frac;
                motor_set_target_rpm(rpm);
            }

            /* 到达掉头路点 → 切换 GPS 回程（保留 GO 阶段漂移修正）*/
            if (g_nav_state == NAV_STATE_DONE)
            {
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
                    cur += g_s1_accel_step;
                    if (cur > g_s1_high_rpm) { cur = g_s1_high_rpm; }
                    motor_set_target_rpm(cur);
                }
            }

            /* GPS 到达起点 → 停车 */
            if (g_nav_state == NAV_STATE_DONE)
            {
                motor_set_target_rpm(0.0f);
                nav_stop();
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
