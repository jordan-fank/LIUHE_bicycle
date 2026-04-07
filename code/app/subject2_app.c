/*
 * 文件: subject2_app.c
 * 功能: 科目2（低速八字绕桩）状态机实现
 *
 * 状态机：
 *   IDLE → KEY3长按 → RUNNING → NAV_STATE_DONE → DONE
 *   任意状态 + KEY3长按 → IDLE（紧急停止）
 *
 * 核心逻辑：
 *   1. start() 配置 8 段 IMU 航向序列（已在 .h 中详述）
 *   2. 自动切换到 LOW_SPEED 控制模式（纯 P+I，低速大舵角更稳）
 *   3. 恒定 SUBJ2_RPM 跑完全程
 *   4. done/stop 时恢复原控制模式
 *
 * 不需要 GPS 勘线；运行时参数可通过全局配置 Flash 保存。
 * KEY4 在科目2模式下无功能（无勘线需求）。
 *
 * 作者: 闫锦
 * 日期: 2026-04-06
 */

#include "subject2_app.h"
#include "nav_app.h"
#include "motor_pid.h"
#include "servo_pid.h"
#include "lqr_balance.h"

/* ================================================================
 * 全局状态
 * ================================================================ */
/* 科目2状态机当前状态，用于区分待机、运行中和完成态。 */
subject2_state_t g_subject2_state = SUBJ2_STATE_IDLE;
/* 科目2八字绕桩全程保持的恒定目标转速。 */
volatile float g_s2_rpm         = SUBJ2_RPM;
/* 八字绕桩的等效转弯半径，决定每个 120 度圆弧段的距离。 */
volatile float g_s2_turn_radius = SUBJ2_TURN_RADIUS_M;

/* ================================================================
 * 内部变量
 * ================================================================ */

/* 启动前保存的控制模式，done/stop 时恢复 */
static uint8_t s2_prev_ctrl_mode = SERVO_CTRL_MODE_SIMPLE_PD;


/* ================================================================
 * 内部：配置八字 IMU 航向段序列
 * ================================================================ */

/**
 * @brief 将 8 段八字轨迹写入 nav_app 的 IMU 段数组
 * @details
 *   航向使用累计角度（相对于 nav_start_imu 时的零点）：
 *   - 绕 A 锥桶左转 360°：航向从 0° → -120° → -240° → -360°
 *   - 绕 B 锥桶右转 360°：航向从 -360° → -240° → -120° → 0°
 *
 *   每段 heading_error = target - current，由 nav_wrap_angle 包裹到 ±180°。
 *   120° 的变化量 < 180°，不会产生方向歧义。
 */
static void s2_configure_segments(void)
{
    const float arc = 2.0f * 3.14159f * g_s2_turn_radius / 3.0f;   /* 每 120° 弧段的弧长 */

    /* 每次启动前先清空旧段，避免重复启动后导航段数组叠加。 */
    nav_clear_imu_segments();

    /* 段0：从发车区直行到交叉点 */
    nav_add_imu_segment(0.0f,    SUBJ2_APPROACH_DIST_M, 0.0f);

    /* 段1~3：绕 A 锥桶左转 360°（3 段 × -120°）*/
    nav_add_imu_segment(-120.0f, arc, 0.0f);
    nav_add_imu_segment(-240.0f, arc, 0.0f);
    nav_add_imu_segment(-360.0f, arc, 0.0f);

    /* 段4~6：绕 B 锥桶右转 360°（航向从 -360° 回到 0°）
       段4: target = -240°（从 -360° 右转 120°）
       段5: target = -120°（继续右转 120°）
       段6: target = 0°（完成 B 圈）*/
    nav_add_imu_segment(-240.0f, arc, 0.0f);
    nav_add_imu_segment(-120.0f, arc, 0.0f);
    nav_add_imu_segment(   0.0f, arc, 0.0f);

    /* 段7：直行回发车区 */
    nav_add_imu_segment(0.0f,    SUBJ2_RETURN_DIST_M, 0.0f);

    printf("[SUBJ2] 8 IMU segments configured: R=%.2fm, arc=%.2fm, total≈%.1fm\r\n",
           g_s2_turn_radius, arc,
           SUBJ2_APPROACH_DIST_M + 6.0f * arc + SUBJ2_RETURN_DIST_M);
}


/* ================================================================
 * 公开 API
 * ================================================================ */

void subject2_app_init(void)
{
    /* 初始化只复位科目2自身状态，不干预当前舵机控制模式。 */
    g_subject2_state = SUBJ2_STATE_IDLE;
}

/**
 * @brief KEY3 长按（科目2模式下）：启动八字绕桩
 * @details
 *   1. 配置 8 段 IMU 航向序列
 *   2. 切换控制模式到 LOW_SPEED（纯 P+I，适合低速）
 *   3. 设置恒定低速转速
 *   4. 启动 IMU 段导航
 */
void subject2_start(void)
{
    if (g_subject2_state != SUBJ2_STATE_IDLE && g_subject2_state != SUBJ2_STATE_DONE)
    {
        printf("[SUBJ2] Already running\r\n");
        return;
    }

    /* 配置八字段序列 */
    s2_configure_segments();

    /* 保存当前控制模式，切换到 LOW_SPEED */
    s2_prev_ctrl_mode    = g_servo_control_mode;
    g_servo_control_mode = SERVO_CTRL_MODE_LOW_SPEED;
    /* 模式切换前清零积分，避免沿用上一控制模式残留的积分量。 */
    balance_pid.integral = 0.0f;   /* 切换模式时清零积分，防止跳变 */
    printf("[SUBJ2] Control mode → LOW_SPEED (was %u)\r\n", (unsigned)s2_prev_ctrl_mode);

    /* 设置恒定低速 */
    motor_set_target_rpm(g_s2_rpm);

    /* 启动 IMU 段导航（自动 reset yaw 零点 + 里程计）*/
    nav_start_imu();

    if (g_nav_state != NAV_STATE_NAVIGATING)
    {
        printf("[SUBJ2] Start FAILED: nav not running\r\n");
        /* 导航启动失败时必须恢复原控制模式，避免系统停留在 LOW_SPEED。 */
        g_servo_control_mode = s2_prev_ctrl_mode;   /* 恢复 */
        motor_output_set_enable(0u);
        return;
    }

    /* 导航已正常进入运行态，此时才真正放行电机输出。 */
    motor_output_set_enable(1u);

    /* 仅在导航成功进入运行态后，才对外宣布科目2已启动。 */
    g_subject2_state = SUBJ2_STATE_RUNNING;
    printf("[SUBJ2] START! Figure-8 @ %.0f RPM, R=%.2fm\r\n",
           g_s2_rpm, g_s2_turn_radius);
}

/**
 * @brief 紧急停止 / 正常结束时的清理
 */
void subject2_stop(void)
{
    /* 先停导航和电机，再恢复控制模式，避免退出时仍按八字段控车。 */
    nav_stop();
    motor_set_target_rpm(0.0f);
    motor_output_set_enable(0u);
    balance_set_expect_angle(0.0f);
    lqr_set_expect_phi(0.0f);

    /* 恢复控制模式 */
    g_servo_control_mode = s2_prev_ctrl_mode;
    balance_pid.integral = 0.0f;   /* 切换模式清零积分 */

    g_subject2_state = SUBJ2_STATE_IDLE;
    printf("[SUBJ2] STOP! Ctrl mode restored to %u\r\n", (unsigned)s2_prev_ctrl_mode);
}

/**
 * @brief 科目2调度任务（100ms）
 * @details 仅监控 NAV_STATE_DONE，到达后停车。
 *          航向误差 → expect_angle 在 isr.c 5ms ISR 中统一注入，此处不管。
 */
void subject2_task(void)
{
    if (g_subject2_state != SUBJ2_STATE_RUNNING)
    {
        return;
    }

    /* IMU 八字全部段完成 → 停车 */
    if (g_nav_state == NAV_STATE_DONE)
    {
        motor_set_target_rpm(0.0f);
        motor_output_set_enable(0u);
        nav_stop();
        balance_set_expect_angle(0.0f);
        lqr_set_expect_phi(0.0f);

        /* 恢复控制模式 */
        g_servo_control_mode = s2_prev_ctrl_mode;
        /* 退出 LOW_SPEED 前清零积分，防止把历史误差带回普通模式。 */
        balance_pid.integral = 0.0f;

        g_subject2_state = SUBJ2_STATE_DONE;
        printf("[SUBJ2] DONE! Figure-8 complete. Ctrl mode restored to %u\r\n",
               (unsigned)s2_prev_ctrl_mode);
    }
}
