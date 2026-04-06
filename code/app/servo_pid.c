#include "servo_pid.h"
#include "imu_app.h"
#include "servo_app.h"



#pragma section all "cpu0_dsram"
// ==================== 全局变量定义 ====================


PID_T balance_pid = {0};           // 平衡控制PID结构体
volatile float expect_angle = 0.0f;   // 期望倾角（默认0，用于转弯控制）

/* [新增] 控制模式，默认保持原有 SIMPLE_PD 行为，不影响任何现有调试结果 */
volatile uint8_t g_servo_control_mode = SERVO_CTRL_MODE_SIMPLE_PD;

/* 舵机转向总方向：
   只在最终输出层生效，不改变 IMU、导航或 PID 的角度正负定义。 */
volatile int8_t g_servo_steer_dir = SERVO_STEER_DIR_REVERSED;

/* [新增] 串级内环参数（CASCADE 模式专用）
   初始默认值：inner_kp=1.0 使得串级输出量级与原 SIMPLE_PD 接近，
   inner_ki=0.0 内环积分默认关闭，避免初次使用时积分饱和。
   调参建议：先调好外环 kp/ki，再逐步增大 inner_kp。           */
volatile float g_balance_inner_kp = 1.0f;
volatile float g_balance_inner_ki = 0.0f;

/* [新增] 内环积分状态（仅在 CASCADE 模式下使用）*/
static float s_inner_integral = 0.0f;





/* ==================== NEW: 平衡控制参数改为全局变量 ====================
   原来的 PID 宏定义改成全局变量，便于在 IPS 舵机页面直接显示和调节。
   注意：这些变量后续会直接参与控制计算，不只是用于显示。
*/
volatile float g_balance_kp = 20.0f;                // 原 BALANCE_KP
volatile float g_balance_ki = 0.25f;                // 原 BALANCE_KI
volatile float g_balance_kd = 0.05f;                // 原 BALANCE_KD
volatile float g_balance_output_limit = 250.0f;     // 原 BALANCE_LIMIT--250
volatile float g_balance_integral_limit = 200.0f;   // 原 INTEGRAL_LIMIT
volatile float g_balance_pid_output = 0.0f;         // 供主循环调试读取的最终舵机输出






/* ==================== NEW: 平衡控制使能标志 ====================
   0：关闭平衡控制
   1：允许 balance_control() 真正输出到舵机
*/
static uint8_t g_balance_control_enable = 0;





//获取目前舵机角度
volatile float pwm_angle = 90.0f;

#pragma section all restore

/**
 * @brief  初始化平衡PID控制器
 * @param  无
 * @return 无
 * @note   在主函数初始化时调用一次
 */
void balance_pid_init(void)
{
    // 初始化PID结构体
    // 参数：PID指针, kp, ki, kd, 目标值, 输出限幅
    pid_init(&balance_pid, g_balance_kp, g_balance_ki, g_balance_kd, 0.0f, g_balance_output_limit);

    // 设置积分限幅，防止积分饱和  (-limit) -   (limit)
    pid_app_limit_integral(&balance_pid, -g_balance_integral_limit, g_balance_integral_limit);

    // 初始化期望角度为0（直线行驶）
    expect_angle = 0.0f;
}

/* ==================== NEW: 平衡控制使能接口 ====================
   关闭控制时：
   - 清空 PID 历史状态，避免重新打开时积分残留
   - 让舵机回到中位，避免启动阶段误动作
*/
void balance_control_set_enable(uint8_t enable)
{
    g_balance_control_enable = enable;

    if (0U == enable)
    {
        pid_reset(&balance_pid);
        s_inner_integral = 0.0f;   /* [新增] 同时清零串级内环积分 */
        servo_set(g_servo_mid_duty);
    }
    else
    {
        /* ==================== NEW: 启动控制前再次清空PID状态 ====================
           目的不是改变原有控制策略，而是避免”重新打开控制”的那个瞬间，
           因为残留积分项或历史状态导致舵机先打一下。
        */
        pid_reset(&balance_pid);
        s_inner_integral = 0.0f;   /* [新增] 同时清零串级内环积分 */
    }
}

/**
 * @brief  核心平衡控制函数（支持三种控制模式）
 * @param  无
 * @return 无
 * @note   在 5ms 定时器中断中调用
 *
 * 模式说明：
 *   SERVO_CTRL_MODE_SIMPLE_PD  (默认)
 *     原始位置式 P+I+D，D 项直接用陀螺仪角速度。
 *     参数：g_balance_kp / g_balance_ki / g_balance_kd
 *
 *   SERVO_CTRL_MODE_CASCADE
 *     串级双环：外环（角度误差→目标角速率）+ 内环（角速率误差→舵机）。
 *     外环参数：g_balance_kp / g_balance_ki
 *     内环参数：g_balance_inner_kp / g_balance_inner_ki
 *     调参顺序：先调外环 kp（令 ki=0 inner_kp=1），再调内环 kp，最后加外环 ki。
 *
 *   SERVO_CTRL_MODE_LOW_SPEED
 *     仅角度环 P+I，去掉陀螺项，消除低速时陀螺噪声导致的震荡。
 *     适用于题目二低速绕八字。参数：g_balance_kp / g_balance_ki
 */
void balance_control(void)
{
    float current_angle;      /* 当前横滚角（控制用）*/
    float current_gyro;       /* 当前 X 轴角速度（deg/s）*/
    float angle_error;        /* 外环角度误差 */
    float pid_output;         /* 最终舵机偏置（duty counts）*/
    float servo_pwm;          /* 舵机 PWM 值 */

    /* 启动阶段保护：零偏校准、零位捕获完成前禁止输出 */
    if (0U == g_balance_control_enable)
    {
        return;
    }

    /* 获取传感器数据（根据 IMU 安装方向，本项目用 roll+gyro_x）*/
    current_angle = roll_ctrl_angle;    /* 补偿后横滚角（°）*/
    current_gyro  = gyro_x_rate;        /* X 轴角速度（deg/s）*/

    /* 角度误差（外环共用）*/
    angle_error = expect_angle - current_angle;

    /* ================================================================
     * 控制模式分支
     * ================================================================ */

    if (g_servo_control_mode == SERVO_CTRL_MODE_CASCADE)
    {
        /* -------- [新增] 串级双环 --------
         * 外环：角度误差 → 目标角速率（deg/s）
         * 内环：角速率误差 → 舵机偏置（duty counts）
         *
         * 量纲说明：
         *   外环输出 = kp * angle_error(°) + ki * integral(°)，结果是 deg/s
         *   内环输出 = inner_kp * rate_error(deg/s)，结果是 duty counts
         * 初始 inner_kp=1 时：输出量级与 SIMPLE_PD 的 kp*angle_error 接近。
         */
        float outer_i_out;
        float target_rate;    /* 外环输出：期望角速率（deg/s）*/
        float rate_error;     /* 内环输入：角速率误差（deg/s）*/
        float inner_i_out;

        /* 外环积分（角度积分）*/
        balance_pid.integral += angle_error;
        if (balance_pid.integral >  g_balance_integral_limit) { balance_pid.integral =  g_balance_integral_limit; }
        if (balance_pid.integral < -g_balance_integral_limit) { balance_pid.integral = -g_balance_integral_limit; }

        outer_i_out = g_balance_ki * balance_pid.integral;
        target_rate = g_balance_kp * angle_error + outer_i_out;

        /* 内环：角速率误差 */
        rate_error = target_rate - current_gyro;

        /* 内环积分（角速率积分，仅 inner_ki != 0 时有意义）*/
        s_inner_integral += rate_error;
        if (s_inner_integral >  g_balance_integral_limit) { s_inner_integral =  g_balance_integral_limit; }
        if (s_inner_integral < -g_balance_integral_limit) { s_inner_integral = -g_balance_integral_limit; }

        inner_i_out = g_balance_inner_ki * s_inner_integral;
        pid_output  = g_balance_inner_kp * rate_error + inner_i_out;

        /* 调试信息保存（复用 balance_pid 字段）*/
        balance_pid.p_out = target_rate;    /* 外环输出（目标角速率）*/
        balance_pid.i_out = inner_i_out;
        balance_pid.d_out = rate_error;     /* 内环输入（角速率误差）*/
        balance_pid.error = angle_error;
    }
    else if (g_servo_control_mode == SERVO_CTRL_MODE_LOW_SPEED)
    {
        /* -------- [新增] 低速模式（题目二） --------
         * 仅角度 P+I，去掉陀螺 D 项。
         * 低速时陀螺噪声放大效应明显，去掉 D 项可消除高频震荡。
         * 响应相对慢，但在低速（<1m/s）绕八字场景下已足够。
         */
        float p_out, i_out;

        balance_pid.integral += angle_error;
        if (balance_pid.integral >  g_balance_integral_limit) { balance_pid.integral =  g_balance_integral_limit; }
        if (balance_pid.integral < -g_balance_integral_limit) { balance_pid.integral = -g_balance_integral_limit; }

        p_out = g_balance_kp * angle_error;
        i_out = g_balance_ki * balance_pid.integral;
        pid_output = p_out + i_out;

        balance_pid.p_out = p_out;
        balance_pid.i_out = i_out;
        balance_pid.d_out = 0.0f;   /* 低速模式无 D 项 */
        balance_pid.error = angle_error;

        s_inner_integral = 0.0f;    /* 未使用，保持清零 */
    }
    else
    {
        /* -------- SIMPLE_PD（默认，保持原有逻辑完全不变）-------- */
        float p_out, i_out, d_out;

        /* 运行时同步参数（IPS页面改参数后立即生效）*/
        balance_pid.kp = g_balance_kp;
        balance_pid.ki = g_balance_ki;
        balance_pid.kd = g_balance_kd;

        p_out = g_balance_kp * angle_error;

        balance_pid.integral += angle_error;
        if (balance_pid.integral >  g_balance_integral_limit) { balance_pid.integral =  g_balance_integral_limit; }
        if (balance_pid.integral < -g_balance_integral_limit) { balance_pid.integral = -g_balance_integral_limit; }
        i_out = g_balance_ki * balance_pid.integral;

        d_out = g_balance_kd * current_gyro;   /* D 项直接用陀螺仪角速度 */

        pid_output = p_out + i_out + d_out;

        balance_pid.p_out = p_out;
        balance_pid.i_out = i_out;
        balance_pid.d_out = d_out;
        balance_pid.error = angle_error;

        s_inner_integral = 0.0f;    /* 切换模式时保持清零 */
    }

    /* ================================================================
     * 输出限幅 → 驱动舵机（三种模式共用）
     * ================================================================ */
    if (pid_output >  g_balance_output_limit) { pid_output =  g_balance_output_limit; }
    if (pid_output < -g_balance_output_limit) { pid_output = -g_balance_output_limit; }

    pid_output *= (float)g_servo_steer_dir;
    balance_pid.out = pid_output;
    g_balance_pid_output = pid_output;

    servo_pwm = (float)g_servo_mid_duty + pid_output;
    servo_set((uint32_t)servo_pwm);

    pwm_angle = SERVO_DUTY_TO_ANGLE(servo_pwm);
}

/**
 * @brief  设置期望倾角
 * @param  angle 期望倾角（单位：度）
 * @return 无
 * @note   该接口只更新控制参考，不直接驱动舵机。
 *         由 5ms 控制 ISR 在 balance_control()/lqr_balance_control() 中消费。
 */
void balance_set_expect_angle(float angle)
{
    expect_angle = angle;
}

/**
 * @brief  运行时更新平衡 PID 参数
 * @param  kp 比例系数
 * @param  ki 积分系数
 * @param  kd 微分系数
 * @return 无
 * @note   仅同步到运行时全局参数；balance_control() 每次执行时会读取这些值。
 */
void balance_set_params(float kp, float ki, float kd)
{
    g_balance_kp = kp;
    g_balance_ki = ki;
    g_balance_kd = kd;
}
