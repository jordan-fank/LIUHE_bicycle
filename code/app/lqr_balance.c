/*
 * 文件: lqr_balance.c
 * 功能: LQR 平衡控制应用层实现，负责读取状态并输出舵机控制量
 * 作者: 闫锦
 * 日期: 2026-03-31
 */

#include "lqr_balance.h"
#include "WP_Math.h"
#include <math.h>

// ==================== 全局变量 ====================
LQR_Controller_t lqr_ctrl;
float lqr_expect_phi = 0.0f;        // 期望侧倾角（度）

// ==================== 函数实现 ====================

/**
 * @brief  初始化LQR平衡控制
 */
void lqr_balance_init(void)
{
    // 初始化LQR控制器
    lqr_init(&lqr_ctrl);

    // 设置物理参数（默认值来自 lqr_driver.h，后续需要按实车测量结果调整）
    lqr_set_physical_params(&lqr_ctrl,
                            LQR_DEFAULT_COM_HEIGHT_M,
                            LQR_DEFAULT_WHEELBASE_M,
                            LQR_DEFAULT_TRAIL_M);

    // 设置限幅（默认值来自 lqr_driver.h，转向角极限仍由应用层机械限位决定）
    lqr_set_limits(&lqr_ctrl,
                   LQR_DELTA_MECHANICAL_MAX,    // 最大转向角
                   LQR_DEFAULT_DELTA_DOT_MAX_RAD_S, // 最大转向角速度
                   LQR_DEFAULT_V_MIN_M_S);          // 最小速度

    // 初始化期望角度
    lqr_expect_phi = 0.0f;      //期望角=0意味着跑直线

    /* ==================== NEW: 默认关闭LQR ====================
       与原 PID 上电流程保持一致，等 zero_compensation() 完成后再显式开启。
    */
    lqr_enable(&lqr_ctrl, 0);
    servo_set(g_servo_mid_duty);
    pwm_angle = SERVO_DUTY_TO_ANGLE(g_servo_mid_duty);
}

/* ==================== NEW: LQR使能接口 ====================
   作用与原 balance_control_set_enable() 保持一致，便于最小代价切换控制方案。
*/
void lqr_balance_set_enable(uint8_t enable)
{
    lqr_enable(&lqr_ctrl, enable);

    if (0U == enable)
    {
        lqr_reset(&lqr_ctrl);
        servo_set(g_servo_mid_duty);
        pwm_angle = SERVO_DUTY_TO_ANGLE(g_servo_mid_duty);
    }
}

/**
 * @brief  获取当前用于 LQR 调度的速度大小
 * @return 速度大小（m/s），恒正
 */
float lqr_get_velocity(void)
{
    /* ==================== NEW: 速度接口收口到电机层 ====================
       LQR 不再自己维护“RPM -> m/s”的换算公式。
       统一由 motor_app 提供线速度接口，避免同一套物理量在两个模块各算一遍。

       这里不直接复用 motor_speed_m_s 这个缓存值：
       1. lqr_balance_control() 当前运行在 5ms 中断
       2. motor_control() 当前运行在 20ms 中断
       3. motor_speed_m_s 是 motor_control() 刷新的结果
       所以 LQR 如果直接拿缓存，读到的可能是 0~20ms 之前的旧值。
       这里直接从最新 RPM 反馈做一次轻量换算，更符合当前时序关系。

       这里取的是“速度大小”而不是“带方向速度”：
       1. 当前 LQR 增益表按自行车前进平衡模型使用
       2. 增益调度只关心车速有多大，不区分正向/反向
       3. 如果以后要支持倒车平衡，应该重新审视模型，而不是直接复用这张表
    */
    return motor_get_vehicle_speed_abs_m_s();
}

/**
 * @brief  转向角转换为舵机PWM
 * @param  delta: 转向角（rad），正值=右转
 * @return 舵机PWM值
 */
uint32 lqr_delta_to_servo_pwm(float delta)
{
    float ratio, pwm;

    // 线性映射：delta -> PWM
    // delta = 0 -> mid
    // delta = +max -> r_max (右转)
    // delta = -max -> l_max (左转)
    ratio = delta / LQR_DELTA_MECHANICAL_MAX;

    // 限幅
    if (ratio > 1.0f) ratio = 1.0f;
    if (ratio < -1.0f) ratio = -1.0f;

    // 映射到PWM（左右分段，适应不对称舵机）
    if (ratio >= 0)
        pwm = (float)LQR_SERVO_MID + ratio * (float)(LQR_SERVO_RIGHT - LQR_SERVO_MID);
    else
        pwm = (float)LQR_SERVO_MID + ratio * (float)(LQR_SERVO_MID - LQR_SERVO_LEFT);

    // 边界保护
    if (pwm > (float)LQR_SERVO_RIGHT) pwm = (float)LQR_SERVO_RIGHT;
    if (pwm < (float)LQR_SERVO_LEFT) pwm = (float)LQR_SERVO_LEFT;

    return (uint32)pwm;
}

/**
 * @brief  LQR平衡控制主函数
 * @note   在5ms中断中调用
 * 
 * 功能-保持平衡不倒：
 * 如果是直线行驶，目标是让车子保持直立；
 *  如果是转弯，目标是让车子保持合适的倾斜角度来安全过弯。
 * 
 * 参数说明：
 * phi (φ) - 侧倾角
    phi_dot (φ̇) - 侧倾角速度
    delta (δ) - 转向角
    delta_cmd - 转向角命令
 */
void lqr_balance_control(void)
{
    float velocity;
    float phi, phi_dot;
    float delta_cmd;
    uint32 servo_pwm;
    int8_t ret;

    /* ==================== NEW: 启动阶段保护 ====================
       LQR 当前作为实际启用的平衡控制入口时，也沿用“零位捕获完成后再允许输出”的策略。
    */
    if (!lqr_ctrl.enabled) {
        return;
    }

    // 1. 获取当前实际速度的绝对值
    velocity = lqr_get_velocity();

    // 2. 更新增益（根据速度插值）
    ret = lqr_update_gain(&lqr_ctrl, velocity);

    // 2.5 计算配套的目标转向角（物理套餐）
    // 公式：tan(phi) = v^2 / (g*L) * delta
    // 反推：delta = tan(phi) * g * L / v^2
    {
        float L = lqr_ctrl.params.L;
        float g = lqr_ctrl.params.g;
        float target_phi_rad = lqr_expect_phi * DEG_TO_RAD;

        if (velocity > 0.5f) {
            lqr_ctrl.target_delta = tanf(target_phi_rad) * g * L / (velocity * velocity);
            // 限幅保护
            if (lqr_ctrl.target_delta > LQR_DELTA_MECHANICAL_MAX) {
                lqr_ctrl.target_delta = LQR_DELTA_MECHANICAL_MAX;
            }
            if (lqr_ctrl.target_delta < -LQR_DELTA_MECHANICAL_MAX) {
                lqr_ctrl.target_delta = -LQR_DELTA_MECHANICAL_MAX;
            }

        }
         else //如果速度低于0.5m/s，则不允许压弯
        {
            lqr_ctrl.target_delta = 0.0f;  // 倾角期望设置为0，低速时不允许压弯
        }
    }

    /* ==================== NEW: 对齐当前工程已验证的平衡轴 ====================
       原移植代码使用的是 pitch_kalman / gyro_y_rate。
       但你当前项目里，真正用于左右平衡的轴已经在 PID 方案中验证为：
       - 角度：roll_ctrl_angle
       - 角速度：gyro_x_rate

       这里保持 LQR 的符号约定不变：
       - IMU 左倾为正
       - LQR 模型按“右倾为正”建模
       因此继续取反。

       ！！！！！！！！！！这里必须注意！！！！
       后续安装到车上，方向又会发生变化，需要改代码
    */
    phi = -(roll_ctrl_angle - lqr_expect_phi) * DEG_TO_RAD;     //侧倾角误差
    phi_dot = -gyro_x_rate * DEG_TO_RAD;                        //侧倾角-角速度

    // 4. 低速保护：速度过低时缓慢回中
    if (ret < 0) {
        // 低速时缓慢回中（衰减系数0.95）
        lqr_ctrl.delta_cmd *= 0.95f;
    } else {
        // 5. 执行LQR计算
        lqr_compute(&lqr_ctrl, phi, phi_dot);       //计算出最优控制量
    }



    /*
        delta_cmd 告诉舵机应该转向多少角度来：

        纠正当前的平衡误差（基于 phi）
        阻尼侧倾运动（基于 phi_dot）
        实现期望的转弯（通过 target_delta 的影响）
    */
    // 6. 获取转向角命令
    delta_cmd = lqr_ctrl.delta_cmd;                 //从控制器结构体中取出计算结果

    // 7. 转换为舵机PWM并输出
    servo_pwm = lqr_delta_to_servo_pwm(delta_cmd);
    servo_set(servo_pwm);

    /* ==================== NEW: 同步当前舵机角度到显示变量 ==================== */
    pwm_angle = SERVO_DUTY_TO_ANGLE(servo_pwm);
}



/**
 * @brief  设置期望侧倾角（用于转弯）
 * @param  phi_deg: 期望侧倾角（度）
 */
void lqr_set_expect_phi(float phi_deg)
{
    lqr_expect_phi = phi_deg;
}

/**
 * @brief  运行时调整物理参数
 */
void lqr_tune_params(float h, float L)
{
    lqr_set_physical_params(&lqr_ctrl, h, L, lqr_ctrl.params.b);
}

/**
 * @brief  获取调试信息
 */
void lqr_get_debug_info(float *phi, float *velocity, float *delta_deg)
{
    if (phi != NULL) {
        *phi = roll_ctrl_angle;
    }
    if (velocity != NULL) {
        *velocity = lqr_ctrl.current_velocity;
    }
    if (delta_deg != NULL) {
        *delta_deg = lqr_ctrl.delta_cmd * RAD_TO_DEG;
    }
}
