#include "servo_pid.h"
#include "imu_app.h"
#include "servo_app.h"

// ==================== 全局变量定义 ====================
PID_T balance_pid = {0};           // 平衡控制PID结构体
float expect_angle = 0.0f;   // 期望倾角（默认0，用于转弯控制）

/* ==================== NEW: 平衡控制参数改为全局变量 ====================
   原来的 PID 宏定义改成全局变量，便于在 IPS 舵机页面直接显示和调节。
   注意：这些变量后续会直接参与控制计算，不只是用于显示。
*/
volatile float g_balance_kp = 20.0f;                // 原 BALANCE_KP
volatile float g_balance_ki = 0.25f;                // 原 BALANCE_KI
volatile float g_balance_kd = 0.05f;                // 原 BALANCE_KD
volatile float g_balance_output_limit = 250.0f;     // 原 BALANCE_LIMIT
volatile float g_balance_integral_limit = 200.0f;   // 原 INTEGRAL_LIMIT



/* ==================== NEW: 平衡控制使能标志 ====================
   0：关闭平衡控制
   1：允许 balance_control() 真正输出到舵机
*/
static uint8_t g_balance_control_enable = 0;



// // ==================== PID初始参数 ====================
// #define BALANCE_KP      20.0f      // 角度比例系数
// #define BALANCE_KI      0.25f      // 积分系数（继续增大，消除残余静差）
// #define BALANCE_KD      0.05f       // 

// /* ==================== PID输出限幅 ==================== */
// #define BALANCE_LIMIT   250.0f      // PID输出限幅
// #define INTEGRAL_LIMIT  200.0f      // 积分限幅（防止积分饱和）




//获取目前舵机角度
volatile float pwm_angle = 90.0f;



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
        servo_set(g_servo_mid_duty);
    }
    else
    {
        /* ==================== NEW: 启动控制前再次清空PID状态 ====================
           目的不是改变原有控制策略，而是避免“重新打开控制”的那个瞬间，
           因为残留积分项或历史状态导致舵机先打一下。
        */
        pid_reset(&balance_pid);
    }
}

/**
 * @brief  核心平衡控制函数
 * @param  无
 * @return 无
 * @note   在5ms定时器中断中调用
 */
void balance_control(void)
{
    float current_angle;    // 当前角度
    float current_gyro;     // 当前角速度
    float angle_error;      // 角度误差
    float p_out, i_out, d_out;  // PID各项输出
    float pid_output;       // PID总输出
    float servo_pwm;        // 舵机PWM值

    /* ==================== NEW: 启动阶段保护 ====================
       在 IMU 零偏校准、机械零位捕获完成前，不允许平衡控制介入。
    */
    if (0U == g_balance_control_enable)
    {
        return;
    }

    /* ==================== NEW: 运行时同步PID参数 ====================
       由于参数已经改成 IPS 可调的全局变量，这里每次控制都同步一次，
       保证页面改参数后能立刻反映到控制器。
    */
    balance_pid.kp = g_balance_kp;
    balance_pid.ki = g_balance_ki;
    balance_pid.kd = g_balance_kd;


    // 1. 获取传感器数据============ 根据实际安装方向调整X/Y =============
    // current_angle = roll_kalman;   
    // current_gyro = gyro_y_rate;    // y轴角速度（deg/s）


    // 1. 获取传感器数据 - 使用补偿后的控制角度
    current_angle = roll_ctrl_angle;    // ✅ 横滚角（左右倾斜）
    current_gyro = gyro_x_rate;         // ✅ X 轴角速度（左右倾倒速度）

  //================这里注意根据IMU安装方向修改============================//

    // 2. 计算角度误差
    angle_error = expect_angle - current_angle;

    // 3. 手动计算位置式PID（关键：D项直接使用陀螺仪值）
    // P项：角度误差的比例控制
    p_out = g_balance_kp * angle_error;

    // I项：积分累加（消除静差）
    balance_pid.integral += angle_error;

    // 积分限幅，防止积分饱和
    if (balance_pid.integral > g_balance_integral_limit) {
        balance_pid.integral = g_balance_integral_limit;
    } else if (balance_pid.integral < -g_balance_integral_limit) {
        balance_pid.integral = -g_balance_integral_limit;
    }
    i_out = g_balance_ki * balance_pid.integral;

    // D项：直接使用陀螺仪角速度（抑制快速倾倒）
    d_out = g_balance_kd * current_gyro;

    // PID总输出
    pid_output = p_out + i_out + d_out;

    // 保存到结构体（用于调试）
    balance_pid.p_out = p_out;
    balance_pid.i_out = i_out;
    balance_pid.d_out = d_out;
    balance_pid.error = angle_error;

    // 4. 输出限幅
    if (pid_output > g_balance_output_limit) {
        pid_output = g_balance_output_limit;
    } else if (pid_output < -g_balance_output_limit) {
        pid_output = -g_balance_output_limit;
    }
    balance_pid.out = pid_output;

    // 5. 将PID输出映射到舵机PWM范围
    servo_pwm = (float)g_servo_mid_duty + pid_output;

    // 6. 发送调试数据
//    JustFloat_Test_three(current_angle, current_gyro, pid_output);

    // 7. 控制舵机
    servo_set((uint32_t)servo_pwm);

    //获取现在转向角度
    pwm_angle = SERVO_DUTY_TO_ANGLE(servo_pwm);


    // 调试输出
//printf("PWM=%d, Angle=%.2f, PID=%.2f\r\n",
//           (uint32_t)servo_pwm, pwm_angle, pid_output);
}
