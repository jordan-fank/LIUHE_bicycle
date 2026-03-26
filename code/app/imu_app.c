/*
    确认 IMU660RB 在电路板上的安装方向：

    算法期望车体坐标：
    X轴：车辆前进方向
    Y轴：车辆左侧
    Z轴：车辆上方

    我目前的安装方向：
    X轴：向后（旋转 180°）
    Y 轴：向右（旋转180°）
    Z轴：向下 ← 所以 acc.z = -0.99

    所以需要初始化和处理全部加负号

    如果当前安装方向与上述定义不一致，需要在本文件中先做坐标映射，
    再把数据送给姿态解算。
*/

#include "imu_app.h"
#include "mcu_dmp.h"
#include "WP_DataType.h"

#define IMU_ACC_LSB_PER_G       (4098.0f)
#define IMU_GYRO_LSB_PER_DPS    (14.3f)

typedef struct
{
    int32_t acc_x;
    int32_t acc_y;
    int32_t acc_z;
    int32_t gyro_x;
    int32_t gyro_y;
    int32_t gyro_z;
} imu_raw_sample_t;

// 以下数据都是在中断或频繁访问的数据，放在 DSRAM 中
#pragma section all "cpu0_dsram"

float roll_kalman = 0;
float pitch_kalman = 0;
float yaw_kalman = 0;
float gyro_x_rate = 0;
float gyro_y_rate = 0;





/* ==================== NEW: 机械零位补偿后的控制角度 ====================
   这两个变量是新增的：
   - roll_kalman / pitch_kalman 保持原始姿态含义，不覆盖
   - roll_ctrl_angle / pitch_ctrl_angle 给控制器使用
*/
float roll_ctrl_angle = 0;
float pitch_ctrl_angle = 0;






// 陀螺仪零偏（校准后的值）
static float gyro_x_offset = 0.0f;
static float gyro_y_offset = 0.0f;
static float gyro_z_offset = 0.0f;

/* ==================== NEW: 机械装配零位补偿值 ====================
   保存“机械零位时，原始姿态角是多少”。

   例如静止时稳定测得：
   - roll_kalman  = 4.27
   - pitch_kalman = 0.38

   则可以把这两个值记为机械零位，后续控制角计算为：
   - roll_ctrl_angle  = roll_kalman  - g_roll_control_zero_deg
   - pitch_ctrl_angle = pitch_kalman - g_pitch_control_zero_deg
*/
static float g_roll_control_zero_deg = 0.0f;
static float g_pitch_control_zero_deg = 0.0f;










// 校准完成标志
uint8_t gyro_calibrated = 0;





// ISR 采样结果，主循环消费
static volatile imu_raw_sample_t g_imu_raw_sample = {0};
static volatile uint8_t g_imu_sample_ready = 0;






/* ==================== TEST ONLY: IMU采样/丢帧统计 ==================== */
volatile imu_diag_stat_t g_imu_diag_stat = {0};
/* ==================== TEST ONLY: IMU采样/丢帧统计 ==================== */

#pragma section all restore





//机械零位补偿
void zero_compensation(void)
{
    
 // ========== 关键：等待姿态稳定 ==========
    // 消费掉至少一帧数据，让 roll_kalman/pitch_kalman 有有效值
    for (int i = 0; i < 100; i++) {
        imu_proc();  // 处理数据
        system_delay_ms(IMU_PERIOD_MS);
    }

    // ========== 调用机械零位补偿 ==========
    // 方法1：使用固定值（如果你已经测出来了）
    // imu_set_control_zero(3.5f, 0.38f);  // 假设你测得静止时 roll=3.5°, pitch=0.38°

    // 方法 2：自动捕获当前姿态为零位（推荐）
    imu_capture_control_zero();  // ✅ 把当前姿态记录为机械零位


printf("===== Mechanical Zero Captured =====\r\n");
printf("  Raw roll  = %.2f°\r\n", roll_kalman);
printf("  Raw pitch = %.2f°\r\n", pitch_kalman);
printf("  After compensation:\r\n");
printf("  roll_ctrl  = %.2f°\r\n", roll_ctrl_angle);
printf("  pitch_ctrl = %.2f°\r\n", pitch_ctrl_angle);
printf("====================================\r\n");
 
}








/* ==================== NEW: 角度归一化辅助函数 ====================
   作用：把补偿后的控制角度限制到 [-180, 180] 范围内。
*/
static float imu_normalize_deg(float angle_deg)
{
    while (angle_deg > 180.0f)
    {
        angle_deg -= 360.0f;
    }
    while (angle_deg < -180.0f)
    {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

void usart_send(float roll, float pitch, float yaw, uint8_t fusion_sta)
{
    uint8_t buffer[15];
    uint8_t sumcheck = 0;
    uint8_t addcheck = 0;
    uint8_t index = 0;
    int16_t roll_int = (int16_t)(roll * 100.0f);
    int16_t pitch_int = (int16_t)(pitch * 100.0f);
    int16_t yaw_int = (int16_t)(yaw * 100.0f);

    buffer[index++] = 0xAB;
    buffer[index++] = 0xDC;
    buffer[index++] = 0xFE;
    buffer[index++] = 0x03;
    buffer[index++] = 7;
    buffer[index++] = 0;

    buffer[index++] = (uint8_t)(roll_int & 0xFF);
    buffer[index++] = (uint8_t)((roll_int >> 8) & 0xFF);
    buffer[index++] = (uint8_t)(pitch_int & 0xFF);
    buffer[index++] = (uint8_t)((pitch_int >> 8) & 0xFF);
    buffer[index++] = (uint8_t)(yaw_int & 0xFF);
    buffer[index++] = (uint8_t)((yaw_int >> 8) & 0xFF);
    buffer[index++] = fusion_sta;

    for (int i = 0; i < index; i++)
    {
        sumcheck += buffer[i];
        addcheck += sumcheck;
    }

    buffer[index++] = sumcheck;
    buffer[index++] = addcheck;

    for (int i = 0; i < index; i++)
    {
        printf("%c", buffer[i]);
    }
}




// 初始化 IMU
void imu_all_init(void)
{
    imu660rb_init();
    g_imu_sample_ready = 0;
    g_roll_control_zero_deg = 0.0f;
    g_pitch_control_zero_deg = 0.0f;
    roll_ctrl_angle = 0.0f;
    pitch_ctrl_angle = 0.0f;

    /* ==================== TEST ONLY: IMU统计复位 ==================== */
    g_imu_diag_stat.sample_count = 0;
    g_imu_diag_stat.process_count = 0;
    g_imu_diag_stat.drop_count = 0;
    /* ==================== TEST ONLY: IMU统计复位 ==================== */

    // 读取第一帧加速度计数据，用于初始化姿态
    imu660rb_get_acc();

    Axis3f acc;
    acc.x = -(float)imu660rb_acc_x / IMU_ACC_LSB_PER_G;
    acc.y = -(float)imu660rb_acc_y / IMU_ACC_LSB_PER_G;
    acc.z = -(float)imu660rb_acc_z / IMU_ACC_LSB_PER_G;

    imu_init_from_accel(acc);       // 初始化姿态

}

/**
 陀螺仪零偏校准
 上电后调用，需要保持车辆静止0
 读取 CALIBRATION_SAMPLES 次数据，取平均值作为零偏
*/
#define CALIBRATION_SAMPLES     30000
#define CALIBRATION_DELAY_MS    2

void imu_calibrate_gyro(void)
{
    int32_t sum_x = 0, sum_y = 0, sum_z = 0;
    uint16_t i;

    system_delay_ms(100);

    for (i = 0; i < CALIBRATION_SAMPLES; i++)
    {
        imu660rb_get_gyro();
        sum_x += imu660rb_gyro_x;
        sum_y += imu660rb_gyro_y;
        sum_z += imu660rb_gyro_z;
        system_delay_ms(CALIBRATION_DELAY_MS);
    }

    gyro_x_offset = (float)sum_x / CALIBRATION_SAMPLES;
    gyro_y_offset = (float)sum_y / CALIBRATION_SAMPLES;
    gyro_z_offset = (float)sum_z / CALIBRATION_SAMPLES;
    printf("%f,%f,%f\r\n", gyro_x_offset, gyro_y_offset, gyro_z_offset);

    gyro_calibrated = 1;
}

/**
 陀螺仪零偏校准临时值
 避免每次上电都零漂校准，这里使用反复测试后的临时值
*/
void imu_calibrate_gyro_temp(void)
{
    gyro_x_offset = 4.935167;
    gyro_y_offset = -12.111170;
    gyro_z_offset = 4.044900;
    gyro_calibrated = 1;
}




/* ==================== NEW: 机械零位补偿接口实现 ==================== */

//手动设置
void imu_set_control_zero(float roll_zero_deg, float pitch_zero_deg)
{
    /* 这里传入的不是“要减去多少”，而是“机械零位时原始角度是多少”。
       例如：
       imu_set_control_zero(4.27f, 0.38f);
       调用后控制角会按以下方式得到：
       - roll_ctrl_angle  = roll_kalman  - 4.27
       - pitch_ctrl_angle = pitch_kalman - 0.38
    */
    g_roll_control_zero_deg = roll_zero_deg;
    g_pitch_control_zero_deg = pitch_zero_deg;

    /* ==================== NEW: 立刻同步控制角 ====================
       只更新“记录的零位值”还不够。
       如果这里不立刻重算 roll_ctrl_angle / pitch_ctrl_angle，
       那么在下一次 imu_proc() 到来之前，控制器读到的仍可能是补偿前的旧值，
       从而在刚打开 balance_control() 时让舵机先抖一下。
    */
    roll_ctrl_angle = imu_normalize_deg(roll_kalman - g_roll_control_zero_deg);
    pitch_ctrl_angle = imu_normalize_deg(pitch_kalman - g_pitch_control_zero_deg);
}

//自动捕获
void imu_capture_control_zero(void)
{
    /* 把“当前姿态”直接记录为机械零位。
       用法：
       1. 先把车摆到你认可的机械装配零位
       2. 再调用一次 imu_capture_control_zero()
    */
    g_roll_control_zero_deg = roll_kalman;
    g_pitch_control_zero_deg = pitch_kalman;

    /* ==================== NEW: 立刻同步控制角 ====================
       对“自动捕获当前姿态为零位”的场景，
       这里理论上会让 roll_ctrl_angle / pitch_ctrl_angle 立即变为 0 附近，
       避免后面刚使能平衡控制时先读到旧控制角。
    */
    roll_ctrl_angle = 0.0f;
    pitch_ctrl_angle = 0.0f;
}
/* ==================== NEW: 机械零位补偿接口实现 ==================== */




// 采样，放在中断
void imu_sample_isr(void)
{
    if (gyro_calibrated != 1)
    {
        return;
    }

    /* ==================== TEST ONLY: IMU采样/丢帧统计 ==================== */
    g_imu_diag_stat.sample_count++;
    if (g_imu_sample_ready != 0)
    {
        g_imu_diag_stat.drop_count++;
    }
    /* ==================== TEST ONLY: IMU采样/丢帧统计 ==================== */

    imu660rb_get_gyro();
    imu660rb_get_acc();

    g_imu_raw_sample.acc_x = imu660rb_acc_x;
    g_imu_raw_sample.acc_y = imu660rb_acc_y;
    g_imu_raw_sample.acc_z = imu660rb_acc_z;
    g_imu_raw_sample.gyro_x = imu660rb_gyro_x;
    g_imu_raw_sample.gyro_y = imu660rb_gyro_y;
    g_imu_raw_sample.gyro_z = imu660rb_gyro_z;

    g_imu_sample_ready = 1;
}

// 对采样的数据进行处理，放在 while，减轻中断负担
void imu_proc(void)
{
    imu_raw_sample_t raw_sample;
    uint32 interrupt_state;
    Axis3f acc, gyro;
    EulerAngles angles;

    if ((gyro_calibrated != 1) || (g_imu_sample_ready == 0))
    {
        return;
    }

    interrupt_state = interrupt_global_disable();
    raw_sample = g_imu_raw_sample;
    g_imu_sample_ready = 0;
    interrupt_global_enable(interrupt_state);

    /* ==================== TEST ONLY: IMU处理计数 ==================== */
    g_imu_diag_stat.process_count++;
    /* ==================== TEST ONLY: IMU处理计数 ==================== */

    // 加速度值转换
    acc.x = -(float)raw_sample.acc_x / IMU_ACC_LSB_PER_G;
    acc.y = -(float)raw_sample.acc_y / IMU_ACC_LSB_PER_G;
    acc.z = -(float)raw_sample.acc_z / IMU_ACC_LSB_PER_G;

    // 陀螺仪值转换（先减零偏再转换单位）
    gyro.x = -((float)raw_sample.gyro_x - gyro_x_offset) / IMU_GYRO_LSB_PER_DPS;
    gyro.y = -((float)raw_sample.gyro_y - gyro_y_offset) / IMU_GYRO_LSB_PER_DPS;
    gyro.z = -((float)raw_sample.gyro_z - gyro_z_offset) / IMU_GYRO_LSB_PER_DPS;


    //IMU_DT_S时基用于积分，和采样周期一致
    imu_update(acc, gyro, IMU_DT_S);
    angles = imu_get_euler_angles(gyro, IMU_DT_S);

    roll_kalman = angles.roll;
    pitch_kalman = angles.pitch;
    yaw_kalman = angles.yaw;
    gyro_x_rate = gyro.x;
    gyro_y_rate = gyro.y;

    if (roll_kalman > 180)
    {
        roll_kalman -= 360;
    }
    if (roll_kalman < -180)
    {
        roll_kalman += 360;
    }

    /* ==================== NEW: 生成补偿后的控制角度 ====================
       为什么这里只处理 roll / pitch：
       - yaw_kalman 当前没有磁力计参考，不做这层机械零位补偿
       - gyro_x_rate / gyro_y_rate 是角速度，已经做过零偏校准，也不在这里补

       这层新增代码的作用只是：
       - 保留原始姿态输出
       - 额外生成一组扣除机械零位后的控制角度
    */
    roll_ctrl_angle = imu_normalize_deg(roll_kalman - g_roll_control_zero_deg);
    pitch_ctrl_angle = imu_normalize_deg(pitch_kalman - g_pitch_control_zero_deg);
}

/* 上位机测试 */
void imu_test(void)
{
//    usart_send(roll_kalman, pitch_kalman, yaw_kalman, 1);
    // JustFloat_Test_three(gyro_x_offset, gyro_y_offset, gyro_z_offset);
//    printf("%f,%f,%f\r\n", gyro_x_offset, gyro_y_offset, gyro_z_offset);
}
