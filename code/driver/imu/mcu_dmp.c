/*
 * mcu_dmp.c
 *
 *  Created on: 2025年5月21日
 *      Author: suiyungui
 *  Modified: v2.1 - 修复角度边界处理问题
 */

#include "mcu_dmp.h"

/* 姿态解算算法参数
 * 采用混合方案：静止时使用PI控制器，运动时使用Madgwick算法
 *
 * 零飘问题调参方案：
 * 1. 开机静止零飘
 *    - 现象：开机时IMU静止但角度缓慢漂移
 *    - 解决方案：
 *      a) 降低Ki至0.0001f-0.0003f
 *      b) 适当增大Kp至18.0f-20.0f
 *      c) 增大STATIC_THRESHOLD至0.8f左右使更容易进入静止模式
 *
 * 2. 运动后静止零飘
 *    - 现象：运动停止后角度继续漂移
 *    - 解决方案：
 *      a) 调整卡尔曼滤波参数：
 *         - 降低q至0.001f减小预测干扰
 *         - 增大r至0.2f降低测量信任度
 *      b) 增大alpha至0.9f加强滤波
 *      c) 减小MAX_YAW_DELTA至6.0f限制变化率
 *
 * 3. 快速转动后零飘
 *    - 现象：剧烈运动后回到静止状态有漂移
 *    - 解决方案：
 *      a) 降低beta至0.08f减小动态响应
 *      b) 增大STATIC_THRESHOLD至0.8f
 *      c) 调整MAX_YAW_RATE至60.0f限制角速度
 *
 * 4. 温漂问题
 *    - 现象：长时间使用后逐渐产生漂移
 *    - 解决方案：
 *      a) 完全降低Ki至0f消除积分作用
 *      b) 增大r至0.25f降低对测量的信任
 *      c) 增大alpha至0.95f加强历史数据权重
 *
 * 5. 震动环境零飘
 *    - 现象：在有震动的环境下角度漂移
 *    - 解决方案：
 *      a) 增大STATIC_THRESHOLD至1.0f
 *      b) 增大alpha至0.92f
 *      c) 增大r至0.3f
 *      d) 减小q至0.001f
 *
 * 调参优先级：
 * 1. 先调节STATIC_THRESHOLD确保静止检测准确
 * 2. 调节Kp/Ki解决静止漂移
 * 3. 调节beta解决动态性能
 * 4. 最后微调卡尔曼和低通滤波参数
 *
 * 调参技巧：
 * 1. beta: Madgwick算法增益，影响动态响应速度
 *    - 增大beta：跟踪更快，但可能更嘈杂
 *    - 减小beta：更平稳，但动态响应变慢
 *    - 建议范围：0.05f-0.15f
 *    - 大动态场景：增大至0.12f-0.15f
 *    - 精确场景：降低至0.05f-0.08f
 *
 * 2. Kp/Ki: PI控制器参数，主要影响静止性能
 *    - Kp影响跟踪速度，Ki影响静态精度
 *    - 抖动时：降低Kp，减小Ki
 *    - 漂移时：增大Kp，适当增大Ki
 *    - 建议范围：Kp(10.0f-20.0f), Ki(0.0001f-0.001f)
 */
static float beta = 0.08f;                               /* Madgwick算法增益 */
static float Kp = 15.00f;                               /* 比例增益（用于静止状态） */
static float Ki = 0.0005f;                              /* 积分增益（用于静止状态） */
static float exInt = 0.0f, eyInt = 0.0f, ezInt = 0.0f;  /* 积分误差累计 */

/* 四元数，表示当前姿态 */
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

/* 旋转矩阵，用于欧拉角计算 */
static float rMat[3][3];

/* 偏航角卡尔曼滤波器
 * 调参技巧：
 * 1. q(过程噪声)：反映系统动态特性
 *    - 增大q：响应更快，但更不稳定
 *    - 减小q：更稳定，但响应变慢
 *    - 快速转动场景：增大至0.005f
 *    - 稳定场景：减小至0.001f
 *
 * 2. r(测量噪声)：反映测量可信度
 *    - 增大r：更多依赖预测值，更平滑但可能滞后
 *    - 减小r：更多依赖测量值，响应快但可能抖动
 *    - 震动环境：增大至0.2f-0.3f
 *    - 稳定环境：减小至0.1f
 */
static KalmanFilter yaw_kf = {
    .q = 0.001f,    // 过程噪声
    .r = 0.15f,     // 测量噪声
    .x = 0.0f,      // 状态估计值
    .p = 0.1f,      // 估计误差协方差
    .k = 0.0f,      // 卡尔曼增益
    .init = 0       // 初始化标志
};
/* 状态变量 */
static float last_yaw_rate = 0.0f;
static float last_yaw = 0.0f;

/* 滤波和限制参数
 * 调参技巧：
 * 1. alpha：低通滤波系数
 *    - 增大alpha：更平滑，但滞后增加
 *    - 减小alpha：响应更快，但可能更嘈杂
 *    - 建议范围：0.7f-0.95f
 *    - 高频震动环境：增大至0.9f以上
 *    - 快速响应场景：减小至0.7f-0.8f
 *
 * 2. MAX_YAW_RATE：偏航角速度限制
 *    - 增大：允许更快的转动
 *    - 减小：更好的稳定性
 *    - 建议范围：50.0f-100.0f
 *    - 快速转动场景：增大至100.0f
 *    - 精确控制场景：减小至50.0f
 *
 * 3. MAX_YAW_DELTA：偏航角变化限制
 *    - 增大：响应更快
 *    - 减小：更好的抗干扰性
 *    - 建议范围：5.0f-15.0f
 *    - 大范围转动：增大至12.0f-15.0f
 *    - 精确控制：减小至5.0f-8.0f
 *
 * 4. STATIC_THRESHOLD：静止检测阈值
 *    - 增大：更容易进入静止模式
 *    - 减小：更容易进入运动模式
 *    - 建议范围：0.3f-1.0f
 *    - 震动环境：增大至0.8f-1.0f
 *    - 高精度需求：减小至0.3f-0.5f
 */
static const float MAX_YAW_RATE = 80.0f;    // 最大偏航角速度 (度/秒)
static const float MAX_YAW_DELTA = 8.0f;    // 最大偏航角变化 (度)
static const float STATIC_THRESHOLD = 0.03f;  // 静止检测阈值

/* 自适应滤波参数结构体 */
typedef struct {
    float alpha_min;      // 最小滤波系数
    float alpha_max;      // 最大滤波系数
    float rate_threshold; // 角速度阈值
} AdaptiveFilter;

static AdaptiveFilter adaptive_filter = {
    .alpha_min = 0.70f,     // 最小滤波系数
    .alpha_max = 0.90f,     // 最大滤波系数
    .rate_threshold = 50.0f
};

/* 角度差计算函数 - 计算两个角度的最小差值，结果在[-180, 180]范围 */
static float angle_difference(float angle1, float angle2)
{
    float diff = angle1 - angle2;
    if(diff > 180.0f) {
        diff -= 360.0f;
    } else if(diff < -180.0f) {
        diff += 360.0f;
    }
    return diff;
}

/* 角度归一化函数 - 将角度范围限制在[-180, 180] */
static float normalize_angle(float angle)
{
    while(angle > 180.0f) angle -= 360.0f;
    while(angle < -180.0f) angle += 360.0f;
    return angle;
}

/* 自适应滤波系数计算 */
float calculate_adaptive_alpha(float yaw_rate)
{
    float rate_abs = fabsf(yaw_rate);
    if(rate_abs < adaptive_filter.rate_threshold)
    {
        return adaptive_filter.alpha_max -
               (rate_abs / adaptive_filter.rate_threshold) *
               (adaptive_filter.alpha_max - adaptive_filter.alpha_min);
    }
    return adaptive_filter.alpha_min;
}

/* 快速开平方求倒函数 */
float invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

/* 卡尔曼滤波器更新 */
float kalman_filter(KalmanFilter* kf, float measurement, float gyro_rate, float dt)
{
    // 初始化
    if(!kf->init) {
        kf->x = measurement;
        kf->init = 1;
        return kf->x;
    }

    // 预测步骤
    kf->x = kf->x + gyro_rate * dt;
    kf->p = kf->p + kf->q;

    // 更新步骤
    kf->k = kf->p / (kf->p + kf->r);
    kf->x = kf->x + kf->k * (measurement - kf->x);
    kf->p = (1.0f - kf->k) * kf->p;

    return kf->x;
}

/* 初始化IMU */
void imu_init(void)
{
    q0 = 1.0f;
    q1 = q2 = q3 = 0.0f;

    // 初始化偏航角卡尔曼滤波器
    yaw_kf.x = 0.0f;
    yaw_kf.p = 0.1f;
    yaw_kf.init = 0;
}

/* 计算旋转矩阵 */
static void compute_rotation_matrix(void)
{
    float q1q1 = q1 * q1;
    float q2q2 = q2 * q2;
    float q3q3 = q3 * q3;

    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q3 = q2 * q3;

    rMat[0][0] = 1.0f - 2.0f * q2q2 - 2.0f * q3q3;
    rMat[0][1] = 2.0f * (q1q2 + -q0q3);
    rMat[0][2] = 2.0f * (q1q3 - -q0q2);

    rMat[1][0] = 2.0f * (q1q2 - -q0q3);
    rMat[1][1] = 1.0f - 2.0f * q1q1 - 2.0f * q3q3;
    rMat[1][2] = 2.0f * (q2q3 + -q0q1);

    rMat[2][0] = 2.0f * (q1q3 + -q0q2);
    rMat[2][1] = 2.0f * (q2q3 - -q0q1);
    rMat[2][2] = 1.0f - 2.0f * q1q1 - 2.0f * q2q2;
}

/* 数据融合 - 互补滤波 */
void imu_update(Axis3f acc, Axis3f gyro, float dt)
{
    float normalise;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, q0q0, q1q1, q2q2, q3q3;
    float gyro_magnitude;

    /* 角速度单位转换为弧度 */
    gyro.x = gyro.x * DEG2RAD;
    gyro.y = gyro.y * DEG2RAD;
    gyro.z = gyro.z * DEG2RAD;

    /* 计算角速度大小用于静止检测 */
    gyro_magnitude = sqrtf(gyro.x * gyro.x + gyro.y * gyro.y + gyro.z * gyro.z);

    /* 检查加速度计数据是否有效 */
    if ((acc.x != 0.0f) || (acc.y != 0.0f) || (acc.z != 0.0f))
    {
        /* 单位化加速度计数据 */
        normalise = invSqrt(acc.x * acc.x + acc.y * acc.y + acc.z * acc.z);
        acc.x *= normalise;
        acc.y *= normalise;
        acc.z *= normalise;

        /* 判断是否处于静止状态 */
        if (gyro_magnitude < STATIC_THRESHOLD) {
            /* 静止状态使用PI控制器 */
            float ex = (acc.y * rMat[2][2] - acc.z * rMat[2][1]);
            float ey = (acc.z * rMat[2][0] - acc.x * rMat[2][2]);
            float ez = (acc.x * rMat[2][1] - acc.y * rMat[2][0]);

            /* 积分误差累积 */
            exInt += Ki * ex * dt;
            eyInt += Ki * ey * dt;
            ezInt += Ki * ez * dt;

            /* 应用PI修正 */
            gyro.x += Kp * ex + exInt;
            gyro.y += Kp * ey + eyInt;
            gyro.z += Kp * ez + ezInt;
        } else {
            /* 运动状态使用Madgwick算法 */
            /* 计算四元数变化率 */
            qDot1 = 0.5f * (-q1 * gyro.x - q2 * gyro.y - q3 * gyro.z);
            qDot2 = 0.5f * (q0 * gyro.x + q2 * gyro.z - q3 * gyro.y);
            qDot3 = 0.5f * (q0 * gyro.y - q1 * gyro.z + q3 * gyro.x);
            qDot4 = 0.5f * (q0 * gyro.z + q1 * gyro.y - q2 * gyro.x);

            /* 计算梯度 */
            _2q0 = 2.0f * q0;
            _2q1 = 2.0f * q1;
            _2q2 = 2.0f * q2;
            _2q3 = 2.0f * q3;
            q0q0 = q0 * q0;
            q1q1 = q1 * q1;
            q2q2 = q2 * q2;
            q3q3 = q3 * q3;

            s0 = _2q0 * q2q2 + _2q2 * acc.x + _2q0 * q1q1 - _2q1 * acc.y;
            s1 = _2q1 * q3q3 - _2q3 * acc.x + 4.0f * q0q0 * q1 - _2q0 * acc.y - _2q1 + _2q1 * q1q1 + _2q1 * q2q2 + _2q1 * acc.z;
            s2 = 4.0f * q0q0 * q2 + _2q0 * acc.x + _2q2 * q3q3 - _2q3 * acc.y - _2q2 + _2q2 * q1q1 + _2q2 * q2q2 + _2q2 * acc.z;
            s3 = 4.0f * q1q1 * q3 - _2q1 * acc.x + 4.0f * q2q2 * q3 - _2q2 * acc.y;

            /* 归一化梯度 */
            normalise = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
            s0 *= normalise;
            s1 *= normalise;
            s2 *= normalise;
            s3 *= normalise;

            /* 应用梯度下降 */
            qDot1 -= beta * s0;
            qDot2 -= beta * s1;
            qDot3 -= beta * s2;
            qDot4 -= beta * s3;

            /* 积分得到四元数 */
            q0 += qDot1 * dt;
            q1 += qDot2 * dt;
            q2 += qDot3 * dt;
            q3 += qDot4 * dt;
        }

        /* 四元数归一化 */
        normalise = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        q0 *= normalise;
        q1 *= normalise;
        q2 *= normalise;
        q3 *= normalise;

        /* 计算旋转矩阵 */
        compute_rotation_matrix();
    }
}

typedef struct {
    float x[2];    // 状态向量 [角度, 角速度]
    float p[2][2]; // 协方差矩阵
    float q[2];    // 过程噪声
    float r;       // 测量噪声
    int init;
} ExtendedKalmanFilter;

static ExtendedKalmanFilter yaw_ekf = {
    .x = {0.0f, 0.0f},
    .p = {{0.1f, 0.0f}, {0.0f, 0.1f}},
    .q = {0.001f, 0.002f},
    .r = 0.3f,
    .init = 0
};


/* 扩展卡尔曼滤波更新 - 修复版本 */
void ekf_update(ExtendedKalmanFilter* ekf, float measurement, float dt)
{
    if(!ekf->init)
    {
        ekf->x[0] = measurement;
        ekf->init = 1;
        return;
    }

    // 预测步骤
    ekf->x[0] += ekf->x[1] * dt;
    // 预测后立即归一化，防止状态累积
    ekf->x[0] = normalize_angle(ekf->x[0]);

    // 更新协方差
    float temp_p[2][2];
    temp_p[0][0] = ekf->p[0][0] + dt * ekf->p[1][0] + dt * ekf->p[0][1] + dt * dt * ekf->p[1][1] + ekf->q[0];
    temp_p[0][1] = ekf->p[0][1] + dt * ekf->p[1][1];
    temp_p[1][0] = ekf->p[1][0] + dt * ekf->p[1][1];
    temp_p[1][1] = ekf->p[1][1] + ekf->q[1];

    // 计算卡尔曼增益
    float k[2];
    k[0] = temp_p[0][0] / (temp_p[0][0] + ekf->r);
    k[1] = temp_p[1][0] / (temp_p[0][0] + ekf->r);

    // 使用角度差值计算创新项，正确处理-180/180边界
    float innovation = angle_difference(measurement, ekf->x[0]);
    ekf->x[0] += k[0] * innovation;
    ekf->x[1] += k[1] * innovation;

    // 更新后再次归一化
    ekf->x[0] = normalize_angle(ekf->x[0]);

    // 更新协方差
    ekf->p[0][0] = (1 - k[0]) * temp_p[0][0];
    ekf->p[0][1] = (1 - k[0]) * temp_p[0][1];
    ekf->p[1][0] = temp_p[1][0] - k[1] * temp_p[0][0];
    ekf->p[1][1] = temp_p[1][1] - k[1] * temp_p[0][1];
}

/* 异常值检测 */
int is_yaw_outlier(float yaw, float last_yaw, float yaw_rate, float dt)
{
    float predicted_yaw = last_yaw + yaw_rate * dt;
    float delta = fabsf(angle_difference(yaw, predicted_yaw));
    return delta > MAX_YAW_DELTA * 1.5f;
}


/* 获取欧拉角 - 修复版本 */
EulerAngles imu_get_euler_angles(Axis3f gyro, float dt)
{
    EulerAngles angles;
    float yaw_rate;
    static float last_raw_yaw = 0.0f;
    static float last_filtered_yaw = 0.0f;
    static float filtered_yaw_rate = 0.0f;
    static uint8_t first_run = 1;

    // 从旋转矩阵中提取欧拉角
    angles.roll = atan2f(rMat[2][1], rMat[2][2]) * RAD2DEG;
    angles.pitch = asinf(-rMat[2][0]) * RAD2DEG;

    // 计算原始偏航角（已经在[-180, 180]范围内）
    float raw_yaw = atan2f(rMat[1][0], rMat[0][0]) * RAD2DEG;

    // 计算偏航角速度（使用姿态补偿）
    float cos_pitch = cosf(angles.pitch * DEG2RAD);
    float sin_pitch = sinf(angles.pitch * DEG2RAD);
    float cos_roll = cosf(angles.roll * DEG2RAD);
    float sin_roll = sinf(angles.roll * DEG2RAD);

    // 补偿后的偏航角速度
    yaw_rate = (gyro.z * cos_roll * cos_pitch +
                gyro.y * sin_roll * cos_pitch -
                gyro.x * sin_pitch) * RAD2DEG;

    // 限制偏航角速度
    if(yaw_rate > MAX_YAW_RATE) yaw_rate = MAX_YAW_RATE;
    if(yaw_rate < -MAX_YAW_RATE) yaw_rate = -MAX_YAW_RATE;

    // 低通滤波处理偏航角速度
    float adaptive_alpha = calculate_adaptive_alpha(yaw_rate);
    filtered_yaw_rate = adaptive_alpha * filtered_yaw_rate +
                    (1.0f - adaptive_alpha) * yaw_rate;

    // 首次运行初始化
    if(first_run) {
        last_raw_yaw = raw_yaw;
        last_yaw = raw_yaw;
        last_filtered_yaw = raw_yaw;
        first_run = 0;
    }

    // 使用扩展卡尔曼滤波（内部已处理角度边界）
    ekf_update(&yaw_ekf, raw_yaw, dt);
    float kf_yaw = yaw_ekf.x[0];

    // 使用角度差值进行低通滤波，避免边界跳变问题
    float delta_kf = angle_difference(kf_yaw, last_yaw);
    float temp_yaw = normalize_angle(last_yaw + (1.0f - adaptive_alpha) * delta_kf);

    // 第二级滤波
    float delta_temp = angle_difference(temp_yaw, last_filtered_yaw);
    angles.yaw = normalize_angle(last_filtered_yaw + (1.0f - adaptive_alpha) * delta_temp);

    // 更新历史值（全部归一化存储）
    last_raw_yaw = raw_yaw;
    last_yaw = temp_yaw;
    last_filtered_yaw = angles.yaw;
    last_yaw_rate = yaw_rate;

    return angles;
}

/* 根据加速度计初始化姿态（解决上电姿态为0的问题）*/
void imu_init_from_accel(Axis3f acc)
{
    // 先初始化滤波器
    q0 = 1.0f;
    q1 = q2 = q3 = 0.0f;

    // 初始化偏航角卡尔曼滤波器
    yaw_kf.x = 0.0f;
    yaw_kf.p = 0.1f;
    yaw_kf.init = 0;

    // 归一化加速度计
    float norm = invSqrt(acc.x * acc.x + acc.y * acc.y + acc.z * acc.z);
    acc.x *= norm;
    acc.y *= norm;
    acc.z *= norm;

    // 从加速度计计算初始 roll 和 pitch（假设静止，加速度计只测到重力）
    // roll: 绕X轴旋转，从Y-Z平面计算
    // pitch: 绕Y轴旋转，从X分量计算
    float roll_rad = atan2f(acc.y, acc.z);
    float pitch_rad = asinf(-acc.x);

    // 将欧拉角转换为四元数（yaw=0，因为没有磁力计）
    float cr = cosf(roll_rad * 0.5f);
    float sr = sinf(roll_rad * 0.5f);
    float cp = cosf(pitch_rad * 0.5f);
    float sp = sinf(pitch_rad * 0.5f);

    // 四元数 = Qyaw(0) * Qpitch * Qroll
    q0 = cr * cp;
    q1 = sr * cp;
    q2 = cr * sp;
    q3 = -sr * sp;

    // 更新旋转矩阵
    compute_rotation_matrix();
}
