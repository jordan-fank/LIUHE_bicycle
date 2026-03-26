# IMU 子系统分析报告

## 1. 硬件配置

### 1.1传感器型号
- **主IMU**: IMU963RA (6轴 IMU，支持加速度计 + 陀螺仪)
- **备用 IMU**: IMU660RA (在EKF实现中使用)

### 1.2 通信接口
```c
// 硬件SPI 配置 (默认)
#define IMU963RA_SPI_SPEED     (10 * 1000 * 1000)    // 10MHz SPI 时钟
#define IMU963RA_SPI           (SPI_0)               // SPI 通道0
#define IMU963RA_SPC_PIN       (SPI0_SCLK_P20_11)    // SCK 引脚
#define IMU963RA_SDI_PIN       (SPI0_MOSI_P20_14)    // MOSI 引脚
#define IMU963RA_SDO_PIN       (SPI0_MISO_P20_12)    // MISO引脚
#define IMU963RA_CS_PIN        (P20_13)              // 片选引脚
```

### 1.3 传感器量程配置
- **加速度计**: ±8g (单位转换系数：4098 LSB/g)
- **陀螺仪**: ±2000 dps (单位转换系数：14.3 LSB/°/s)

---

## 2. 软件架构

### 2.1分层架构
```
┌─────────────────────────────────────┐
│         应用层(imu_app.c)          │
│  - imu_all_init()  初始化           │
│  - imu_proc()      主处理循环       │
│  - imu_calibrate_gyro() 校准        │
└─────────────────────────────────────┘
                  ↓
┌─────────────────────────────────────┐
│       算法层(mcu_dmp.c)            │
│  - imu_update()    姿态解算         │
│  - kalman_filter() 卡尔曼滤波       │
│  - ekf_update()    扩展卡尔曼滤波   │
└─────────────────────────────────────┘
                  ↓
┌─────────────────────────────────────┐
│      驱动层(zf_device_imu963ra)    │
│  - imu963ra_init()   硬件初始化     │
│  - imu963ra_get_gyro()读取陀螺仪   │
│  - imu963ra_get_acc()  读取加速度计 │
└─────────────────────────────────────┘
```

### 2.2 数据流
```
IMU963RA传感器
    ↓ (SPI 通信)
imu963ra_get_gyro() / imu963ra_get_acc()
    ↓ (原始数据，LSB)
imu_proc()
    ↓ (减去零偏 + 单位转换)
Axis3f acc (单位：g), Axis3f gyro (单位：°/s)
    ↓
imu_update(acc, gyro, dt)
    ↓ (互补滤波 + Madgwick算法)
四元数q0, q1, q2, q3
    ↓
imu_get_euler_angles()
    ↓ (四元数转欧拉角)
roll_kalman, pitch_kalman, yaw_kalman
```

---

## 3. 核心算法

### 3.1陀螺仪校准
**文件**: [`imu_app.c`](file://e:\AURIX-v1.10.28-workspace\LIUHE_bicycle1\code\app\imu_app.c#L89-L125)

```c
// 采样30000 次，间隔 2ms，总耗时约 60秒
#define CALIBRATION_SAMPLES     30000
#define CALIBRATION_DELAY_MS    2

void imu_calibrate_gyro(void)
{
    // 累加30000次读数
    for (i = 0; i < CALIBRATION_SAMPLES; i++)
    {
        imu963ra_get_gyro();
        sum_x += imu963ra_gyro_x;
        sum_y += imu963ra_gyro_y;
        sum_z += imu963ra_gyro_z;
        system_delay_ms(CALIBRATION_DELAY_MS);
    }
    
    // 计算零偏(保持LSB单位，不转换)
    gyro_x_offset = (float)sum_x / CALIBRATION_SAMPLES;
    gyro_y_offset = (float)sum_y / CALIBRATION_SAMPLES;
    gyro_z_offset = (float)sum_z / CALIBRATION_SAMPLES;
}
```

**临时校准值** (用于快速测试):
```c
gyro_x_offset = 0.83f;
gyro_y_offset = -6.18f;
gyro_z_offset = 0.69f;
```

### 3.2姿态解算(互补滤波 + Madgwick)
**文件**: [`mcu_dmp.c`](file://e:\AURIX-v1.10.28-workspace\LIUHE_bicycle1\code\driver\imu\mcu_dmp.c#L267-L400)

#### 3.2.1静止检测
```c
// 角速度阈值检测
if (gyro_magnitude < STATIC_THRESHOLD) {
    // 静止状态：使用PI控制器修正漂移
    float ex = (acc.y * rMat[2][2] - acc.z * rMat[2][1]);
    float ey = (acc.z * rMat[2][0] - acc.x * rMat[2][2]);
    float ez = (acc.x * rMat[2][1] - acc.y * rMat[2][0]);
    
    // 积分误差
    exInt += Ki * ex * dt;
    eyInt += Ki * ey * dt;
    ezInt += Ki * ez * dt;
    
    // PI修正陀螺仪
    gyro.x += Kp * ex + exInt;
    gyro.y += Kp * ey + eyInt;
    gyro.z += Kp * ez + ezInt;
}
```

#### 3.2.2运动状态(Madgwick算法)
```c
// 计算四元数变化率
qDot1 = 0.5f * (-q1 * gyro.x - q2 * gyro.y - q3 * gyro.z);
qDot2 = 0.5f * (q0 * gyro.x + q2 * gyro.z - q3 * gyro.y);
qDot3 = 0.5f * (q0 * gyro.y - q1 * gyro.z + q3 * gyro.x);
qDot4 = 0.5f * (q0 * gyro.z + q1 * gyro.y - q2 * gyro.x);

// 梯度下降修正
qDot1 -= beta * s0;
qDot2 -= beta * s1;
qDot3 -= beta * s2;
qDot4 -= beta * s3;

// 积分得到新四元数
q0 += qDot1 * dt;
q1 += qDot2 * dt;
q2 += qDot3 * dt;
q3 += qDot4 * dt;
```

**参数**:
- `Kp = 0.5f`: 比例增益(静止时加速度计修正)
- `Ki = 0.001f`: 积分增益(消除稳态误差)
- `beta = 0.1f`: Madgwick算法梯度下降步长

### 3.3卡尔曼滤波(备用方案)
**文件**: [`mcu_dmp.c`](file://e:\AURIX-v1.10.28-workspace\LIUHE_bicycle1\code\driver\imu\mcu_dmp.c#L234-L251)

```c
typedef struct {
    float x;  // 估计值
    float p;  // 估计误差协方差
    float q;  // 过程噪声
    float r;  // 测量噪声
    float k;  // 卡尔曼增益
    int init; // 初始化标志
} KalmanFilter;

float kalman_filter(KalmanFilter* kf, float measurement, float gyro_rate, float dt)
{
    // 预测
    kf->x = kf->x + gyro_rate * dt;
    kf->p = kf->p + kf->q;
    
    // 更新
    kf->k = kf->p / (kf->p + kf->r);
    kf->x = kf->x + kf->k * (measurement - kf->x);
    kf->p = (1.0f - kf->k) * kf->p;
    
    return kf->x;
}
```

### 3.4 扩展卡尔曼滤波(EKF)
**文件**: [`ekf.c`](file://e:\AURIX-v1.10.28-workspace\LIUHE_bicycle1\code\driver\imu\ekf.c)

使用四元数作为状态变量的EKF 实现:
- **状态向量**: `[q0, q1, q2, q3]` (四元数)
- **观测向量**: `[acc_x, acc_y, acc_z]` (归一化加速度)
- **状态转移矩阵**: 基于陀螺仪角速度的四元数微分方程
- **观测矩阵**: 四元数到重力向量的投影

---

## 4. 任务调度

### 4.1 当前调度配置
**文件**: [`schedule.c`](file://e:\AURIX-v1.10.28-workspace\LIUHE_bicycle1\code\app\schedule.c)

```c
static task_t scheduler_task[] = {
    {key_scan,      10,  0},  // 按键扫描 10ms
    {key_task,      10,  0},  // 按键处理 10ms
    {ips_app_task,  200, 0},  // 屏幕刷新200ms
};
```

**问题**: IMU 处理任务(`imu_proc()`) **未注册到调度器**！

### 4.2 实际执行路径
通过全局搜索发现，`imu_proc()` 可能在以下位置被调用:
1. **主循环直接调用** (需在 [`cpu0_main.c`](file://e:\AURIX-v1.10.28-workspace\LIUHE_bicycle1\user\cpu0_main.c)中确认)
2. **PIT 定时器中断** (需在 [`isr.c`](file://e:\AURIX-v1.10.28-workspace\LIUHE_bicycle1\user\isr.c) 中确认)
3. **其他应用层任务间接调用**

**建议**: 将IMU 任务添加到调度器，推荐周期 **5ms** (200Hz)，与`dt=0.005f` 匹配。

---

## 5. 数据输出

### 5.1全局变量接口
```c
extern float roll_kalman;    // 横滚角(度)
extern float pitch_kalman;   // 俯仰角(度)
extern float yaw_kalman;     // 偏航角(度)
extern float gyro_x_rate;    // X 轴角速度 (°/s)
extern float gyro_y_rate;    // Y轴角速度(°/s)
```

### 5.2上位机通信协议
**文件**: [`imu_app.c`](file://e:\AURIX-v1.10.28-workspace\LIUHE_bicycle1\code\app\imu_app.c#L26-L78)

数据帧格式(匿名飞控协议):
```
帧头(0xAB) + 源地址(0xDC) + 目标地址(0xFE) + 功能码(0x03) + 数据长度(7)
+ Roll(int16, ×100) + Pitch(int16, ×100) + Yaw(int16, ×100)
+ 融合状态 (uint8) + 校验和 + 附加校验
```

---

## 6. 关键问题与改进建议

### 6.1架构问题
1. **IMU任务未纳入调度管理**
   - 当前`imu_proc()` 调用位置不明确
   - 建议：添加到[`scheduler_task[]`](file://e:\AURIX-v1.10.28-workspace\LIUHE_bicycle1\code\app\schedule.c#L14-L20)，周期 5ms

2. **双 IMU实现并存**
   - `mcu_dmp.c`使用 IMU963RA
   - `ekf.c` 使用IMU660RA
   - 建议：统一传感器，删除冗余代码

3. **卡尔曼滤波未实际使用**
   - `kalman_filter()`函数已实现但未在 `imu_proc()` 中调用
   - 当前使用互补滤波 + Madgwick
   - 建议：评估是否需要EKF，避免代码冗余

### 6.2性能优化建议
1. **减少校准时间**
   - 当前30000 次采样 × 2ms = 60 秒，过长
   - 建议：降至 5000 次(10 秒) 或使用出厂校准值

2. **浮点运算优化**
   - 大量使用`sqrtf()`, `atan2f()`, `asinf()`等库函数
   - 建议：在性能关键路径使用近似算法(如快速平方根倒数)

3. **内存优化**
   - 四元数、旋转矩阵、卡尔曼状态均为静态分配
   - 建议：检查RAM 占用，避免栈溢出

### 6.3 功能增强建议
1. **添加温度补偿**
   - IMU963RA 内置温度传感器
   - 建议：建立零偏 - 温度模型，提升精度

2. **磁强计融合** (如使用9 轴模式)
   - 当前仅使用 6 轴(加速度计 + 陀螺仪)
   - 建议：添加磁力计，修正偏航角漂移

3. **振动检测**
   - 通过加速度计高频分量检测电机振动
   - 建议：动态调整`beta` 参数，运动时降低加速度计权重

---

## 7. 调试指南

### 7.1 零漂校准流程
```c
// 1. 上电后保持车辆静止
// 2. 调用校准函数
imu_calibrate_gyro();  // 60秒完整版
// 或
imu_calibrate_gyro_temp();  // 使用临时值快速测试

// 3. 打印零偏值
printf("Gyro Offset: %f, %f, %f\r\n", 
       gyro_x_offset, gyro_y_offset, gyro_z_offset);
```

### 7.2姿态数据读取
```c
// 在控制循环中调用
imu_proc();  // 更新姿态

// 读取欧拉角
float roll  = roll_kalman;   // [-180, 180]
float pitch = pitch_kalman;  // [-90, 90]
float yaw   = yaw_kalman;    // [-180, 180]
```

### 7.3 上位机监控
取消注释[`imu_test()`](file://e:\AURIX-v1.10.28-workspace\LIUHE_bicycle1\code\app\imu_app.c#L180-L184) 中的发送函数:
```c
void imu_test(void)
{
    usart_send(roll_kalman, pitch_kalman, yaw_kalman, 1);
}
```

使用匿名飞控上位机查看实时波形。

---

## 8. 相关文件索引

| 文件路径 | 功能描述 |
|---------|---------|
| [`code/app/imu_app.c`](file://e:\AURIX-v1.10.28-workspace\LIUHE_bicycle1\code\app\imu_app.c) | IMU 应用层接口 |
| [`code/app/imu_app.h`](file://e:\AURIX-v1.10.28-workspace\LIUHE_bicycle1\code\app\imu_app.h) | IMU全局变量声明 |
| [`code/driver/imu/mcu_dmp.c`](file://e:\AURIX-v1.10.28-workspace\LIUHE_bicycle1\code\driver\imu\mcu_dmp.c) | 姿态解算核心算法 |
| [`code/driver/imu/mcu_dmp.h`](file://e:\AURIX-v1.10.28-workspace\LIUHE_bicycle1\code\driver\imu\mcu_dmp.h) | 算法数据结构定义 |
| [`code/driver/imu/ekf.c`](file://e:\AURIX-v1.10.28-workspace\LIUHE_bicycle1\code\driver\imu\ekf.c) | 扩展卡尔曼滤波实现 |
| [`code/driver/imu/ekf.h`](file://e:\AURIX-v1.10.28-workspace\LIUHE_bicycle1\code\driver\imu\ekf.h) | EKF接口声明 |
| [`libraries/zf_device/zf_device_imu963ra.h`](file://e:\AURIX-v1.10.28-workspace\LIUHE_bicycle1\libraries\zf_device\zf_device_imu963ra.h) | IMU驱动配置 |
| [`code/app/schedule.c`](file://e:\AURIX-v1.10.28-workspace\LIUHE_bicycle1\code\app\schedule.c) | 任务调度器 |

---

## 9. 总结

本项目IMU子系统采用**IMU963RA六轴传感器**，通过**硬件SPI**通信，使用**互补滤波 + Madgwick 算法**进行姿态解算，输出**横滚、俯仰、偏航三轴欧拉角**。

**优点**:
- ✅ 算法成熟(Madgwick + 互补滤波)
- ✅ 支持静止/运动状态自适应
- ✅ 提供陀螺仪零漂校准功能
- ✅ 预留EKF实现 (可选)

**待改进**:
- ⚠️ IMU任务未纳入调度管理
- ⚠️ 双 IMU代码并存，需统一
- ⚠️ 校准时间过长(60秒)
- ⚠️ 缺少温度补偿和磁强计融合

**建议优先级**:
1. **高**: 将`imu_proc()` 添加到调度器(5ms周期)
2. **中**: 删除`ekf.c` 冗余代码(如不使用)
3. **低**: 优化校准时间，添加温度补偿
