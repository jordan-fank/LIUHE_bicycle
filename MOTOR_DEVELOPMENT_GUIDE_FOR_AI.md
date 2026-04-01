# TC264 控制 CYT2BL3 无刷 FOC 电机开发指南

**文档类型**: AI 助手技术培训文档  
**适用项目**: suiyungui_example, LIUHE_bicycle1  
**硬件平台**: Infineon TC264 (主控) + CYT2BL3 (双路 FOC 无刷驱动)  
**最后更新**: 2026-03-31  

---

## 📖 一、系统架构概览

### 1.1 硬件拓扑

```
┌─────────────────┐         UART @460800bps        ┌──────────────────┐
│   TC264 MCU     │◄────── TX: P02_3 / RX: P02_2──►│  CYT2BL3 Driver  │
│  (AURIX 架构)    │                                │  (双路 FOC 控制器)  │
│                 │                                │                  │
│  - 用户应用层    │                                │  - 电机 A (左轮)   │
│  - LQR 平衡算法   │                                │  - 电机 B (右轮)   │
│  - 运动控制      │                                │                  │
└─────────────────┘                                └──────────────────┘
                                                        │
                                              ┌─────────┴─────────┐
                                              ▼                   ▼
                                       ┌──────────┐       ┌──────────┐
                                       │ MENC15A  │       │ MENC15A  │
                                       │ 磁编码器  │       │ 磁编码器  │
                                       │ (左侧)    │       │ (右侧)    │
                                       └──────────┘       └──────────┘
```

### 1.2 软件分层架构

```
┌─────────────────────────────────────────┐
│         应用层 (Application Layer)       │
│  motor_app.c/h  ← 你在这里工作           │
│  - motor_init()                         │
│  - motor_set(duty)                      │
│  - motor_get_raw_rpm()                  │
│  - motor_get_vehicle_rpm()              │
│  - motor_get_vehicle_speed_m_s()        │
│  - motor_get_voltage() [待实现]          │
└─────────────────────────────────────────┘
              ▲
              │ 调用
              ▼
┌─────────────────────────────────────────┐
│         驱动层 (Driver Layer)            │
│  small_driver_uart_control.c/h          │
│  - uart_control_callback()              │
│  - small_driver_set_duty(L, R)          │
│  - small_driver_get_speed()             │
│  - small_driver_request_voltage() [...] │
└─────────────────────────────────────────┘
              ▲
              │ 使用
              ▼
┌─────────────────────────────────────────┐
│      逐飞库 (ZF Libraries)               │
│  zf_driver/uart/                        │
│  - uart_init()                          │
│  - uart_write_buffer()                  │
│  - uart_query_byte()                    │
└─────────────────────────────────────────┘
```

---

## 🔧 二、核心概念与数据类型

### 2.1 占空比 (Duty Cycle)

**定义**: 控制电机输出功率的百分比参数

| 参数项 | 说明 |
|--------|------|
| **数据类型** | `int16` (有符号 16 位整数) |
| **取值范围** | `-10000 ~ +10000` |
| **物理意义** | `-10000 = -100%`(全速反转), `0 = 停止`, `+10000 = +100%`(全速正转) |
| **映射关系** | `duty_float = duty_int / 10000.0` (范围：-1.0 ~ +1.0) |
| **实际效果** | 占空比 → PWM 输出电压 → 电机转速 (非线性，依赖负载、电池电压、KV 值) |

**示例代码**:
```c
// 设置左右轮都以前进 50% 功率运行
motor_set(5000);  // 内部调用 small_driver_set_duty(5000, 5000)

// 差速转向：左转 (左轮慢，右轮快)
small_driver_set_duty(3000, 7000);

// 原地左转：左轮反转，右轮正转
small_driver_set_duty(-4000, 4000);
```

---

### 2.2 速度反馈 (Speed Feedback)

**定义**: 从 MENC15A 磁性编码器读取的实际转速

| 参数项 | 说明 |
|--------|------|
| **数据类型** | `int16` (有符号 16 位整数) |
| **单位** | **RPM** (Revolutions Per Minute, 转/分钟) |
| **数据来源** | MENC15A 硬件速度寄存器 (原始 rad/s) → ×1.917476 → RPM |
| **信号处理** | 一阶低通滤波：`y[n] = (19*y[n-1] + x[n]) / 20` (时间常数~20ms) |
| **更新频率** | CYT2BL3 每 **10ms** 自动发送一次速度遥测数据 |
| **正负定义** | 正数 = 电机正转 (顺时针)，负数 = 电机反转 (逆时针) |

**速度计算链**:
```c
// CYT2BL3 侧 (motor_control.c)
int16 menc15a_raw = read_encoder_speed_register();  // 单位：rad/s
float speed_rpm = menc15a_raw * 1.917476f;          // 转换为 RPM
motor_speed_filter = (old_filter * 19 + speed_rpm) / 20;  // 低通滤波

// TC264 侧接收 (small_driver_uart_control.c)
// UART 中断回调解析后存入:
motor_value.receive_left_speed_data = (int16)parsed_from_uart;  // 单位：RPM

// 用户获取原始 RPM
int16 speed_rpm = motor_get_raw_rpm();  // 返回左侧电机 RPM（保留驱动原始符号）
```

**典型数值范围**:
- 静止：`0 ± 50 RPM` (传感器噪声)
- 低速巡航：`500 ~ 1500 RPM`
- 高速运行：`2000 ~ 4000 RPM`
- 最大理论速度：取决于电机 KV 值和电池电压 (例：12V×800KV = 9600 RPM 无载)

---

### 2.3 UART 通信协议

**物理层**:
- 波特率：**460800 bps**
- 数据线：TX=P02_3, RX=P02_2 (UART1)
- 帧格式：N81 (无校验，8 数据位，1 停止位)

**协议帧结构** (固定 7 字节):
```
Byte 0:  帧头 (固定 0xA5)
Byte 1:  功能字 (命令类型)
Byte 2-3: 数据高位 + 低位 (大端序)
Byte 4-5: 数据高位 + 低位 (大端序，用于双电机)
Byte 6:  校验和 (Byte0 + Byte1 + ... + Byte5 的低 8 位)
```

**已实现命令集**:

| 功能字 | 命令名 | 用途 | 数据区含义 | 响应方式 |
|--------|--------|------|------------|----------|
| `0x01` | `SET_DUTY` | 设置占空比 | Byte2-3: 左轮，Byte4-5: 右轮 | 无 ACK，立即执行 |
| `0x02` | `GET_SPEED` | 获取速度 | 无数据 (仅触发) | 周期发送 (10ms) |
| `0x03` | `SET_ZERO` | 编码器零位校准 | 无数据 | 无 ACK，自校准 |
| `0x0A` | `GET_VOLTAGE` | 获取电池电压 | 无数据 | 周期发送 (待实现) |

**校验和计算示例**:
```c
uint8 frame[7] = {0xA5, 0x01, 0x13, 0x88, 0x13, 0x88, 0x00};
// 假设左右占空比都是 5000 (0x1388)
frame[6] = 0xA5 + 0x01 + 0x13 + 0x88 + 0x13 + 0x88;  // 取低 8 位
```

---

## 💻 三、API 参考手册

### 3.1 初始化函数

#### `void motor_init(void)`
**功能**: 初始化电机驱动相关硬件 (UART 串口)  
**调用时机**: 系统上电后，在 `cpu0_main.c`的`main()` 中最早调用  
**示例**:
```c
int main(void)
{
    sys_init();           // 系统初始化
    motor_init();         // ← 电机驱动初始化
    lqr_balance_init();   // 再初始化上层控制算法
    while(1) {
        // 主循环
    }
}
```

---

### 3.2 运动控制函数

#### `void motor_set(int32_t duty)`
**功能**: 设置左右电机的占空比 (对称输出)  
**参数**: 
- `duty`: 占空比值，范围 `-10000 ~ +10000`
  - 正值 = 前进/正转
  - 负值 = 后退/反转
  - 绝对值越大 = 速度越快

**限幅保护**: 内部自动限制在 `±5000` (防止过冲)  
**示例**:
```c
// 前进 50% 功率
motor_set(5000);

// 后退 30% 功率
motor_set(-3000);

// 停车
motor_set(0);
```

---

#### `void small_driver_set_duty(int16 left_duty, int16 right_duty)`
**功能**: 独立设置左右轮占空比 (差速转向)  
**参数**:
- `left_duty`: 左轮占空比 (`-10000 ~ +10000`)
- `right_duty`: 右轮占空比 (`-10000 ~ +10000`)

**示例**:
```c
// 直线前进
small_driver_set_duty(5000, 5000);

// 右转 (左轮快，右轮慢)
small_driver_set_duty(7000, 3000);

// 原地右转 (左轮正转，右轮反转)
small_driver_set_duty(4000, -4000);
```

---

### 3.3 传感器读取函数

#### `int16 motor_get_raw_rpm(void)`
**功能**: 获取左侧电机的实时速度反馈  
**返回值**: 
- 单位：**RPM** (转/分钟)
- 正负号表示驱动原始旋转方向
- 刷新率：10ms (100Hz)

**使用场景**:
```c
// 速度闭环控制
float actual_rpm = motor_get_vehicle_rpm();
float error = target_rpm - actual_rpm;
pid_update(&speed_pid, error);

// 超速保护
if(motor_get_raw_rpm() > 4000) {
    motor_set(0);  // 紧急停车
}
```

---

#### `float motor_get_vehicle_rpm(void)`
**功能**: 获取已经按车体方向统一后的电机转速  
**返回值**:
- 单位：**RPM**
- 约定：**车辆前进为正，后退为负**

**说明**:
- `motor_get_raw_rpm()` 保留驱动原始符号
- `motor_get_vehicle_rpm()` 在应用层做一次符号统一，避免上层控制模块各自乘 `-1`

---

#### `float motor_get_vehicle_speed_m_s(void)`
**功能**: 获取带方向的线速度  
**返回值**:
- 单位：**m/s**
- 约定：**车辆前进为正，后退为负**

---

#### `float motor_get_vehicle_speed_abs_m_s(void)`
**功能**: 获取速度大小  
**返回值**:
- 单位：**m/s**
- 恒正

**使用建议**:
- 只关心速度大小的控制器，例如当前 LQR 速度调度，可以直接用它
- 需要区分前进/后退方向时，应该使用 `motor_get_vehicle_rpm()` 或 `motor_get_speed_m_s()`

---

#### `int16 motor_get_right_speed(void)` *(待实现)*
**功能**: 获取右侧电机的实时速度反馈  
**实现提示**: 读取 `motor_value.receive_right_speed_data`  
**TODO**: 需要在 `motor_app.h/c`中添加此函数

---

## 🛠️ 四、常见开发任务示例

### 4.1 任务 1: 添加电池电压监测 ⭐⭐⭐ (高优先级)

**背景**: 当前项目缺少低压保护，可能导致电池过放损坏

**需要修改的文件**:
1. [`small_driver_uart_control.h`](e:\AURIX-v1.10.28-workspace\suiyungui_example\code\driver\motor\small_driver_uart_control.h)
2. [`small_driver_uart_control.c`](e:\AURIX-v1.10.28-workspace\suiyungui_example\code\driver\motor\small_driver_uart_control.c)
3. [`motor_app.h`](e:\AURIX-v1.10.28-workspace\suiyungui_example\code\app\motor_app.h)
4. [`motor_app.c`](e:\AURIX-v1.10.28-workspace\suiyungui_example\code\app\motor_app.c)

**实现步骤**:

**Step 1**: 扩展数据结构 (small_driver_uart_control.h)
```c
typedef struct
{
    uint8 send_data_buffer[7];
    uint8 receive_data_buffer[7];
    uint8 receive_data_count;
    uint8 sum_check_data;
    
    int16 receive_left_speed_data;
    int16 receive_right_speed_data;
    
    // ===== 新增字段 =====
    float receive_battery_voltage;      // 电池电压 (单位：V)
    uint8 voltage_valid_flag;           // 电压数据有效标志 (0=无效，1=有效)
    
} small_device_value_struct;
```

**Step 2**: 添加电压请求函数 (small_driver_uart_control.c)
```c
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     无刷驱动 请求电池电压
// 参数说明     void
// 返回参数     void
// 使用示例     small_driver_request_voltage();
// 备注信息     建议每 100ms 调用一次
//-------------------------------------------------------------------------------------------------------------------
void small_driver_request_voltage(void)
{
    motor_value.send_data_buffer[0] = 0xA5;                                         // 帧头
    motor_value.send_data_buffer[1] = 0X0A;                                         // GET_VOLTAGE 功能字
    motor_value.send_data_buffer[2] = 0x00;                                         // 数据位清空
    motor_value.send_data_buffer[3] = 0x00;
    motor_value.send_data_buffer[4] = 0x00;
    motor_value.send_data_buffer[5] = 0x00;
    motor_value.send_data_buffer[6] = 0xAF;                                         // 校验和：0xA5+0x0A=0xAF
    
    uart_write_buffer(SMALL_DRIVER_UART, motor_value.send_data_buffer, 7);
}
```

**Step 3**: 在回调中解析电压数据 (small_driver_uart_control.c)
```c
void uart_control_callback(void)
{
    // ... 现有代码 ...
    
    if(motor_value.receive_data_buffer[1] == 0x02)  // 速度数据
    {
        motor_value.receive_left_speed_data  = (((int)motor_value.receive_data_buffer[2] << 8) | 
                                                 (int)motor_value.receive_data_buffer[3]);
        motor_value.receive_right_speed_data = (((int)motor_value.receive_data_buffer[4] << 8) | 
                                                 (int)motor_value.receive_data_buffer[5]);
    }
    
    // ===== 新增：电压数据解析 =====
    if(motor_value.receive_data_buffer[1] == 0x0A)  // GET_VOLTAGE 响应
    {
        // 假设电压以 0.1V 为单位传输 (需要根据实际协议调整)
        uint16 voltage_raw = (((uint16)motor_value.receive_data_buffer[2] << 8) | 
                               motor_value.receive_data_buffer[3]);
        motor_value.receive_battery_voltage = voltage_raw * 0.1f;  // 转换为伏特
        motor_value.voltage_valid_flag = 1;  // 标记数据有效
    }
    
    // ... 剩余代码 ...
}
```

**Step 4**: 暴露 API 给应用层 (motor_app.h)
```c
// 新增函数声明
float motor_get_battery_voltage(void);   // 获取电池电压 (单位：V)
uint8 motor_is_voltage_low(void);        // 判断是否低压 (<10.5V 返回 1)
void motor_start_voltage_monitoring(void); // 启动电压监测 (定时器调用)
```

**Step 5**: 实现应用层接口 (motor_app.c)
```c
float motor_get_battery_voltage(void)
{
    return motor_value.receive_battery_voltage;
}

uint8 motor_is_voltage_low(void)
{
    const float LOW_VOLTAGE_THRESHOLD = 10.5f;  // 低压报警阈值 (根据电池配置调整)
    
    if(!motor_value.voltage_valid_flag) {
        return 0;  // 数据无效时不报警
    }
    
    return (motor_value.receive_battery_voltage < LOW_VOLTAGE_THRESHOLD) ? 1 : 0;
}

// 建议在定时器中周期性调用 (如 100ms 一次)
void motor_voltage_monitor_task(void)
{
    small_driver_request_voltage();
    
    if(motor_is_voltage_low()) {
        // 低压保护逻辑
        static uint8 warning_count = 0;
        warning_count++;
        
        if(warning_count >= 10) {  // 连续 1 秒低压则强制停车
            motor_set(0);
            warning_count = 0;
        }
    } else {
        warning_count = 0;
    }
}
```

---

### 4.2 任务 2: 实现速度闭环 PID 控制

**背景**: 开环占空比控制在负载变化时速度不稳定

**控制框图**:
```
期望速度 (RPM) ──→ [PID 控制器] ──→ 占空比输出 ──→ [电机] ──→ 实际速度
                     ↑                                        │
                     └────────── 速度反馈 ◄───────────────────┘
```

**代码框架**:
```c
// motor_pid.h
typedef struct {
    float kp, ki, kd;           // PID 参数
    float integral;             // 积分累积
    float last_error;           // 上次误差
    float output_max;           // 输出限幅
    float integral_max;         // 积分抗饱和限幅
} pid_controller_t;

typedef struct {
    pid_controller_t pid;
    float target_speed;         // 目标速度 (RPM)
    float actual_speed;         // 实际速度 (RPM)
    int16 duty_output;          // PID输出的占空比
} motor_speed_loop_t;

void motor_speed_loop_init(void);
void motor_speed_loop_update(float target_rpm);
```

```c
// motor_pid.c
#include "motor_pid.h"

static motor_speed_loop_t g_speed_loop;

void motor_speed_loop_init(void)
{
    // 初始化 PID 参数 (需要根据实际车辆调优)
    g_speed_loop.pid.kp = 2.0f;
    g_speed_loop.pid.ki = 0.1f;
    g_speed_loop.pid.kd = 0.05f;
    
    g_speed_loop.pid.integral = 0;
    g_speed_loop.pid.last_error = 0;
    g_speed_loop.pid.output_max = 5000;        // 最大占空比输出
    g_speed_loop.pid.integral_max = 1000;      // 积分抗饱和
    
    g_speed_loop.target_speed = 0;
    g_speed_loop.actual_speed = 0;
    g_speed_loop.duty_output = 0;
}

void motor_speed_loop_update(float target_rpm)
{
    g_speed_loop.target_speed = target_rpm;
    g_speed_loop.actual_speed = (float)motor_get_speed();
    
    // 计算误差
    float error = g_speed_loop.target_speed - g_speed_loop.actual_speed;
    
    // 比例项
    float p_term = g_speed_loop.pid.kp * error;
    
    // 积分项 (带抗饱和)
    g_speed_loop.pid.integral += error;
    if(g_speed_loop.pid.integral > g_speed_loop.pid.integral_max) {
        g_speed_loop.pid.integral = g_speed_loop.pid.integral_max;
    } else if(g_speed_loop.pid.integral < -g_speed_loop.pid.integral_max) {
        g_speed_loop.pid.integral = -g_speed_loop.pid.integral_max;
    }
    float i_term = g_speed_loop.pid.ki * g_speed_loop.pid.integral;
    
    // 微分项
    float d_term = g_speed_loop.pid.kd * (error - g_speed_loop.pid.last_error);
    
    // 计算总输出
    float pid_output = p_term + i_term + d_term;
    
    // 输出去饱和
    if(pid_output > g_speed_loop.pid.output_max) {
        pid_output = g_speed_loop.pid.output_max;
    } else if(pid_output < -g_speed_loop.pid.output_max) {
        pid_output = -g_speed_loop.pid.output_max;
    }
    
    g_speed_loop.duty_output = (int16)pid_output;
    
    // 更新历史状态
    g_speed_loop.pid.last_error = error;
    
    // 应用到电机
    motor_set(g_speed_loop.duty_output);
}
```

**使用方法**:
```c
// cpu0_main.c 或 scheduler 中
void control_loop_task(void)
{
    static float target_speed = 1000.0f;  // 目标 1000 RPM
    
    // 每 10ms 调用一次速度环
    motor_speed_loop_update(target_speed);
}
```

---

### 4.3 任务 3: 编码器零位校准流程

**背景**: 长时间运行后编码器可能偏移，导致速度反馈不准

**校准时机**:
- 新车组装完成后
- 更换电机或编码器后
- 发现速度反馈异常时

**实现代码**:
```c
// motor_app.h
void motor_calibrate_zero_position(void);   // 启动零位校准
uint8 motor_is_calibrating(void);           // 查询是否在 calibration 中

// motor_app.c
static uint8 g_is_calibrating = 0;

void motor_calibrate_zero_position(void)
{
    g_is_calibrating = 1;
    
    // 发送 SET_ZERO 命令
    motor_value.send_data_buffer[0] = 0xA5;
    motor_value.send_data_buffer[1] = 0x03;  // SET_ZERO
    motor_value.send_data_buffer[2] = 0x00;
    motor_value.send_data_buffer[3] = 0x00;
    motor_value.send_data_buffer[4] = 0x00;
    motor_value.send_data_buffer[5] = 0x00;
    motor_value.send_data_buffer[6] = 0xA8;  // 校验和：0xA5+0x03=0xA8
    
    uart_write_buffer(SMALL_DRIVER_UART, motor_value.send_data_buffer, 7);
    
    // 等待校准完成 (通常 1-2 秒)
    // 注意：此处不应阻塞，应通过状态机或标志位查询
    
    delay_ms(2000);  // 简单延时等待 (实际应用中建议用非阻塞方式)
    
    g_is_calibrating = 0;
}

uint8 motor_is_calibrating(void)
{
    return g_is_calibrating;
}
```

**注意事项**:
- 校准时车辆必须**静止不动**
- 校准过程中不要发送运动指令
- 校准成功后会保存到 Flash，断电不丢失

---

## 🚨 五、故障排查指南

### 5.1 常见问题速查表

| 现象 | 可能原因 | 排查方法 |
|------|----------|----------|
| 电机不转 | 1. UART 接线错误<br>2. 驱动器未供电<br>3. 使能信号未拉高 | 1. 用示波器检查 TX/RX波形<br>2. 测量驱动器 VCC电压<br>3. 检查 EN引脚电平 |
| 速度始终为 0 | 1. 编码器线松动<br>2. 电机轴未转动<br>3. 协议解析错误 | 1. 重新插拔编码器线<br>2. 手转电机看是否有反馈<br>3. 单步调试回调函数 |
| 速度跳变剧烈 | 1. 电磁干扰<br>2. 接地不良<br>3. 滤波器失效 | 1. 远离动力线布线<br>2. 确保共地连接<br>3. 检查滤波系数 |
| 车体抖动 | 1. PID 参数过大<br>2. 机械间隙<br>3. 编码器安装偏心 | 1. 减小 Kp/Kd<br>2. 紧固螺丝<br>3. 重新校准零位 |
| 低压误报 | 1. 电池老化<br>2. 接触电阻大<br>3. 阈值设置过低 | 1. 万用表实测对比<br>2. 清洁触点<br>3. 调整阈值 |

---

### 5.2 调试技巧

**技巧 1: 打印关键变量**
```c
// 在 ISR 或主循环中
printf("Speed: %d RPM, Duty: %d, Voltage: %.2fV\n", 
       motor_get_speed(), 
       g_current_duty, 
       motor_get_battery_voltage());
```

**技巧 2: 使用逻辑分析仪抓取 UART 波形**
- 捕获 TX/RX 信号
- 解码 7 字节帧结构
- 验证校验和是否正确

**技巧 3: 注入测试信号**
```c
// 模拟速度输入 (测试控制算法而不实际运行电机)
#ifdef DEBUG_SIMULATION
    motor_value.receive_left_speed_data = 1000;  // 假装有 1000 RPM
#else
    // 正常从 UART 读取
#endif
```

---

## 📝 六、最佳实践清单

### ✅ DO (应该做的)

1. **每次上电先初始化**
   ```c
   motor_init();  // 必须在 main() 最开始调用
   ```

2. **定期检查电池电压**
   ```c
   // 在调度器中每 100ms 调用
   if(motor_is_voltage_low()) {
       emergency_stop();
   }
   ```

3. **软启动避免冲击**
   ```c
   // 不要直接从 0 跳到 5000
   for(int16 duty = 0; duty <= 5000; duty += 100) {
       motor_set(duty);
       delay_ms(10);
   }
   ```

4. **停车时逐渐减速**
   ```c
   for(int16 duty = current_duty; duty >= 0; duty -= 100) {
       motor_set(duty);
       delay_ms(10);
   }
   ```

5. **使用看门狗监控通信**
   ```c
   // 如果超过 50ms 没收到速度数据，认为通信故障
   static uint16 com_timeout = 0;
   if(speed_updated) {
       com_timeout = 0;
   } else {
       com_timeout++;
       if(com_timeout > 5) {  // 50ms超时
           motor_set(0);  // 紧急停车
       }
   }
   ```

---

### ❌ DON'T (不应该做的)

1. **不要在 ISR 中执行复杂计算**
   ```c
   // 错误示范
   void uart_isr(void) {
       uart_control_callback();
       lqr_balance_update();  // ❌ 耗时太长
       printf("Debug");       // ❌ 阻塞操作
   }
   
   // 正确做法
   void uart_isr(void) {
       uart_control_callback();  // ✓ 只解析数据，设置标志位
       g_speed_data_ready = 1;
   }
   
   void main_loop(void) {
       if(g_speed_data_ready) {
           lqr_balance_update();  // ✓ 在主循环处理
       }
   }
   ```

2. **不要直接修改驱动层代码**
   - 所有自定义代码放在 `code/app/`目录
   - 通过封装 API 间接调用底层驱动

3. **不要忽略限幅保护**
   ```c
   // 错误：直接使用未经验证的输入
   motor_set(user_input);  // ❌ 如果 user_input=20000 会怎样？
   
   // 正确：总是做边界检查
   if(user_input > 10000) user_input = 10000;
   if(user_input < -10000) user_input = -10000;
   motor_set(user_input);
   ```

4. **不要在中断中调用延时函数**
   ```c
   // 绝对禁止
   void isr_handler(void) {
       delay_ms(100);  // ❌ 会阻塞其他中断
   }
   ```

---

## 🔬 七、进阶主题

### 7.1 多模式控制切换

```c
typedef enum {
    MODE_OPENLOOP_DUTY = 0,     // 开环占空比 (默认)
    MODE_SPEED_CLOSED_LOOP,     // 速度闭环
    MODE_POSITION_CONTROL,      // 位置伺服
    MODE_CURRENT_TORQUE         // 电流/力矩控制
} motor_control_mode_t;

static motor_control_mode_t g_current_mode = MODE_OPENLOOP_DUTY;

void motor_set_control_mode(motor_control_mode_t mode)
{
    g_current_mode = mode;
    
    switch(mode) {
        case MODE_OPENLOOP_DUTY:
            // 清除积分器，避免模式切换冲击
            pid_reset_integral(&g_speed_loop.pid);
            break;
            
        case MODE_SPEED_CLOSED_LOOP:
            // 初始化速度环
            motor_speed_loop_init();
            break;
            
        // ... 其他模式初始化 ...
            
        default:
            break;
    }
}

void motor_control_dispatch(float command)
{
    switch(g_current_mode) {
        case MODE_OPENLOOP_DUTY:
            motor_set((int16)command);  // command 直接作为占空比
            break;
            
        case MODE_SPEED_CLOSED_LOOP:
            motor_speed_loop_update(command);  // command 作为目标 RPM
            break;
            
        // ... 其他模式处理 ...
            
        default:
            motor_set(0);
            break;
    }
}
```

---

### 7.2 数据记录与分析

**记录关键变量到 SD 卡或上位机**:
```c
typedef struct {
    uint32 timestamp_ms;
    int16 target_rpm;
    int16 actual_vehicle_rpm;
    int16 duty_output;
    float battery_voltage;
    uint8 fault_code;
} motor_log_entry_t;

void motor_log_save(motor_log_entry_t* entry)
{
    // 写入 SD 卡或通过 USB 发送到 PC
    fprintf(sd_card_file, "%lu,%d,%d,%d,%.2f,%d\n",
            entry->timestamp_ms,
            entry->target_speed,
            entry->actual_speed,
            entry->duty_output,
            entry->battery_voltage,
            entry->fault_code);
}

// 在控制循环中调用
void control_loop_with_logging(void)
{
    static uint32 log_counter = 0;
    
    motor_log_entry_t log = {
        .timestamp_ms = get_system_time_ms(),
        .target_rpm = g_target_speed,
        .actual_vehicle_rpm = (int16)motor_get_vehicle_rpm(),
        .duty_output = g_current_duty,
        .battery_voltage = motor_get_battery_voltage(),
        .fault_code = get_fault_status()
    };
    
    motor_log_save(&log);
    
    // 每 10ms 记录一次
    if(++log_counter >= 10) {
        log_counter = 0;
    }
}
```

---

## 📚 八、参考资料索引

### 8.1 项目内文件路径

| 文件名 | 路径 | 作用 |
|--------|------|------|
| `motor_app.h/c` | `code/app/` | 用户应用层接口 |
| `small_driver_uart_control.h/c` | `code/driver/motor/` | UART 驱动层 |
| `lqr_balance.h/c` | `code/app/` | LQR 平衡控制算法 |
| `motor_pid.h/c` | `code/app/` | PID 控制器实现 |
| `zf_common_headfile.h` | `libraries/zf_common/` | 统一头文件入口 |

---

### 8.2 外部资源

- **CYT2BL3 官方文档**: `libraries/doc/`文件夹内的版本说明和技术手册
- **MENC15A 编码器 datasheet**: 查阅磁性编码器工作原理
- **TC264 数据手册**: Infineon 官网下载完整参考手册
- **逐飞科技论坛**: https://seekfree.taobao.com/ 获取技术支持

---

## 🎯 九、AI 助手快速上手检查清单

在阅读完本文档后，AI 助手应该能够独立完成以下任务：

### 基础能力 ✓
- [ ] 解释占空比、速度反馈、UART 协议的概念
- [ ] 说出 `motor_set(5000)` 和 `motor_get_raw_rpm()` 的含义和单位
- [ ] 画出系统架构图 (TC264 → CYT2BL3 → 电机 → 编码器)
- [ ] 列出至少 3 个常见的故障现象及排查方法

### 中级能力 ✓
- [ ] 为用户添加电池电压监测功能 (修改 4 个文件)
- [ ] 实现简单的速度闭环 PID 控制
- [ ] 编写编码器零位校准流程
- [ ] 解释为什么不能在 ISR 中调用 `delay_ms()`

### 高级能力 ✓
- [ ] 设计多模式控制切换框架
- [ ] 实现数据记录和回放系统
- [ ] 优化 PID 参数以适应不同负载条件
- [ ] 诊断并解决复杂的 EMC 干扰问题

---

## 📞 十、获取帮助

当遇到本文档未覆盖的问题时：

1. **首先检查**: 
   - 代码是否有编译错误？
   - UART 接线是否正确 (TX↔RX交叉)?
   - 驱动器是否正常供电？

2. **然后尝试**:
   - 使用官方例程测试硬件是否正常
   - 降低波特率到 115200 测试通信稳定性
   - 单独测试每个电机 (断开另一个电机)

3. **最后求助**:
   - 提供详细的错误现象描述
   - 附上相关代码片段
   - 如有可能，提供逻辑分析仪抓取的波形

---

**文档结束**  
*如有疑问或发现错误，请联系项目维护者更新本文档*
