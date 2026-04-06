# LIUHE_bicycle1

> 第二十一届全国智能车竞赛 · 单车定向组参赛工程  
> 平台：Infineon AURIX TC264D · 工具链：TASKING · 框架：逐飞科技开源库

---

## 目录

1. [项目概述](#1-项目概述)
2. [代码规模](#2-代码规模)
3. [软件架构](#3-软件架构)
4. [核心模块详解](#4-核心模块详解)
   - 4.1 IMU 姿态模块
   - 4.2 舵机平衡控制（PID / 串级 / 低速）
   - 4.3 LQR 平衡控制
   - 4.4 电机速度控制
   - 4.5 GPS / IMU 导航模块（nav_app）
   - 4.6 无线调试模块（wireless_debug_app）
   - 4.7 调度器
5. [启动流程](#5-启动流程)
6. [快速上手指南](#6-快速上手指南)
   - 6.1 [按键使用说明](#61-按键使用说明)
   - 6.2 [IPS 页面说明](#62-ips-页面说明)
7. [无线调参通道速查](#7-无线调参通道速查)
8. [导航模块使用手册](#8-导航模块使用手册)
9. [编译与烧录](#9-编译与烧录)
10. [目录结构](#10-目录结构)
11. [科目1 使用指南](#11-科目1-使用指南)
12. [科目2 使用指南](#12-科目2-使用指南)
13. [科目3 使用指南](#13-科目3-使用指南)
14. [Flash 持久化使用指南](#14-flash-持久化使用指南)
15. [BUG 修复记录](#15-bug-修复记录更新至-2026-04-06)
16. [已知现状与后续建议](#16-已知现状与后续建议)
17. [赛前验证顺序](#17-赛前验证顺序)

---

## 1. 项目概述

本工程实现一辆**摩托车结构自平衡单车**的全套控制系统，参加第二十一届全国智能车竞赛单车定向组。

### 车辆特性

| 属性 | 说明 |
|------|------|
| 平衡原理 | 纯舵机转向平衡（无动量轮）：倾斜 → 转向同侧 → 离心力回正 |
| 驱动 | 无刷电机 FOC，轮驱一体，减速比 1:1，轮径 64mm |
| 传感器 | IMU660RB（6轴，SPI），TAU1201 双频 GPS |
| 控制器 | AURIX TC264D，主频 200MHz，双核 TriCore |
| 舵机 | MG996R（调试）/ BDS300（比赛，330Hz） |

### 比赛题目对应

| 题目 | 内容 | 本工程对应方案 |
|------|------|---------------|
| 题目一 | 高速直线行驶 | GPS 路点导航 + CASCADE 串级 PID 平衡 |
| 题目二 | 低速绕八字 | IMU 航向段导航 + LOW_SPEED 纯角度环 |
| 题目三 | 地形障碍 | GPS 路点导航 + LQR 自适应增益平衡 |

---

## 2. 代码规模

### 用户代码统计（不含第三方库）

| 层次 | 目录 | 行数 |
|------|------|------|
| 应用层 | `code/app/` | **3551 行** |
| 驱动层 | `code/driver/` | **3637 行** |
| 用户主程序 | `user/` | **637 行** |
| **用户代码合计** | | **≈ 7825 行** |

### 应用层模块行数明细

| 模块文件 | 行数 | 说明 |
|---------|------|------|
| `imu_app.c` | 433 | IMU 应用层、校准、零位补偿 |
| `nav_app.c` | 413 | **[新增]** GPS/IMU 融合导航 |
| `ips_app.c` | 378 | 屏幕显示逻辑 |
| `lqr_balance.c` | 260 | LQR 平衡控制应用层 |
| `servo_pid.c` | 258 | **[扩展]** 舵机 PID（三种模式）|
| `wireless_debug_app.c` | 223 | **[新增]** WiFi 无线调试 |
| `scheduler.c` | 133 | 任务调度器 |
| `motor_pid.c` | 121 | 电机速度 PID |
| `key_app.c` | 104 | 按键业务逻辑 |
| `motor_app.c` | 96 | 电机应用层封装 |
| `servo_app.c` | 84 | 舵机驱动封装 |

### 本次新增 / 修改的代码

| 文件 | 状态 | 变更 |
|------|------|------|
| `code/app/nav_app.c` | **全新** | 413 行，GPS/IMU 导航完整实现 |
| `code/app/nav_app.h` | **全新** | 138 行，导航模块公开接口 |
| `code/app/wireless_debug_app.c` | **全新** | 223 行，WiFi 调试核心实现 |
| `code/app/wireless_debug_app.h` | **全新** | 79 行，WiFi 调试配置与接口 |
| `code/app/servo_pid.c` | **扩展** | +127 行，新增串级 / 低速双模式 |
| `code/app/servo_pid.h` | **扩展** | +16 行，新增模式宏和内环变量声明 |
| `code/app/motor_pid.c` | **微改** | +3 行，暴露 `g_motor_pid_output` |
| `code/app/motor_pid.h` | **微改** | +2 行，extern 声明 |
| `user/cpu0_main.c` | **微改** | +10 行，新增两个模块初始化调用 |
| `code/app/scheduler.c` | **微改** | +3 行，注册导航任务 |

---

## 3. 软件架构

```
┌─────────────────────────────────────────────────────────────┐
│                      用户主程序层 (user/)                    │
│  cpu0_main.c  ·  isr.c  ·  isr_config.h                    │
└───────────────────────┬─────────────────────────────────────┘
                        │ 调用
┌───────────────────────▼─────────────────────────────────────┐
│                      应用层 (code/app/)                      │
│                                                             │
│  ┌─────────────┐  ┌──────────────┐  ┌──────────────────┐   │
│  │  平衡控制    │  │   速度控制    │  │    导航模块       │   │
│  │ servo_pid   │  │  motor_pid   │  │    nav_app       │   │
│  │ lqr_balance │  │  motor_app   │  │    gps_app       │   │
│  └──────┬──────┘  └──────┬───────┘  └────────┬─────────┘   │
│         │                │                    │             │
│  ┌──────▼──────┐  ┌──────▼───────┐  ┌────────▼─────────┐   │
│  │  imu_app    │  │  scheduler   │  │ wireless_debug   │   │
│  └─────────────┘  └──────────────┘  └──────────────────┘   │
└───────────────────────┬─────────────────────────────────────┘
                        │ 调用
┌───────────────────────▼─────────────────────────────────────┐
│                  自定义驱动层 (code/driver/)                  │
│  mcu_dmp (姿态融合)  ·  lqr_driver  ·  pid_driver           │
│  ips_driver  ·  key_driver  ·  led_driver                   │
└───────────────────────┬─────────────────────────────────────┘
                        │ 调用
┌───────────────────────▼─────────────────────────────────────┐
│                  逐飞底层库 (libraries/)                      │
│  zf_driver  ·  zf_device (GNSS/WiFi_SPI)                   │
│  zf_components (seekfree_assistant)  ·  zf_common           │
│  infineon_libraries (iLLD/Infra/Service)                    │
└─────────────────────────────────────────────────────────────┘
```

### 中断时序

| 中断 | 周期 | 工作 |
|------|------|------|
| CCU60_CH1 PIT | **5ms** | IMU 采样 + 平衡控制输出（PID 或 LQR 二选一）|
| CCU61_CH0 PIT | **20ms** | 串口数据接收处理 |
| 主循环（while 1）| — | `imu_proc()`（最高优先）+ `scheduler_run()` |

### 调度器任务表

| 任务 | 周期 | 说明 |
|------|------|------|
| `key_scan()` | 10ms | 按键状态采集 |
| `key_task()` | 10ms | 按键业务处理 |
| `ips_app_task()` | 200ms | 屏幕局部刷新 |
| `gps_task()` | 10ms | GPS 数据解析 |
| `wireless_debug_task()` | **50ms** | WiFi 参数接收 + 波形发送 |
| `nav_app_task()` | **100ms** | GPS/IMU 导航，航向误差计算 |

---

## 4. 核心模块详解

### 4.1 IMU 姿态模块

**文件：** `code/app/imu_app.c/h`，`code/driver/imu/mcu_dmp.c`

**算法：** Madgwick 融合 + 静止 PI 修正 + 偏航角扩展卡尔曼

**关键输出变量：**

| 变量 | 含义 |
|------|------|
| `roll_kalman` | IMU 原始横滚角（含装配偏差，°）|
| `pitch_kalman` | IMU 原始俯仰角（°）|
| `yaw_kalman` | 相对偏航角（无磁力计，为相对值，°）|
| `gyro_x_rate` | X 轴角速度（已减零偏，deg/s）|
| `roll_ctrl_angle` | 控制用横滚角 = raw - 机械零位（°）|
| `pitch_ctrl_angle` | 控制用俯仰角（°）|

**IMU 安装与代码映射说明：**
- 算法统一使用的车体坐标系：X 轴 → 车辆前进方向，Y 轴 → 车辆左侧，Z 轴 → 垂直向上
- 当前实车安装方向：IMU 芯片 X 轴 → 车辆左侧，Y 轴 → 车辆后方，Z 轴 → 垂直向上
- 已在 `code/app/imu_app.c` 输入层完成坐标映射：`body_x = -sensor_y`，`body_y = sensor_x`，`body_z = sensor_z`

**校准方式：**

```c
// 快速调试（写死历史零偏值，立即可用）
imu_calibrate_gyro_temp();

// 精确校准（采样 30000 次，约 60s）
// imu_calibrate_gyro();
```

---

### 4.2 舵机平衡控制

**文件：** `code/app/servo_pid.c/h`

本模块支持三种控制模式，通过全局变量 `g_servo_control_mode` 在线切换：

#### 模式 0：SIMPLE_PD（默认）

```
输出 = Kp × angle_error + Ki × ∫angle_error + Kd × gyro_rate
```

- 原始位置式 PID，D 项直接用陀螺仪角速度
- **适用：** 台架联调、参数初步验证

#### 模式 1：CASCADE 串级双环（推荐比赛模式）

```
外环：target_rate = Kp × angle_error + Ki × ∫angle_error   （°→ deg/s）
内环：output = inner_Kp × (target_rate - gyro_rate) + inner_Ki × ∫...   （deg/s → duty）
```

- 外环输出是期望角速率，内环跟踪角速率误差
- 两环独立调参，抗扰动更强，适合中高速行驶
- **适用：** 题目一（高速直行）、题目三（地形障碍）

#### 模式 2：LOW_SPEED 低速纯角度环

```
输出 = Kp × angle_error + Ki × ∫angle_error   （无陀螺项）
```

- 去掉陀螺 D 项，消除低速时陀螺噪声引发的高频震荡
- 响应略慢，但低速（<1m/s）下稳定性更好
- **适用：** 题目二（低速绕八字）

**关键参数变量：**

| 变量 | 初始值 | 说明 |
|------|-------|------|
| `g_servo_control_mode` | 0 | 控制模式（0/1/2）|
| `g_balance_kp` | 20.0 | 外环角度 Kp |
| `g_balance_ki` | 0.25 | 外环角度 Ki |
| `g_balance_kd` | 0.05 | 陀螺系数（仅模式 0）|
| `g_balance_inner_kp` | 1.0 | 内环角速率 Kp（仅模式 1）|
| `g_balance_inner_ki` | 0.0 | 内环角速率 Ki（仅模式 1）|
| `g_balance_output_limit` | 250.0 | 最终输出限幅 |
| `g_servo_steer_dir` | 1 | 舵机总方向，`1` 正常，`-1` 反向 |
| `expect_angle` | 0.0 | 期望侧倾角（转弯 / 导航输入）|

**CASCADE 调参顺序：**

1. 令 `ki=0, inner_kp=1.0`，先调外环 `kp`（让车能基本直立）
2. 固定外环，逐步增大 `inner_kp`（提升动态响应）
3. 最后再加 `ki` 消除静差

**使能控制（必须在零位捕获后再开启）：**

```c
balance_control_set_enable(0U);  // 关闭，PID 复位，舵机归中
balance_control_set_enable(1U);  // 开启
```

**转向方向反了时的处理：**

```c
// code/app/servo_pid.c
volatile int8_t g_servo_steer_dir = SERVO_STEER_DIR_NORMAL;    // 当前方向
// volatile int8_t g_servo_steer_dir = SERVO_STEER_DIR_REVERSED;  // 整体反向
```

- 此变量只作用在最终舵机输出层，不修改 IMU、PID 误差和导航正负号定义
- 若实车出现“右倾时前轮实际左打”或“左倾时前轮实际右打”，优先切换这里

---

### 4.3 LQR 平衡控制

**文件：** `code/app/lqr_balance.c/h`，`code/driver/lqr/lqr_driver.c/h`

**状态方程：**

```
状态：x = [φ, φ_dot, δ]    （横滚角、横滚角速率、舵机角）
输入：u = δ_dot             （舵机角速度）
```

**速度自适应增益表：** 15 个速度点（0.3~7.0 m/s），每点一组 K 值，低速/高速分段插值。

**切换控制：** 修改 `code/app/balance_control_mode.h`：

```c
#define BALANCE_CONTROL_MODE  BALANCE_CONTROL_MODE_PID   // 使用 PID
#define BALANCE_CONTROL_MODE  BALANCE_CONTROL_MODE_LQR   // 使用 LQR
```

**LQR 调参工具：** `code/tools/lqr_gain_generator.py`，修改物理参数后重新运行，将输出粘贴到 `lqr_driver.c` 的 `default_gain_entries[]`。

---

### 4.4 电机速度控制

**文件：** `code/app/motor_pid.c/h`，`code/app/motor_app.c/h`

**控制方式：** 增量式 PID，20ms 周期（CCU61_CH0 PIT 中断）

**关键变量：**

| 变量 | 说明 |
|------|------|
| `target_motor_rpm` | 目标转速（RPM）|
| `motor_speed_rpm` | 实际转速（RPM，前进为正）|
| `motor_speed_m_s` | 实际线速度（m/s，前进为正）|
| `g_motor_pid_output` | 当前 PID 输出值（可由无线调试读取）|

**速度辅助平衡（Speed Assist）：**

```c
// 公式：ideal_speed = base_speed + abs(roll_angle) * assist_gain
// 倾斜时加速以增大离心力，辅助平衡
speed_assist_enable(1U);
speed_assist_init(600.0f, 20.0f, 200.0f);  // base_rpm, gain, max_boost
```

---

### 4.5 GPS / IMU 导航模块（nav_app）

**文件：** `code/app/nav_app.c/h`（新增，413+138 行）

这是本工程的核心新增模块，实现两种导航模式，不直接修改任何控制变量，输出航向误差供控制层读取。

#### 主输出变量

| 变量 | 说明 |
|------|------|
| `g_nav_heading_error` | **主输出**：航向误差（°，正=需右转，负=需左转）|
| `g_nav_target_speed` | 当前段目标速度（m/s）|
| `g_nav_imu_heading` | IMU 积分偏航（°，相对于起点）|
| `g_nav_distance_m` | 当前段已行里程（m）|
| `g_nav_state` | 导航状态（IDLE/READY/NAVIGATING/DONE）|
| `g_nav_heading_gain` | 航向增益（可无线调参）|

#### 控制层接入（二选一）

```c
// 已接入：user/isr.c 的 5ms 控制节拍中自动执行
expect_angle = g_nav_heading_error * g_nav_heading_gain;

// 已接入：同一处同时同步到 LQR
lqr_expect_phi = g_nav_heading_error * g_nav_heading_gain;
```

#### 模式 A：GPS 路点导航（题目一、三）

依赖逐飞库已有函数 `get_two_points_distance()` 和 `get_two_points_azimuth()`，无需自己实现 Haversine。

**GPS 漂移修正原理：**

同一物理位置每次开机 GPS 读数存在米级偏移。解决方案：勘线时在起点记录参考坐标，比赛时再次测量起点坐标，差值即漂移量，对所有后续 GPS 读数做统一修正。

#### 模式 B：IMU 航向段（题目二）

按预设偏航角序列逐段跟踪，用陀螺仪积分偏航角计算误差，用电机线速度积分里程来切换段。

---

### 4.6 无线调试模块（wireless_debug_app）

**文件：** `code/app/wireless_debug_app.c/h`（新增，223+79 行）

**硬件：** 高速 WiFi SPI 模块（SPI3，30MHz）+ 逐飞助手 PC 软件

**功能：**
- **参数调试**（PC → MCU，8 通道）：实时下发 PID 参数
- **波形显示**（MCU → PC，8 通道）：实时查看状态曲线
- 两个方向同时工作，互不干扰

**配置（修改后烧录）：**

```c
// code/app/wireless_debug_app.h
#define WIRELESS_WIFI_SSID       "你的WiFi名称"
#define WIRELESS_WIFI_PASSWORD   "你的WiFi密码"
#define WIRELESS_TCP_SERVER_IP   "192.168.x.x"   // PC 的 IP 地址
#define WIRELESS_TCP_SERVER_PORT "8086"           // 默认端口
```

**PC 端操作：** 打开逐飞助手 → 网络配置 → TCP Server → 端口 8086 → 启动监听

**失败容错：** WiFi 连接失败时仅打印提示，不阻塞系统启动。

---

### 4.7 调度器

**文件：** `code/app/scheduler.c/h`

简单轮询调度，在主循环中以 `scheduler_run()` 驱动。首次运行时跳过（避免立即触发），后续按设定周期调用。IMU 中断和主循环 `imu_proc()` 具有更高优先级，不受调度器影响。

---

## 5. 启动流程

`user/cpu0_main.c` 中 `core0_main()` 执行顺序：

```
1.  clock_init()                      时钟初始化
2.  debug_init()                      调试串口
3.  led_init() / key_app_init()       LED、按键
4.  ips_app_init()                    屏幕
5.  gps_init()                        GPS 驱动初始化
6.  pwm_init() / servo_init()         舵机 PWM
7.  balance_pid_init()                PID 参数初始化
8.  lqr_balance_init()                LQR 初始化
9.  balance_control_set_enable(0)     ← 关闭控制输出（保护 IMU 校准阶段）
10. lqr_balance_set_enable(0)
11. imu_all_init()                    IMU 硬件 + 四元数初始化
12. imu_calibrate_gyro_temp()         加载陀螺零偏临时值
13. pit_ms_init(CCU60_CH1, 5ms)       启动 5ms IMU 采样中断
14. zero_compensation()               等待 100 帧后捕获机械零位
15. balance_control_set_enable(1)  ← 根据 balance_control_mode.h 二选一开启
    或 lqr_balance_set_enable(1)
16. motor_init()                      电机串口驱动
17. motor_pid_init()                  电机 PID
18. uart_receiver_init()              串口接收
19. pit_ms_init(CCU61_CH0, 20ms)      启动 20ms 电机控制中断
20. wireless_debug_init()             ← 【新增】WiFi SPI + 逐飞助手
21. nav_app_init()                    ← 【新增】导航模块状态清零
22. scheduler_init()                  调度器初始化
23. cpu_wait_event_ready()
24. led_all_set(LED_ON)               ← 初始化完成标志（LED 点亮）
25. while(1): imu_proc() + scheduler_run()
```

> **关键设计**：第 9-10 步先关闭所有控制输出，等 IMU 零位捕获完成后再开启，防止舵机动作干扰姿态基线。

---

## 6. 快速上手指南

### Step 1：确认硬件接线

WiFi SPI 模块接线（SPI3）：

| 信号 | MCU 引脚 |
|------|---------|
| SCLK | P22_3 |
| MOSI | P22_0 |
| MISO | P22_1 |
| CS   | P22_2 |
| INT  | P15_8 |
| RST  | P23_1 |

> 若有引脚冲突，在 `libraries/zf_device/zf_device_wifi_spi.h` 中修改（注意：库文件不修改，通过宏覆盖）。

### Step 2：填写 WiFi 配置

```c
// code/app/wireless_debug_app.h
#define WIRELESS_WIFI_SSID       "MyWiFi"
#define WIRELESS_WIFI_PASSWORD   "password123"
#define WIRELESS_TCP_SERVER_IP   "192.168.1.100"
```

### Step 3：选择平衡控制方案

```c
// code/app/balance_control_mode.h
#define BALANCE_CONTROL_MODE  BALANCE_CONTROL_MODE_PID   // 新车先用 PID 验证
// #define BALANCE_CONTROL_MODE  BALANCE_CONTROL_MODE_LQR
```

### Step 4：选择舵机型号

```c
// code/app/servo_app.h
#define CURRENT_SERVO_TYPE  SERVO_TYPE_MG996R   // 调试用
// #define CURRENT_SERVO_TYPE  SERVO_TYPE_BDS300   // 比赛用
```

### Step 5：编译烧录，观察串口输出

正常初始化完成后应看到：
```
[WIRELESS] WiFi debug init OK. Module IP: ...
```

LED 全部点亮 = 初始化完成。

### Step 6：在逐飞助手调参

- 打开逐飞助手 → TCP Server → 端口 8086 → 连接
- 在"参数调试"窗口逐项调整 CH1~CH8
- 在"波形显示"窗口观察实时曲线
- 通过 CH8 在线切换控制模式（无需重烧）

### Step 7：导航功能（比赛题目一、三）

```c
// --- 勘线阶段（在赛场跑一遍，记录关键坐标）---
// 1. 车停在起点，等 GPS 定位稳定
nav_set_ref_start();              // 记录参考起点
// 2. 行驶到每个路点处，调用：
nav_record_waypoint(3.0f, 2.0f); // (目标速度 m/s, 到达半径 m)
// 3. 最多记录 20 个路点

// --- 比赛阶段 ---
// 1. 车放回物理起点
nav_start_gps();                  // 自动算漂移修正，启动导航
// 2. 导航航向误差已在 user/isr.c 的 5ms 控制节拍中自动接入 PID/LQR
```

---

### 6.1 按键使用说明

当前工程实际生效的按键逻辑以 `[key_app.c](E:\AURIX-v1.10.28-workspace\LIUHE_bicycle1\code\app\key_app.c)` 为准：

| 按键 | 操作 | 功能 |
|------|------|------|
| `KEY1` | 短按 | 当前 IPS 选中参数减小一次 |
| `KEY1` | 长按 | 当前 IPS 选中参数连续减小 |
| `KEY1` | 双击 | 保存全部配置到 Flash |
| `KEY2` | 短按 | 当前 IPS 选中参数增大一次 |
| `KEY2` | 长按 | 当前 IPS 选中参数连续增大 |
| `KEY2` | 双击 | 活跃科目循环切换：`1 → 2 → 3 → 1` |
| `KEY3` | 短按 | IPS 切换到下一页 |
| `KEY3` | 双击 | IPS 切换到上一页 |
| `KEY3` | 长按 | 按当前活跃科目启动/停止：科目1/2/3共用 |
| `KEY4` | 短按 | IPS 选中下一项参数 |
| `KEY4` | 双击 | IPS 选中上一项参数 |
| `KEY4` | 长按 | 按当前活跃科目勘线：科目1两点勘线，科目3多点勘线，科目2无勘线 |

补充说明：
- `KEY1/KEY2` 的长按连续调节是由 `key_task()` 每 10ms 周期触发的，所以长按时参数会持续变化。
- `KEY3/KEY4` 的长按功能优先级高于普通页面操作，实际比赛时主要用来启动/停止和勘线。
- Flash 持久化当前以 `KEY1 双击` 为准，README 其余章节若涉及保存动作，也应以此为准。

### 6.2 IPS 页面说明

当前 IPS 默认开机页已经改为 `HOME`，页面顺序如下：

`HOME -> Motor PID -> Servo PID -> GPS -> IMU -> TEST -> SUBJ1 -> SUBJ2 -> SUBJ3`

各新增页面用途：

- `HOME`：总览当前活跃科目、GPS 状态、导航状态、控制模式、电机转速、5ms 控制最大耗时、yaw，以及主要模块初始化标志。
- `SUBJ1`：显示科目1状态、距目标路点距离、航向误差，并可直接调 `HIGH/MID/TURN/BrakeDst/FinDst/Resume/Accel`。
- `SUBJ2`：显示科目2状态、IMU 航向、里程、航向误差，并可直接调 `RPM` 和 `Radius`。
- `SUBJ3`：显示科目3状态、距目标路点距离、航向误差，并可直接调 `HIGH/MID/TURN/BrakeDst/FinDst/Resume/Accel`。

`HOME` 页里的 `IMUinit/GPSinit/MotInit/WiFiInit/NavInit/CfgLd` 含义说明：

- 这些标志表示对应的软件初始化流程已经走通。
- `CfgLd=1` 表示本次上电成功从全局配置 Flash 恢复参数。
- 它们不等同于“GPS 已锁星”或“WiFi 已连上上位机”，实时链路状态仍应结合 `GPS` 页、串口打印和实际运行现象判断。

---

## 7. 无线调参通道速查

### 参数调试（PC → MCU，逐飞助手"参数调试"面板）

| CH | 变量 | 说明 | 推荐调参范围 |
|----|------|------|------------|
| **1** | `g_balance_kp` | 外环角度 Kp（三种模式通用）| 10 ~ 40 |
| **2** | `g_balance_ki` | 外环角度 Ki | 0 ~ 1.0 |
| **3** | `g_balance_kd` | 陀螺 D 系数（仅 SIMPLE_PD）| 0 ~ 0.2 |
| **4** | `g_balance_inner_kp` | CASCADE 内环角速率 Kp | 0.5 ~ 5 |
| **5** | `g_balance_inner_ki` | CASCADE 内环角速率 Ki | 0 ~ 0.5 |
| **6** | `g_nav_heading_gain` | 导航航向增益 | 0.1 ~ 0.8 |
| **7** | `target_motor_rpm` | 目标转速（RPM）| 300 ~ 1500 |
| **8** | `g_servo_control_mode` | 控制模式：0/1/2 | 整数 0、1、2 |

> 发送整数 0/1/2 给 CH8 即可在线切换控制模式，`wireless_debug_app.c` 会立即清零外环积分，避免切换瞬间舵机猛打。

### 波形显示（MCU → PC，逐飞助手"波形显示"面板）

| CH | 变量 | 单位 | 说明 |
|----|------|------|------|
| **1** | `roll_ctrl_angle` | ° | 补偿后横滚角（核心平衡观测量）|
| **2** | `g_balance_pid_output` | duty | 舵机最终输出（观察饱和情况）|
| **3** | `motor_speed_rpm` | RPM | 电机实际转速 |
| **4** | `motor_speed_m_s` | m/s | 车体线速度（导航调参更直观）|
| **5** | `g_nav_heading_error` | ° | 导航航向误差（导航调试核心）|
| **6** | `yaw_kalman` | ° | IMU 偏航角（观察导航漂移）|
| **7** | `g_motor_pid_output` | — | 电机 PID 输出 |
| **8** | `target_motor_rpm` | RPM | 目标转速 |

---

## 8. 导航模块使用手册

### 8.1 GPS 路点模式（题目一、三）

#### 勘线流程（只做一次）

```c
// 1. 将车放置在物理起点，等待 GPS 定位（gnss.state == 1）
nav_set_ref_start();               // 记录参考起点坐标

// 2. 沿比赛路线行驶，在关键转折点或终点前 2m 处调用：
nav_record_waypoint(3.0f, 2.0f);  // 速度 3m/s，到达半径 2m
nav_record_waypoint(2.0f, 1.5f);  // 速度 2m/s，到达半径 1.5m
// ... 最多 20 个路点

// 3. 第2次勘线完成后会自动保存到 Flash
//    后续上电可直接恢复参考起点和路点
```

#### 比赛流程

```c
// 1. 车放在起点
nav_start_gps();                   // 自动修正 GPS 漂移，启动导航

// 2. 导航航向误差已在 user/isr.c 的 5ms 控制节拍中自动接入 PID/LQR
```

#### GPS 漂移修正说明

```
漂移量 = 比赛时起点 GPS 坐标 - 勘线时参考起点 GPS 坐标
修正后当前位置 = 实时 GPS 坐标 - 漂移量
```

**前提：** 每次将车放在同一物理位置；场地面积 <200m 可视漂移为空间均匀量。

### 8.2 IMU 航向段模式（题目二）

```c
// 规划八字路线（示例：两个圆圈）
nav_add_imu_segment(  0.0f, 8.0f, 1.0f);   // 直行 8m，航向 0°，速度 1m/s
nav_add_imu_segment( 90.0f, 5.0f, 0.8f);   // 左弧，目标航向 90°
nav_add_imu_segment(180.0f, 8.0f, 1.0f);   // 反向直行
nav_add_imu_segment(-90.0f, 5.0f, 0.8f);   // 右弧
// 最多 10 段

// 比赛时：
nav_start_imu();   // 重置偏航参考，启动 IMU 段导航
```

**注意：** IMU yaw 无磁力计，长期漂移约 1~5°/分钟，此模式适合 1~2 分钟内短程应用。

这是一个坑点

### 8.3 停止导航

```c
nav_stop();   // 回到 IDLE，g_nav_heading_error 清零
```

### 8.4 状态枚举

```c
typedef enum {
    NAV_STATE_IDLE       = 0,   // 空闲，无输出
    NAV_STATE_READY      = 1,   // 路点已加载，等待 nav_start_xxx()
    NAV_STATE_NAVIGATING = 2,   // 导航中，持续更新 g_nav_heading_error
    NAV_STATE_DONE       = 3,   // 末路点到达
} nav_state_t;
```

---

## 9. 编译与烧录

### 工具链

- **IDE：** AURIX Development Studio（ADS）/ Eclipse CDT
- **编译器：** TASKING TriCore 工具链
- **目标：** TC264D，Package BGA292
- **调试入口：** `LIUHE_bicycle1 Debug.launch`，停止于 `core0_main`

### 头文件规范

所有用户代码统一使用：

```c
#include "zf_common_headfile.h"   // 包含全部逐飞库和用户自定义头文件
```

### 代码组织规范

- 用户代码全部在 `code/` 目录
- **不修改** `libraries/` 目录（逐飞底层库）
- 新增模块放在 `code/app/` 或 `code/driver/`

---

## 10. 目录结构

```text
LIUHE_bicycle1/
├── code/
│   ├── app/
│   │   ├── imu_app.c / .h              IMU 应用层（校准、零位补偿）
│   │   ├── servo_pid.c / .h            舵机平衡控制（SIMPLE/CASCADE/LOW_SPEED）
│   │   ├── lqr_balance.c / .h          LQR 平衡控制应用层
│   │   ├── balance_control_mode.h      PID/LQR 一键切换开关
│   │   ├── servo_app.c / .h            舵机硬件封装（型号 / PWM 参数）
│   │   ├── motor_app.c / .h            电机应用层（速度换算、方向约定）
│   │   ├── motor_pid.c / .h            电机速度 PID（增量式）
│   │   ├── gps_app.c / .h              GPS 驱动封装（TAU1201 初始化与解析）
│   │   ├── nav_app.c / .h          ★  GPS/IMU 融合导航（新增）
│   │   ├── wireless_debug_app.c / .h ★  WiFi SPI 无线调参 + 波形显示（新增）
│   │   ├── ips_app.c / .h              屏幕显示逻辑
│   │   ├── ips_app_config.h            屏幕页面与参数配置
│   │   ├── key_app.c / .h              按键业务逻辑
│   │   ├── scheduler.c / .h            主循环轮询调度器
│   │   └── led_app.c                   LED 测试
│   ├── driver/
│   │   ├── imu/
│   │   │   ├── mcu_dmp.c / .h          姿态融合算法（Madgwick + PI + EKF）
│   │   │   └── ekf.c / .h              扩展卡尔曼（偏航）
│   │   ├── lqr/
│   │   │   └── lqr_driver.c / .h       LQR 控制器 + 速度自适应增益表
│   │   ├── pid/
│   │   │   └── pid_driver.c / .h       通用 PID（位置式 + 增量式）
│   │   ├── motor/
│   │   │   └── small_driver_uart_control.c / .h  无刷电机 FOC 串口驱动
│   │   ├── ips/                        IPS 屏幕基础绘图驱动
│   │   ├── key/                        按键驱动（消抖、长按、双击）
│   │   └── led/                        LED 驱动
│   └── tools/
│       ├── lqr_gain_generator.py       LQR 增益表离线生成脚本
│       └── README.md                   LQR 移植说明
├── libraries/                          逐飞开源库（不可修改）
│   ├── infineon_libraries/             iLLD / Infra / Service
│   ├── zf_common/
│   ├── zf_driver/
│   ├── zf_device/                      含 GNSS、WiFi_SPI 驱动
│   └── zf_components/                  含 seekfree_assistant
├── user/
│   ├── cpu0_main.c                     CPU0 主入口 + 初始化流程
│   ├── isr.c                           所有中断服务函数
│   ├── isr_config.h                    中断优先级配置
│   └── cpu1_main.c                     CPU1（预留，无实际逻辑）
├── 推荐IO分配.txt
├── LQR_GUIDE.md                        LQR 详细调参指南
└── README.md                           本文件
```

★ = 本次新增模块

---

## 11. 科目1 使用指南

> 本章节中若前后描述不一致，以本节后面的“当前代码覆盖说明（2026-04-06）”为准；该说明已按当前 `subject1_app.c/.h` 实现同步更新。

### 导航策略

| 阶段 | 方案 | 原因 |
|------|------|------|
| 去程（直行至掉头区） | GPS路点导航 | 速度>2km/h时航向精度~2°，20m偏差<1m |
| 回程（掉头+返回） | **GPS路点导航**（升级后，全程GPS） | 比IMU回程少~1.5m侧偏，路径更直 |
| 掉头段 | GPS+IMU融合（H2修复） | 低速时自动用IMU补偿，平滑无跳变 |

### 速度分段策略（升级）

| 状态 | 速度 | 切换条件 |
|------|------|---------|
| GO（直行）| HIGH_RPM（600）| 距掉头点 < PRE_BRAKE_DIST |
| GO_BRAKE（减速进弯）| 线性 MID(400)→TURN(280) | GPS到达路点 |
| UTURN（掉头）| TURN_RPM（280）恒定 | \|heading_error\| < RESUME_THRESH |
| RETURN（回程加速）| 每100ms +ACCEL_STEP，最高HIGH | GPS到达起点 |
| 终点制动区 | MID_RPM（400）| dist_to_wp < FINISH_BRAKE_DIST |

### 参数配置（`code/app/subject1_app.h`）

```c
#define SUBJ1_HIGH_RPM            600.0f  // 全速直行（RPM）
#define SUBJ1_MID_RPM             400.0f  // 中速过渡（RPM）
#define SUBJ1_TURN_RPM            280.0f  // 掉头低速（RPM）
#define SUBJ1_PRE_BRAKE_DIST_M    6.0f    // 距掉头点开始减速（m），初始6m
#define SUBJ1_FINISH_BRAKE_DIST_M 4.0f    // 距起点终点制动区（m）
#define SUBJ1_RESUME_THRESH_DEG   25.0f   // 出弯开始加速的航向误差阈值（°）
#define SUBJ1_ACCEL_STEP_RPM      50.0f   // 出弯每100ms加速步进（RPM）
#define SUBJ1_WP_RADIUS           2.5f    // 路点到达判定半径（m）
```

**调参建议：**
- 掉头半径过大 → 降低 `SUBJ1_TURN_RPM` 或增大 `SUBJ1_PRE_BRAKE_DIST_M`
- 出弯后翻车 → 降低 `SUBJ1_ACCEL_STEP_RPM` 或增大 `SUBJ1_RESUME_THRESH_DEG`
- 终点过冲 → 增大 `SUBJ1_FINISH_BRAKE_DIST_M`

### Flash 持久化（新增）

勘线后参数自动保存到 Flash，**下次上电无需重新勘线**：
- **勘线完成（KEY4第2次）**：自动保存 GPS 路点 + 全部 PID 参数
- **WiFi / IPS 调参后**：`KEY1 双击` → 保存所有参数

### 操作流程

**首次勘线（GPS锁星后进行，保存后后续上电无需重复）：**

1. 车放在发车区，等待GPS锁星（IPS GPS页 state=1）
2. **KEY4 长按（第1次）** → 串口打印 `Ref start recorded`
3. 推车至掉头区（15~30m处），保持静止
4. **KEY4 长按（第2次）** → 串口打印 `Config saved to flash`（路点自动保存）
5. 将车推回发车区起始线

**后续上电（已勘线，Flash有数据）：**

1. 上电初始化完成，Flash自动恢复路点和PID参数
2. 串口打印 `[FLASH] Loaded OK`
3. 科目1勘线状态会自动恢复为“已完成”，可直接 `KEY3` 长按进入比赛流程

**比赛：**

1. **KEY3 长按** → 电机高速启动，GPS导航直行
2. 到达掉头路点后自动切换GPS回程：先低速掉头，再高速返回
3. 回程结束：电机停止，舵机归中，打印 `Done!`

**紧急停止：** 运行中 **KEY3 长按**

### 串口输出示例

```
[NAV] Ref start recorded: lat=xx.xxxxxxx lon=xxx.xxxxxxx
[SUBJ1] Survey step 2/2: done. Config saved to flash.
[SUBJ1] Start! GO phase (GPS), target=600 RPM
[NAV] GPS navigation done
[SUBJ1] Waypoint reached! GPS RETURN phase started
[NAV] GPS navigation done
[SUBJ1] Done! Return complete.
```

### 当前代码覆盖说明（2026-04-06）

#### 方案评价

你现在这版科目1，已经明显比最早那版“去程高速，到点后低速掉头，角度误差小了再高速回程”的逻辑更成熟了。

当前实现的核心升级有四点：

- 去程增加了 `GO_BRAKE` 预减速段，不再高速直接冲进掉头区
- 掉头单独拆成 `UTURN` 状态，固定低速更稳
- 回程不是直接满速，而是先中速出弯，再渐进加速
- 回程终点前还有制动区，避免高速冲过起点

如果你的目标是“前期先把科目1稳定跑顺、把参数调通”，这版是合理的，也值得先用它作为主调试版本。

但要明确，这一版还不是我们前面讨论的“扁水滴型最快方案”或者“多点几何轨迹优化版”。它本质上仍然是：

- 一个参考起点 `ref_start`
- 一个掉头路点 `waypoint`
- 回程重新导航回 `ref_start`

所以它是“实用、稳、适合先跑起来”的版本，不是最终极限竞速版本。

#### 当前真实状态机

以 [`subject1_app.c`](E:\AURIX-v1.10.28-workspace\LIUHE_bicycle1\code\app\subject1_app.c) 为准，当前状态机为：

| 状态 | 含义 | 当前行为 |
|------|------|---------|
| `SUBJ1_STATE_IDLE` | 空闲 | 等待勘线或启动 |
| `SUBJ1_STATE_GO` | 去程高速 | 固定 `SUBJ1_HIGH_RPM` |
| `SUBJ1_STATE_GO_BRAKE` | 掉头前预减速 | 按距离线性从 `MID -> TURN` |
| `SUBJ1_STATE_UTURN` | 低速掉头 | 固定 `SUBJ1_TURN_RPM` |
| `SUBJ1_STATE_RETURN` | 回程 | 先中速，再按步进加速，终点前再降速 |
| `SUBJ1_STATE_DONE` | 完成 | 电机停转，导航停止 |

状态切换逻辑：

1. `KEY3` 长按启动后，进入 `GO`
2. 当 `g_nav_dist_to_wp < SUBJ1_PRE_BRAKE_DIST_M` 时进入 `GO_BRAKE`
3. 去程 GPS 判定到达掉头点后，切换回程目标并进入 `UTURN`
4. 当 `fabsf(g_nav_heading_error) < SUBJ1_RESUME_THRESH_DEG` 时进入 `RETURN`
5. 回程 GPS 判定到达起点后，进入 `DONE`
6. 任意运行状态下 `KEY3` 长按，都执行紧急停止

#### 这版方案的优点

- 比老版本稳得多，尤其是进弯和出弯两个最容易翻车的阶段。
- `GO_BRAKE` 让掉头区前的速度变化更可控。
- `UTURN` 单独保留低速状态，符合单车掉头时对稳定性的需求。
- `RETURN` 渐进加速，比“角度一小就直接满速”更适合调试。
- 回程使用 `nav_start_gps_keep_drift()`，保留去程的漂移修正，这一点非常关键。

#### 这版方案的局限

- 还不是最快几何路径，只是一个两点式的往返导航。
- 回程目标仍然是回到 `ref_start`，不是你之前构想的“额外停跑点”或“平行回程线”。
- 掉头轨迹仍然主要依赖 GPS 指向自然形成弧线，不是显式规划一条理想曲线。

所以我的判断是：

- 作为前期调试版本，这版是对的。
- 作为后期冲最快成绩版本，还可以继续升级。

#### 当前参数与调法

当前参数在 [`subject1_app.h`](E:\AURIX-v1.10.28-workspace\LIUHE_bicycle1\code\app\subject1_app.h) 中：

```c
#define SUBJ1_HIGH_RPM            600.0f
#define SUBJ1_MID_RPM             400.0f
#define SUBJ1_TURN_RPM            280.0f
#define SUBJ1_PRE_BRAKE_DIST_M    6.0f
#define SUBJ1_FINISH_BRAKE_DIST_M 4.0f
#define SUBJ1_RESUME_THRESH_DEG   25.0f
#define SUBJ1_ACCEL_STEP_RPM      50.0f
#define SUBJ1_WP_RADIUS           2.5f
```

各参数的实际意义：

| 参数 | 含义 | 调整建议 |
|------|------|---------|
| `SUBJ1_HIGH_RPM` | 去程和回程可达到的最高转速 | 先保守，稳定后再逐步增加 |
| `SUBJ1_MID_RPM` | 进弯过渡、出弯初段、终点制动时的中速 | 太高会让进弯和收尾发飘 |
| `SUBJ1_TURN_RPM` | 掉头时固定低速 | 掉头半径大、容易甩就降它 |
| `SUBJ1_PRE_BRAKE_DIST_M` | 离掉头点多远开始预减速 | 太小会冲弯，太大则浪费时间 |
| `SUBJ1_FINISH_BRAKE_DIST_M` | 离起点多远开始终点制动 | 太小容易冲过起点 |
| `SUBJ1_RESUME_THRESH_DEG` | 航向误差小于多少后允许回程加速 | 太大会提前加速，太小会磨蹭 |
| `SUBJ1_ACCEL_STEP_RPM` | 回程每 100ms 提高多少转速 | 太大最容易导致出弯后失稳 |
| `SUBJ1_WP_RADIUS` | 路点到达判定半径 | 太大提前判定，太小不易到点 |

推荐调参顺序：

1. 先调 `SUBJ1_PRE_BRAKE_DIST_M` 和 `SUBJ1_TURN_RPM`，保证能稳定进弯和掉头。
2. 再调 `SUBJ1_RESUME_THRESH_DEG` 和 `SUBJ1_ACCEL_STEP_RPM`，保证出弯不翻。
3. 稳定后再逐步提高 `SUBJ1_HIGH_RPM`。
4. 最后再修 `SUBJ1_FINISH_BRAKE_DIST_M`，让回程停车干净。

#### 当前版本怎么用

首次勘线前提：GPS 必须有效，否则当前代码不会推进勘线状态。

勘线步骤：

1. 把车放在发车区，等待 GPS 稳定。
2. `KEY4` 长按第一次，记录 `ref_start`。
3. 把车推到掉头区，静止后再 `KEY4` 长按第二次。
4. 第二次成功后，记录掉头路点，并自动保存到 Flash。

当前代码实际串口输出：

```text
[SUBJ1] Survey 1/2: ref_start recorded. Move to turn-around point.
[SUBJ1] Survey 2/2: waypoint recorded. Config saved to flash. Ready!
```

启动步骤：

1. 把车重新放回发车区。
2. `KEY3` 长按启动。
3. 系统自动按 `GO -> GO_BRAKE -> UTURN -> RETURN -> DONE` 运行。

阶段切换时的典型输出如下：

```text
[SUBJ1] START! GO @ 600 RPM
[SUBJ1] → GO_BRAKE @ dist=5.8m
[SUBJ1] → UTURN @ 280 RPM
[SUBJ1] → RETURN, heading_err=18.6°
[SUBJ1] DONE! Return complete.
```

紧急停止：

- 运行中 `KEY3` 长按
- 会执行 `nav_stop()`
- 电机目标转速归零
- 平衡参考清零
- 状态回到 `IDLE`

#### Flash 持久化行为

当前这版科目1已经和 Flash 联动：

- `KEY4` 第 2 次勘线成功后，会自动保存 GPS 路点和相关配置
- 下次上电后，如果 Flash 中已恢复出 `ref_start + waypoint`，`subject1_app_init()` 会自动判断勘线已完成
- 因此不是每次上电都必须重新勘线

#### 关键注意事项

- 这版是当前调试主版本，不是最终最快版本。
- 回程这里必须用 `nav_start_gps_keep_drift()`，不能随便改成 `nav_start_gps()`。
- 勘线两次打点都应该在 GPS 已稳定时完成，否则后面回程精度会受影响。
- `SUBJ1_PRE_BRAKE_DIST_M` 是最重要的稳定性参数之一，太小最容易冲过掉头点。
- `SUBJ1_ACCEL_STEP_RPM` 不要贪大，掉头后恢复加速是最危险的阶段。
- `SUBJ1_RESUME_THRESH_DEG` 也不是越小越好，太小会导致掉头后长时间磨蹭，整体反而更慢。
- 后续如果要继续追求更快成绩，下一步重点应转向“多点几何路径优化”，而不是单纯继续堆转速。

---

## 12. 科目2 使用指南

### 赛题概要

科目2：在半径 2m 的圆内完成锥桶**八字绕桩**，两锥桶间距 2m，单向绕行一周。考察低速平衡能力，越慢越稳越好。

### 导航方案

GPS 精度 ~1m，占圆半径 50%，**无法使用 GPS**。采用 **纯 IMU 惯导**：yaw_kalman 航向 + 编码器里程。

控制模式自动切换到 **LOW_SPEED**（纯 P+I，去掉角速度 D 项），适合低速大舵角平衡。

### 八字轨迹（8 段 IMU 航向序列，自动计算）

```
段0: heading=0°     → 直行到交叉点（发车区→中心）
段1: heading=-120°  → 绕 A 锥桶左转 120°
段2: heading=-240°  → 继续左转 120°
段3: heading=-360°  → 完成 A 整圈
段4: heading=-240°  → 绕 B 锥桶右转 120°
段5: heading=-120°  → 继续右转 120°
段6: heading=0°     → 完成 B 整圈
段7: heading=0°     → 直行回发车区
```

### 参数配置（`code/app/subject2_app.h`）

```c
#define SUBJ2_RPM               200.0f    // 低速转速，越低越稳（RPM）
#define SUBJ2_TURN_RADIUS_M     0.8f      // 绕桩半径，≤1.0 才能在圆内（m）
#define SUBJ2_APPROACH_DIST_M   0.5f      // 发车区到交叉点距离（m）
#define SUBJ2_RETURN_DIST_M     0.5f      // 交叉点回发车区距离（m）
```

弧长由半径自动计算：`ARC = 2πR/3 ≈ 1.68m`（R=0.8m时）

### 操作流程

**无需勘线！** 段序列由编译期参数自动生成。

1. `KEY2 双击` 两次 → 串口 `Active subject: 2`
2. 将车放在发车区，面朝八字中心
3. `KEY3 长按` → 启动
4. 车辆自动完成八字：直行→绕A→绕B→回发车区
5. 完成后自动停车

**紧急停止**：运行中 `KEY3 长按`

### 串口输出示例

```
[KEY] Active subject: 2
[SUBJ2] 8 IMU segments configured: R=0.80m, arc=1.68m, total≈11.1m
[SUBJ2] Control mode → LOW_SPEED (was 1)
[SUBJ2] START! Figure-8 @ 200 RPM, R=0.80m
[NAV] IMU -> seg 2/8 target=-120.0 deg
[NAV] IMU -> seg 3/8 target=-240.0 deg
...
[NAV] IMU navigation done
[SUBJ2] DONE! Figure-8 complete. Ctrl mode restored to 1
```

### 调参建议

| 症状 | 调整 |
|------|------|
| 八字偏大出圈 | 减小 `SUBJ2_TURN_RADIUS_M`（如 0.7m） |
| 转弯不够快 | 增大 `g_nav_heading_gain`（从 0.3 提到 0.5~0.8） |
| 低速摔车 | 增大 `SUBJ2_RPM`（如 250），或调 LOW_SPEED 模式的 kp/ki |
| 八字结束位置偏移 | IMU yaw 漂移导致，可微调 `SUBJ2_TURN_RADIUS_M` 补偿 |

### 重要注意事项

- 科目2使用 LOW_SPEED 控制模式，启动前的平衡状态可能与科目1/3不同，需单独调 kp/ki
- `g_nav_heading_gain` 在低速时可能需要比科目1大（0.5~0.8），因为低速产生的侧倾力更小
- IMU yaw 无磁力计，长时间绕桩（>30s）漂移会积累，保持合理速度以缩短总时间

---

## 13. 科目3 使用指南

### 赛题概要

科目3：从发车区出发，途经**坡道、草地、锥桶**，到达掉头区，原路返回发车区。距离20~30m，锥桶间距2~5m。

### 与科目1的区别

| 项 | 科目1 | 科目3 |
|---|---|---|
| 路径 | 纯直线 | 有坡道/草地/锥桶 |
| 路点数 | 2（起点+掉头点） | N（起点+多个障碍间隙+掉头点） |
| 回程 | 直接GPS回起点 | **反向经过相同路点**（避障） |
| 数据隔离 | Flash page 11 | Flash page 10 |
| 默认速度 | 600 RPM | 500 RPM（颠簸路面更保守） |

### 科目切换

**KEY2 双击**切换活跃科目（1 ↔ 3），串口打印 `[KEY] Active subject: 3`。切换后 KEY3/KEY4 长按的行为跟随当前科目。

### 参数配置（`code/app/subject3_app.h`）

```c
#define SUBJ3_GO_RPM              500.0f   // 去程/回程常速（RPM）
#define SUBJ3_MID_RPM             350.0f   // 中速过渡
#define SUBJ3_TURN_RPM            250.0f   // 掉头低速（RPM）
#define SUBJ3_PRE_BRAKE_DIST_M    5.0f     // 掉头前减速距离（m）
#define SUBJ3_FINISH_BRAKE_DIST_M 4.0f     // 终点制动距离（m）
#define SUBJ3_RESUME_THRESH_DEG   30.0f    // 出弯加速阈值（°）
#define SUBJ3_ACCEL_STEP_RPM      40.0f    // 出弯加速步进（RPM/100ms）
#define SUBJ3_WP_RADIUS           2.0f     // 路点到达半径（m）
```

### 操作流程

**第一步：切换到科目3模式**
- `KEY2 双击` → 串口打印 `Active subject: 3`

**第二步：勘线（KEY4长按，多路点）**
1. 车放在发车区，GPS锁星
2. `KEY4 长按（第1次）` → 记录 ref_start → 串口：`Survey: ref_start recorded`
3. 推车到第一个障碍前（如坡道前）→ `KEY4 长按` → 记录 WP#1
4. 推车到下一个关键位置（如草地出口）→ `KEY4 长按` → 记录 WP#2
5. 继续推车到每个锥桶间隙 → `KEY4 长按` → 记录 WP#3, WP#4, ...
6. 最后推车到掉头区 → `KEY4 长按` → 记录最后路点（掉头点）
7. 每次记录路点后自动保存到 Flash page 10

**第三步：比赛**
1. 将车推回发车区
2. `KEY3 长按` → 启动科目3
3. 车辆自动按路点序列导航：WP0 → WP1 → ... → 掉头点
4. 到达掉头点后低速掉头
5. 回程自动反向经过路点：WPN-2 → ... → WP0 → ref_start → 停车

**紧急停止**：运行中 `KEY3 长按`

### 回程路径自动生成逻辑

```
去程：[WP0, WP1, WP2, ..., WPN-1(掉头)]
      ↓ nav_prepare_return_path() 自动生成
回程：[WPN-2, WPN-3, ..., WP0, ref_start]
```

- 跳过掉头点（WPN-1）：已到达，无需再导航
- 反转中间路点：保证回程经过相同的坡道/草地/锥桶间隙
- 追加 ref_start：确保回到发车区

### 串口输出示例

```
[KEY] Active subject: 3
[SUBJ3] Survey: ref_start recorded (lat=30.0000001 lon=114.0000001)
[SUBJ3] Survey: WP#1 recorded. Total=1
[SUBJ3] Survey: WP#2 recorded. Total=2
[SUBJ3] Survey: WP#3 recorded. Total=3
[SUBJ3] START! GO → 3 waypoints @ 500 RPM
[NAV] -> Waypoint 2/3
[NAV] -> Waypoint 3/3
[SUBJ3] → GO_BRAKE @ dist=4.5m (last WP)
[NAV] GPS navigation done
[NAV] Return path prepared: 3 WPs (reversed + ref_start)
[SUBJ3] → UTURN @ 250 RPM
[SUBJ3] → RETURN, heading_err=22.5°
[NAV] -> Waypoint 2/3
[NAV] -> Waypoint 3/3
[NAV] GPS navigation done
[SUBJ3] DONE! Return complete.
```

### Flash 持久化行为

- 科目3路点独立存储在 Flash page 10（magic=`0xCAFE3333`）
- 科目1路点存在 Flash page 11（互不干扰）
- 上电后 `subject3_app_init()` 自动从 page 10 恢复路点
- 恢复成功后无需重新勘线，直接 KEY3 启动

### 调参建议

| 症状 | 调整 |
|------|------|
| 撞到锥桶 | 减小 `SUBJ3_WP_RADIUS`（更精确到达），或增加路点密度 |
| 坡道冲太快 | 降低 `SUBJ3_GO_RPM`，或在坡道前后各加一个路点 |
| 掉头冲过 | 增大 `SUBJ3_PRE_BRAKE_DIST_M` |
| 出弯翻车 | 减小 `SUBJ3_ACCEL_STEP_RPM`，增大 `SUBJ3_RESUME_THRESH_DEG` |

---

## 14. Flash 持久化使用指南

### 存储内容

| 类别 | 字段 |
|------|------|
| 电机PID | kp / ki / kd / output_limit / target_rpm |
| 舵机外环PID | kp / ki / kd / output_limit / integral_limit |
| 舵机内环PID | inner_kp / inner_ki（串级模式）|
| 舵机机械限位 | left / mid / right duty |
| 控制模式 | SIMPLE_PD / CASCADE / LOW_SPEED |
| 导航增益 | g_nav_heading_gain |
| 科目1参数 | high / mid / turn / pre_brake / finish_brake / resume / accel |
| 科目2参数 | rpm / turn_radius |
| 科目3参数 | go / mid / turn / pre_brake / finish_brake / resume / accel |
| GPS勘线数据 | 参考起点坐标 + 路点数组（最多20个）|

**不存储**：传感器实时数据、yaw零偏（每次上电自动校准）

### 触发机制

| 触发方式 | 场景 |
|------|------|
| KEY4长按第2次（勘线完成）| 自动保存GPS路点+全部参数 |
| KEY1 双击 | WiFi/IPS调参后手动持久化 |

### Flash 地址

使用逐飞 DFlash 第11页（`0xAF000000 + 11×8KB`），与程序区完全隔离。校验：魔数 `0xBEEF5A5A` + uint32 累加和。

### 恢复出厂默认

```c
// 调用即可擦除Flash配置页，下次上电使用编译期默认值
config_flash_erase();
```

---

## 15. BUG 修复记录（更新至 2026-04-06）

以下问题经全面系统审计发现并已修复：

### 致命级（CRITICAL）

| 编号 | 问题 | 修复文件 | 说明 |
|------|------|---------|------|
| C1 | GNSS double 竞态 | `nav_app.c` | TC264D 32 位 MCU 读写 64 位 double 需 2 次内存操作，UART3 RX ISR 可在中间更新 gnss 导致坐标跳变。修复：所有 gnss double 读取加 `interrupt_global_disable/enable` 临界区 |
| C2 | `target_motor_rpm` 缺 volatile | `motor_pid.c/h` | 变量在 20ms ISR 中读、主循环中写，无 volatile 编译器可能缓存旧值。修复：加 `volatile` 修饰 |
| C3 | TX ISR 错误调用接收回调 | `isr.c:272` | UART3 发送完成中断错误调用 `gnss_uart_callback()`，与 RX ISR 形成重入风险。修复：移除该调用 |

### 高危级（HIGH）

| 编号 | 问题 | 修复文件 | 说明 |
|------|------|---------|------|
| H2 | GPS→IMU 航向切换跳变 | `nav_app.c` | 低速时直接用 `yaw_kalman`（相对值），与 GPS 绝对航向坐标系不同。修复：记录最后可信 GPS 航向，低速时用「绝对基准 + IMU 变化量」 |
| H3 | 模式切换积分未清零 | `wireless_debug_app.c` | 无线调参切换控制模式时 `balance_pid.integral` 继承旧值，导致舵机猛打。修复：切换时立即清零外环积分 |

### 中危级（MEDIUM）

| 编号 | 问题 | 修复文件 | 说明 |
|------|------|---------|------|
| M2 | `nav_wrap_angle` 边界不对称 | `nav_app.c` | `< -180` 改为 `<= -180`，保证 ±180° 对称 |
| M5 | 参数 printf 阻塞调度 | `wireless_debug_app.c/h` | 新增 `WIRELESS_DEBUG_PRINTF_ENABLE` 宏，比赛模式置 0 禁用 printf |
| M6 | 里程矩形积分精度差 | `nav_app.c` | 改为梯形积分 `(v_now + v_last) / 2 × dt`，加减速段误差减半 |

### 已识别问题当前结论

| 编号 | 问题 | 优先级 | 说明 |
|------|------|--------|------|
| H1 | 5ms ISR 执行时间过重 | 已验证通过 | 已在 `user/isr.c` 增加 5ms 控制 ISR 的 `last/max/min/run_count` 统计，实测 `g_ctrl_5ms_max_us = 39`，相对 5ms 周期占用约 0.78%，当前不是风险点 |
| H4 | PID/路点参数掉电丢失 | 已修复 | 已新增 `config_flash.c/h`，上电自动恢复 PID、舵机限位和 GPS 勘线数据 |
| M1 | GPS 经度漂移修正近似误差 | 中 | 直接在度上做加减，经度 1° 实距随纬度变化，场地 <200m 可接受 |
| M3 | 速度数据 20ms→5ms 延迟 | 已修复 | LQR 改为在 5ms 控制周期直接读取最新速度接口，不再依赖 20ms 缓存 |

### 本轮集成修复（2026-04-03）

| 编号 | 问题 | 修复文件 | 说明 |
|------|------|---------|------|
| I1 | 舵机 PWM 标度分裂 | `servo_app.h/.c` | 统一 PID / LQR / 舵机驱动使用同一组运行时边界；BDS300 默认值修正为 3300 / 4950 / 6600 |
| I2 | 导航误差未接入闭环 | `user/isr.c` | 在 5ms 控制节拍中统一把 `g_nav_heading_error * g_nav_heading_gain` 写入 `expect_angle` 和 `lqr_expect_phi` |
| I3 | UART2 服务核与 ISR 归属不一致 | `user/isr_config.h` | 将 `UART2_INT_SERVICE` 对齐到 CPU0，避免中断源注册到 CPU1 而入口函数定义在 CPU0 |
| I4 | 科目1勘线状态误推进 | `subject1_app.c` | 勘线第1次/第2次只有在参考起点或路点真正记录成功时才推进状态，避免 GPS 无效时误判“勘线完成” |
| I5 | PID/LQR 停止时残留导航参考 | `subject1_app.c` | 科目1停止和完成时同时清零 `expect_angle` 与 `lqr_expect_phi`，避免残留转向参考 |
| I6 | `balance_set_expect_angle` 链接缺失 | `servo_pid.c` | 补回 `balance_set_expect_angle()` 和 `balance_set_params()` 实现，修复链接错误 |

---

## 16. 已知现状与后续建议

### 当前已知现状

| 项目 | 状态 |
|------|------|
| CPU1 | 入口保留，无实际逻辑 |
| yaw 漂移 | 无磁力计，偏航角为相对值，长期不可靠 |
| 零偏校准 | 使用临时写死值，精度受温度影响 |
| 路点掉电丢失 | 已修复，勘线数据与 PID 参数均可从 Flash 恢复 |
| 控制层导航接入 | 已接入 `user/isr.c` 的 5ms 控制节拍，PID/LQR 共用同一导航参考 |
| 5ms ISR 耗时观测 | 已接入 `TEST` 页面，可看 `5ms_last / 5ms_max / 5ms_min / 5ms_cnt` |
| 串级 PID 内环默认值 | `inner_kp=1.0, inner_ki=0.0` 仅为量级参考，实车需重新调参 |

### 后续建议

1. **陀螺零偏 Flash 存储**：`imu_calibrate_gyro()` 结果写 Flash，下次上电直接读取

2. **引入磁力计**：为 `yaw_kalman` 提供绝对参考，改善 IMU 模式导航长期精度

3. **LQR 实车调参**：当前增益表为默认物理参数生成，需测量实车 `h`（质心高度）、`L`（轴距）后重新运行 `lqr_gain_generator.py`

4. **比赛模式配置**：`wireless_debug_app.h` 中 `WIRELESS_DEBUG_PRINTF_ENABLE` 当前默认已为 `0U`，需要串口调试时再临时改回 `1U`

---

## 17. 赛前验证顺序

建议按下面顺序上车验证，优先处理真正影响比赛结果的链路：

1. **编译与基础启动**
   确认工程可完整编译，上电后 `Init OK` 正常打印，LED 进入初始化完成状态。

2. **5ms 控制链路**
   进入 IPS `TEST` 页面，观察 `5ms_last / 5ms_max / 5ms_min / 5ms_cnt`。
   当前实测 `g_ctrl_5ms_max_us = 39`，若后续改 LQR/导航后显著升高，再重新评估。

3. **科目1勘线流程**
   在 GPS 无效状态下测试 `KEY4`，确认不会误推进到下一步。
   在 GPS 有效状态下完整勘线两次，确认参考起点、路点和 Flash 保存都正常。

4. **Flash 恢复**
   断电重启后确认 `[FLASH] Loaded OK` 正常出现，且科目1可直接 `KEY3` 启动，不需要重新勘线。

5. **电机反馈链路**
   重点确认 `target_motor_rpm`、`motor_speed_rpm`、`g_motor_pid_output` 三者关系正常，速度反馈不再长期为 0。

6. **科目2低速长时间运行**
   这是当前剩余的主要系统性风险，不在 ISR 算力，而在 6 轴 IMU 的长期偏航漂移。
   建议单独做长时间低速绕圈/绕八字测试，评估 `yaw_kalman` 漂移速度和控制可接受时长。

---

*最后更新：2026-04-06 · 作者：闫锦*
