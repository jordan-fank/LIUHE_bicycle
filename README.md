# LIUHE_bicycle1

## 1. 项目简介

本项目是一个基于 Infineon AURIX TC26xB 平台的嵌入式工程，实现摩托车结构转向平衡车的姿态采集与舵机平衡控制，主要功能如下：

- IMU660RB 六轴姿态采集与姿态解算（Madgwick + PI 融合）
- 舵机平衡控制（PID 保留，LQR 已接入并可切换）
- IPS200 屏幕显示与参数页面切换
- 4 个按键的人机交互
- 基于主循环的简单任务调度

从工程配置可确认，当前编译目标为 `tc26xb`，使用 TASKING 工具链。

---

## 2. 当前工程功能

### 2.1 IMU 功能

- IMU 设备使用 `imu660rb`
- 上电后先完成 IMU 初始化，再进行陀螺仪零偏校准
- 姿态解算周期为 `5ms`
- 输出数据包括：

| 变量 | 含义 |
|------|------|
| `roll_kalman` | IMU 原始横滚角（含机械装配偏差） |
| `pitch_kalman` | IMU 原始俯仰角 |
| `yaw_kalman` | IMU 相对偏航角（无磁力计，为相对值） |
| `gyro_x_rate` | X 轴角速度（deg/s，已减零偏） |
| `gyro_y_rate` | Y 轴角速度（deg/s，已减零偏） |
| `roll_ctrl_angle` | 控制用横滚角（扣除机械零位后） |
| `pitch_ctrl_angle` | 控制用俯仰角（扣除机械零位后） |

当前姿态算法位于 `code/driver/imu/mcu_dmp.c`，主要包含：

- 四元数姿态更新
- 静止状态 PI 修正（`gyro_magnitude < 0.03`）
- 运动状态 Madgwick 融合
- 偏航角扩展卡尔曼滤波
- 欧拉角边界归一化

说明：

- 初始姿态通过加速度计重力方向初始化，因此上电不是简单全零姿态
- 当前代码未使用磁力计，因此 `yaw` 为相对偏航，不是绝对航向角
- `roll_kalman` / `pitch_kalman` 保留为 IMU 原始姿态输出
- `roll_ctrl_angle` / `pitch_ctrl_angle` 用于机械零位补偿后的控制角度

### 2.2 舵机平衡控制

当前工程同时保留两套舵机平衡方案：

- `code/app/servo_pid.c`：原 PID 平衡控制
- `code/app/lqr_balance.c`：新接入的 LQR 平衡控制
- `code/app/balance_control_mode.h`：PID / LQR 一键切换开关

当前切换方式：

- 修改 `code/app/balance_control_mode.h` 中的 `BALANCE_CONTROL_MODE`
- 设为 `BALANCE_CONTROL_MODE_PID`：使用原 PID，适合台架联调、无速度测试
- 设为 `BALANCE_CONTROL_MODE_LQR`：使用 LQR，适合实车前进平衡测试
- 当前仓库默认值为 `BALANCE_CONTROL_MODE_PID`
- LQR 默认物理参数和默认增益表生成条件现在统一放在 `code/driver/lqr/lqr_driver.h`，应用层不再单独维护第二套默认值

当前 `user/isr.c` 的 5ms PIT 中断里，通过条件编译在 `balance_control()` 和 `lqr_balance_control()` 之间二选一，原 PID 方案完整保留，便于后续回切对比。

**PID 参数（当前值）：**

| 参数 | 值 | 说明 |
|------|----|------|
| `BALANCE_KP` | 20.0 | 角度比例系数 |
| `BALANCE_KI` | 0.25 | 积分系数（消除静差） |
| `BALANCE_KD` | 0.05 | D 项直接使用陀螺仪角速度 |
| `BALANCE_LIMIT` | 250.0 | PID 输出限幅 |
| `INTEGRAL_LIMIT` | 200.0 | 积分限幅（防饱和） |

**使能控制：**

```c
balance_control_set_enable(0U);   // 关闭 PID（PID 复位，舵机归中）
balance_control_set_enable(1U);   // 开启 PID

lqr_balance_set_enable(0U);       // 关闭 LQR（LQR 复位，舵机归中）
lqr_balance_set_enable(1U);       // 开启 LQR
```

**注意**：必须在 `zero_compensation()` 完成后再开启，防止舵机动作干扰零位捕获。

**当前舵机角度读取：**

全局变量 `pwm_angle`（单位 °）实时反映当前舵机输出角度，可通过屏幕或调试串口监视。

### 2.3 电机速度单位与车速换算

当前项目已经确认：

- `motor_get_raw_rpm()` 返回的是电机转速 `RPM`
- `motor_get_raw_rpm()` 保留驱动原始符号，应用层通过 `motor_get_vehicle_rpm()` 统一成“车辆前进为正”
- 本车为轮驱一体，`MOTOR_GEAR_RATIO = 1`
- 车轮直径 `64mm`，对应半径 `0.032m`

因此：

- `code/app/motor_app.c` 中统一提供了：
  - `motor_get_vehicle_rpm()`：按车体方向统一后的 RPM
  - `motor_get_vehicle_speed_m_s()`：带方向线速度
  - `motor_get_vehicle_speed_abs_m_s()`：速度大小
- `code/app/lqr_balance.c` 不再自己重复写速度换算公式，而是直接调用电机层的 `motor_get_vehicle_speed_abs_m_s()`
- `code/app/ips_app_config.h` 的电机页面标签已改为 `target_rpm` / `real_rpm` / `target_m_s` / `real_m_s`

### 2.4 舵机型号选择

在 `code/app/servo_app.h` 中通过宏选择：

```c
#define CURRENT_SERVO_TYPE    SERVO_TYPE_MG996R   // 调试用 50Hz
// #define CURRENT_SERVO_TYPE    SERVO_TYPE_BDS300   // 比赛用 330Hz
```

| 型号 | 频率 | 占空比范围 | 角度范围 |
|------|------|-----------|---------|
| MG996R | 50Hz | 250 ~ 1250 | ±90° |
| BDS300 | 330Hz | 3300 ~ 6600 | 45° ~ 135°（机械限位） |

角度与占空比互转宏（通用）：

```c
SERVO_DUTY_TO_ANGLE(duty)    // 占空比 → 角度（°）
SERVO_ANGLE_TO_DUTY(angle)   // 角度（°） → 占空比
```

### 2.5 IPS 页面

当前 IPS 页面定义在 `code/app/ips_app_config.h`，共有 5 页：

1. `Motor PID`
2. `Servo PID`
3. `GPS`
4. `IMU`
5. `TEST`

其中：

- `Motor PID`、`Servo PID` 页面支持参数调整
- `GPS`、`IMU`、`TEST` 页面为只读显示

IMU 页面当前显示：`roll`、`pitch`、`yaw`、`gyro_x`、`gyro_y`、`calibrated`、`roll_ctrl`、`pitch_ctrl`、`pwm_angle`、`drop_count`

### 2.6 按键映射

按键逻辑位于 `code/app/key_app.c`：

| 按键 | 操作 | 功能 |
|------|------|------|
| KEY1 | 短按 / 长按 | 减小当前参数 / 连续减小 |
| KEY2 | 短按 / 长按 | 增大当前参数 / 连续增大 |
| KEY3 | 短按 | 切换到下一页 |
| KEY4 | 短按 / 双击 | 选择下一项 / 选择上一项 |

按键驱动默认参数（`code/driver/key/key_driver.h`）：消抖 20ms，长按 800ms，双击 300ms。

---

## 3. 软件架构

工程大致分为 4 层：

### 3.1 启动与中断层

Path: `user/`

- `user/cpu0_main.c`：CPU0 主入口
- `user/isr.c`：所有中断服务函数
- `user/isr_config.h`：中断优先级配置

当前主业务运行在 `CPU0`，`CPU1` 入口保留但未承载实际逻辑。

### 3.2 应用层

目录：`code/app/`

| 文件 | 功能 |
|------|------|
| `imu_app.c` | IMU 应用层封装、零偏校准、机械零位补偿 |
| `servo_pid.c` | 舵机 PID 平衡控制 |
| `lqr_balance.c` | LQR 平衡控制应用层接入 |
| `ips_app.c` | IPS 页面与显示逻辑 |
| `key_app.c` | 按键业务逻辑 |
| `scheduler.c` | 简单调度器 |
| `led_app.c` | LED 测试逻辑 |

### 3.3 自定义驱动层

目录：`code/driver/`

- `imu/mcu_dmp.c`：姿态融合算法（Madgwick + PI + EKF）
- `lqr/lqr_driver.c`：LQR 控制器驱动层
- `ips/ips_driver.*`：IPS 基础绘图接口
- `key/key_driver.*`：按键驱动
- `led/led_driver.*`：LED 驱动
- `pid/pid_driver.*`：通用 PID 驱动

### 3.4 底层库

目录：`libraries/`

包含 Infineon iLLD / Infra / Service、`zf_common`、`zf_driver`、`zf_device`、`zf_components`。

`libraries/zf_common/zf_common_headfile.h` 统一聚合了项目常用头文件。

---

## 4. 启动流程

`CPU0` 主流程位于 `user/cpu0_main.c`，当前启动顺序如下：

1. `clock_init()` — 时钟初始化
2. `debug_init()` — 调试串口初始化
3. `led_init()` / `key_app_init()` / `ips_app_init()` — 外设初始化
4. `pwm_init()` / `servo_init()` — 舵机 PWM 初始化（含 2s 预热）
5. `balance_pid_init()` / `lqr_balance_init()` — 初始化 PID 与 LQR，两套方案并存
6. `balance_control_set_enable(0)` / `lqr_balance_set_enable(0)` — **先关闭控制输出**，防止干扰 IMU 校准
7. `imu_all_init()` — IMU 硬件初始化 + 四元数初始化
8. `imu_calibrate_gyro_temp()` — 加载零偏临时值（快速启动）
9. `pit_ms_init(CCU60_CH1, 5ms)` — 启动 IMU 周期采样中断
10. `zero_compensation()` — 等待 100 帧有效姿态后捕获机械零位
11. `balance_control_set_enable(1)` 或 `lqr_balance_set_enable(1)` — 实际启用哪套方案由 `balance_control_mode.h` 决定
12. `scheduler_init()` — 调度器初始化
13. `led_all_set(LED_ON)` — LED 点亮，作为初始化完成标志
14. 主循环：`imu_proc()` 优先执行，然后 `scheduler_run()`

---

## 5. IMU 运行说明

### 5.1 安装方向要求

`code/app/imu_app.c` 中明确要求 IMU 安装方向如下：

- X 轴指向车辆前进方向
- Y 轴指向车辆左侧
- Z 轴垂直向上

如果安装方向与上述定义不一致，需要在 `imu_app.c` 中对原始数据取负号处理。

### 5.2 校准方式

当前代码提供两种陀螺仪零偏方式：

| 函数 | 耗时 | 说明 |
|------|------|------|
| `imu_calibrate_gyro()` | ~60s | 实时采样 30000 次，精度高 |
| `imu_calibrate_gyro_temp()` | 0 | 写死历史零偏值，适合快速调试 |

在 `user/cpu0_main.c` 中可二选一：

```c
// imu_calibrate_gyro();         // 实时校准（精度高，启动慢）
imu_calibrate_gyro_temp();       // 临时零偏（启动快，仅用于调试）
```

### 5.3 IMU 中断与主循环分工

当前 `CCU60_CH1` 的 5ms PIT 中断位于 `user/isr.c`：

```c
imu_sample_isr();         // 采样原始数据，写入共享缓冲区
#if (BALANCE_CONTROL_MODE == BALANCE_CONTROL_MODE_PID)
balance_control();        // PID 平衡控制
#else
lqr_balance_control();    // LQR 平衡控制
#endif
```

`imu_proc()` 在主循环中执行：

- 从共享缓冲区复制最新采样（关中断临界区内完成）
- 清除 `g_imu_sample_ready` 标志
- 单位换算与零偏补偿
- 姿态融合（Madgwick / PI）
- 欧拉角输出更新

**关键设计原则**：

- ISR 中只做最小工作（采样 + 平衡控制输出，当前为 LQR），姿态解算在主循环执行
- `g_imu_sample_ready` 的清零与原始数据复制必须在同一个关中断临界区内完成
- 主循环的 `imu_proc()` 优先于 `scheduler_run()` 执行，确保每帧都能被及时消费

### 5.4 zero_compensation() — 机械零位捕获

`zero_compensation()` 在平衡控制使能前调用，等待 100 帧有效姿态后自动捕获当前朝向为控制零位。

**实现要点**（已修复历史 BUG）：

旧实现使用 `imu_proc() + system_delay_ms(5ms)` 循环，导致循环周期 = `T_proc + 5ms > 5ms`，与 PIT 5ms 节拍之间产生累计相位漂移。漂移积累后 PIT 会在 `imu_proc()` 执行期间触发，造成初始化阶段连续丢帧。

当前实现改为事件驱动模式，与主循环行为一致：

```c
while (processed < 100) {
    imu_proc();
    if (process_count 增加) processed++;
}
imu_capture_control_zero();
```

**使用建议**：捕获期间保持 IMU 静止，否则会把运动中的姿态错误记录为零位。

### 5.5 CPU0 DSPR 变量放置原则

- `Lcf_Tasking_Tricore_Tc.lsl` 中保持 `LCF_DEFAULT_HOST = LCF_CPU1`
- 只将 `g_imu_raw_sample`、`g_imu_sample_ready`、`g_imu_diag_stat`、`roll_ctrl_angle`、`gyro_x_rate` 放入 `cpu0_dsram`
- 上述 5 个变量属于 `CPU0 5ms ISR <-> 主循环` 的共享热路径，手工放入 `cpu0_dsram` 有明确收益
- 其余 IMU 算法状态、IPS 显示变量和普通全局变量保持默认宿主段，避免因大范围迁移引入隐蔽的数据布局和显示问题

### 5.6 机械零位补偿

当前工程将 IMU 原始姿态与控制零位补偿分开处理：

- `roll_kalman` / `pitch_kalman` — 用于观察 IMU 原始输出
- `roll_ctrl_angle` / `pitch_ctrl_angle` — 用于控制环（已扣除机械零位）

手动设置零位：

```c
imu_set_control_zero(4.27f, 0.38f);   // 静止时测得的原始角度
```

自动捕获当前姿态为零位：

```c
imu_capture_control_zero();
```

### 5.7 IMU 丢帧诊断

工程保留了调试统计结构体 `g_imu_diag_stat`（`imu_app.h`）：

| 字段 | 含义 |
|------|------|
| `sample_count` | ISR 触发总次数 |
| `process_count` | 主循环实际处理帧数 |
| `drop_count` | 丢帧次数（主循环未及时消费） |

正常运行时 `drop_count` 应接近 0。该统计标记为 `TEST ONLY`，后续可整体移除。

---

## 6. 调度器说明

当前调度器位于 `code/app/scheduler.c`，属于主循环轮询调度。

当前任务表：

| 任务 | 周期 | 说明 |
|------|------|------|
| `key_scan()` | 10ms | 按键状态采集 |
| `key_task()` | 10ms | 按键业务处理 |
| `ips_app_task()` | 200ms | 屏幕局部刷新 |

说明：

- IMU 采样由 5ms 中断驱动，不进入调度表
- IMU 解算在主循环最高优先级执行（`scheduler_run()` 之前）
- IPS 刷新采用局部更新，避免整屏闪烁

---

## 7. 编译与调试

### 7.1 工具链

- AURIX Development Studio / Eclipse CDT 工程结构
- TASKING 编译工具链
- Debug / Release 两套构建配置

### 7.2 目标配置

- Target CPU：`tc26xb`
- Package：`bga292`

### 7.3 调试入口

工程包含共享启动配置文件 `LIUHE_bicycle1 Debug.launch`，停止入口为 `core0_main`。

---

## 8. 目录结构

```text
LIUHE_bicycle1/
├─ code/
│  ├─ app/
│  │  ├─ imu_app.c / .h        # IMU 应用层
│  │  ├─ servo_pid.c / .h      # 舵机 PID 平衡控制
│  │  ├─ lqr_balance.c / .h    # LQR 平衡控制应用层
│  │  ├─ servo_app.c / .h      # 舵机硬件驱动封装
│  │  ├─ ips_app.c / .h        # 屏幕应用层
│  │  ├─ ips_app_config.h      # 屏幕页面配置
│  │  ├─ key_app.c / .h        # 按键业务逻辑
│  │  ├─ scheduler.c / .h      # 主循环调度器
│  │  └─ led_app.c             # LED 测试
│  ├─ tools/                   # LQR 来源脚本、移植说明和参考资料
│  └─ driver/
│     ├─ imu/
│     │  ├─ mcu_dmp.c / .h     # 姿态融合算法
│     │  └─ ekf.c / .h         # EKF（备用）
│     ├─ lqr/                  # LQR 控制器驱动层
│     ├─ pid/pid_driver.*      # 通用 PID
│     ├─ ips/                  # IPS 基础驱动
│     ├─ key/                  # 按键驱动
│     └─ led/                  # LED 驱动
├─ libraries/
│  ├─ infineon_libraries/      # iLLD / Infra / Service
│  ├─ zf_common/
│  ├─ zf_driver/
│  ├─ zf_device/
│  └─ zf_components/
├─ user/
│  ├─ cpu0_main.c              # CPU0 主入口
│  ├─ isr.c                    # 中断服务函数
│  └─ isr_config.h             # 中断优先级配置
├─ 推荐IO分配.txt
└─ README.md
```

---

## 9. 当前已知现状

- 当前工程主业务集中在 `CPU0`，`CPU1` 入口保留但无实际逻辑
- `yaw` 未引入磁力计绝对参考，只能作为相对偏航
- 上电实时零偏校准（`imu_calibrate_gyro()`）耗时约 60s，日常调试使用临时值
- 姿态解算 CPU 占用未精确测量，如发现丢帧可通过 `g_imu_diag_stat` 诊断

## 10. 后续建议

- 引入磁力计为 yaw 提供绝对参考
- 将陀螺仪零偏保存到 Flash，避免每次上电重新校准
- 为 PID 参数增加运行时持久化（Flash 存储）
- 完善 CPU1 的分担策略（如将姿态解算移到 CPU1）

---

## 11. LQR 接入现状

- 当前工程已经保留两套舵机平衡方案：`PID` 和 `LQR`
- 当前 5ms 中断控制入口由 `code/app/balance_control_mode.h` 统一切换，当前默认配置为 `PID`
- 原 `balance_control()` 和 `lqr_balance_control()` 都没有删除，仍保留在工程中，便于后续做 A/B 对比
- 当前 LQR 接入位置：
  - 应用层：[lqr_balance.c](/E:/AURIX-v1.10.28-workspace/LIUHE_bicycle1/code/app/lqr_balance.c)
  - 驱动层：[lqr_driver.c](/E:/AURIX-v1.10.28-workspace/LIUHE_bicycle1/code/driver/lqr/lqr_driver.c)
  - 中断入口：[isr.c](/E:/AURIX-v1.10.28-workspace/LIUHE_bicycle1/user/isr.c)
- 当前 LQR 已按本项目实际平衡轴改为读取 `roll_ctrl_angle` 和 `gyro_x_rate`
- 当前 LQR 参数仍以“移植默认值 + 最小可运行接入”为主，不代表最终可比赛参数
- `code/tools/` 目录保存了这套 LQR 的来源材料：
  - `lqr_gain_generator.py`：离线增益表生成脚本
  - `README.md`：原始移植说明
  - `README.pdf`、`纯舵机驱动单车控制系统的LQR模型.pdf`：理论与移植参考资料
- 两个 PDF 读完后，可以确认当前 LQR 使用的核心模型与来源文档一致：
  - 状态：`x = [phi, phi_dot, delta]`
  - 输入：`u = delta_dot`
  - 连续模型：`A = [[0,1,0],[g/h,0,-v^2/(hL)],[0,0,0]]`
  - 连续模型：`B = [[0],[-b*v/(hL)],[1]]`
- 当前 `code/driver/lqr/lqr_driver.c` 中的 `default_gain_entries[]` 与 `code/tools/lqr_gain_generator.py` 的默认生成条件一致：
  - 物理参数：`h=0.12m`、`w=0.21m`、`b=0.02m`、`g=9.8m/s^2`
  - 权重：`Q=diag(100, 15, 30)`、`R=2`
  - 速度范围：`0.5~7.0m/s`
  - 速度点数：`15`
- 需要特别注意脚本与工程的命名差异：
  - 脚本中的 `w` 表示轴距
  - 工程中的 `L` 也表示轴距
  - 两者是同一个物理量，只是命名不同
- 当前项目已经不是 `code/tools/README.md` 里的“extern 模板式接入”：
  - 不再要求单独实现 `imu_get_roll()`、`imu_get_roll_rate()`、`servo_set()` 这一套移植模板接口
  - 现在由 `lqr_balance.c` 直接对接项目现有的 IMU、电机、舵机接口
- 当前项目也没有照搬 PDF 里的 `pitch_kalman / gyro_y_rate` 示例代码：
  - PDF 中这组变量是原始移植包的示例轴
  - 本工程已经按你现有 PID 验证结果，改为 `roll_ctrl_angle / gyro_x_rate`
  - 这一点属于“按实车安装方向和现有工程坐标系做的项目级适配”，不是偏离模型本身
- 如果 `README.pdf`、`纯舵机驱动单车控制系统的LQR模型.pdf` 里的示例代码，与仓库中的 `lqr_gain_generator.py` 或当前工程实现不一致，应以仓库内这两类内容为准：
  - 可执行来源：`code/tools/lqr_gain_generator.py`
  - 当前接入结果：`code/app/lqr_balance.c`、`code/driver/lqr/lqr_driver.c`
- 一个已经确认的例子：
  - 理论 PDF 中有一段示例脚本使用 `np.arange(0.5, 7.5, 0.5)` 生成速度点
  - 但当前仓库实际提供的 `lqr_gain_generator.py` 默认是“`0.5~7.0m/s`、`15` 点、线性均分”
  - 当前 `default_gain_entries[]` 也与后者一致
  - 因此后续生成新表时，不要照抄 PDF 里的那段示例，应直接运行仓库里的脚本
- 如果后续修改了以下任一条件：
  - `h / L / b`
  - `Q / R`
  - 速度范围或速度点数
  则必须重新运行 `code/tools/lqr_gain_generator.py`，并同步替换 `code/driver/lqr/lqr_driver.c` 中的 `default_gain_entries[]`
- 只修改运行时物理参数宏，并不会自动重算增益表；那样只能改变部分应用层计算和默认初值，不能让 LQR 真正匹配新车模
- 详细说明、调参逻辑和实测方法见 [LQR_GUIDE.md](/E:/AURIX-v1.10.28-workspace/LIUHE_bicycle1/LQR_GUIDE.md)
