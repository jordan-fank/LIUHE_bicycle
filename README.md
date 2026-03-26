# LIUHE_bicycle1

## 1. 项目简介

本项目是一个基于 Infineon AURIX TC26xB 平台的嵌入式工程，当前代码主线围绕以下功能展开：

- IMU660RB 六轴姿态采集与姿态解算
- IPS200 屏幕显示与参数页面切换
- 4 个按键的人机交互
- 基于主循环的简单任务调度
- 基于 PIT 5ms 中断的 IMU 周期处理

从工程配置可确认，当前编译目标为 `tc26xb`，使用 TASKING 工具链。

## 2. 当前工程功能

### 2.1 IMU 功能

- IMU 设备使用 `imu660rb`
- 上电后先完成 IMU 初始化，再进行陀螺仪零偏校准
- 姿态解算周期为 `5ms`
- 输出数据包括：
  - `roll_kalman`
  - `pitch_kalman`
  - `yaw_kalman`
  - `gyro_x_rate`
  - `gyro_y_rate`
  - `roll_ctrl_angle`
  - `pitch_ctrl_angle`

当前姿态算法位于 `code/driver/imu/mcu_dmp.c`，主要包含：

- 四元数姿态更新
- 静止状态 PI 修正
- 运动状态 Madgwick 融合
- 偏航角扩展卡尔曼滤波
- 欧拉角边界归一化

说明：

- 初始姿态通过加速度计重力方向初始化，因此上电不是简单全零姿态
- 当前代码未使用磁力计，因此 `yaw` 为相对偏航，不是绝对航向角
- `roll_kalman` / `pitch_kalman` 保留为 IMU 原始姿态输出
- `roll_ctrl_angle` / `pitch_ctrl_angle` 用于机械零位补偿后的控制角度

### 2.2 IPS 页面

当前 IPS 页面定义在 `code/app/ips_app_config.h`，共有 4 页：

1. `Motor PID`
2. `Servo PID`
3. `Battery`
4. `IMU`

其中：

- `Motor PID`、`Servo PID` 页面支持参数调整
- `Battery`、`IMU` 页面为只读显示

IMU 页面当前显示：

- `roll`
- `pitch`
- `yaw`
- `gyro_x`
- `gyro_y`
- `calibrated`

### 2.3 按键映射

按键逻辑位于 `code/app/key_app.c`。

- `KEY1`
  - 短按：减小当前参数
  - 长按：连续减小当前参数
- `KEY2`
  - 短按：增大当前参数
  - 长按：连续增大当前参数
- `KEY3`
  - 短按：切换到下一页
- `KEY4`
  - 短按：选择下一项
  - 双击：选择上一项

按键驱动默认参数位于 `code/driver/key/key_driver.h`：

- 消抖时间：`20ms`
- 长按判定：`800ms`
- 双击判定：`300ms`

## 3. 软件架构

工程大致分为 4 层：

### 3.1 启动与中断层

Path: `user/`

关键文件：

- `user/cpu0_main.c`
- `user/cpu1_main.c`
- `user/isr.c`
- `user/isr_config.h`

说明：

- 当前主业务运行在 `CPU0`
- `CPU1` 入口保留，但未承载实际业务逻辑

### 3.2 应用层

目录：`code/app/`

主要文件：

- `imu_app.c`：IMU 应用层封装
- `ips_app.c`：IPS 页面与显示逻辑
- `key_app.c`：按键业务逻辑
- `scheduler.c`：简单调度器
- `led_app.c`：LED 测试逻辑

### 3.3 自定义驱动层

目录：`code/driver/`

主要文件：

- `imu/mcu_dmp.c`：姿态融合算法
- `ips/ips_driver.*`：IPS 基础绘图接口
- `key/key_driver.*`：按键驱动
- `led/led_driver.*`：LED 驱动

### 3.4 底层库

目录：`libraries/`

包含：

- Infineon iLLD / Infra / Service
- `zf_common`
- `zf_driver`
- `zf_device`
- `zf_components`

其中 `libraries/zf_common/zf_common_headfile.h` 统一聚合了项目常用头文件。

## 4. 启动流程

`CPU0` 主流程位于 `user/cpu0_main.c`，当前启动顺序如下：

1. `clock_init()` 时钟初始化
2. `debug_init()` 调试串口初始化
3. `led_init()` LED 初始化
4. `key_app_init()` 按键初始化
5. `ips_app_init()` 屏幕初始化
6. `imu_all_init()` IMU 初始化
7. `imu_calibrate_gyro()` 陀螺仪真实零偏校准
8. `pit_ms_init(CCU60_CH1, IMU_PERIOD_MS)` 启动 IMU 周期定时器
9. `zero_compensation()` 捕获机械零位
10. 机械零位捕获完成后再开启平衡控制
11. `scheduler_init()` 调度器初始化
12. `led_test_all()` LED 自检
13. 主循环先执行 `imu_proc()`
14. 主循环再执行 `scheduler_run()`

## 5. IMU 运行说明

### 5.1 安装方向要求

`code/app/imu_app.c` 中明确要求 IMU 安装方向如下：

- X 轴指向车辆前进方向
- Y 轴指向车辆左侧
- Z 轴垂直向上

如果安装方向与上述定义不一致，姿态结果会失真。

### 5.2 校准方式

当前代码提供两种陀螺仪零偏方式：

- `imu_calibrate_gyro()`
  - 实时校准
  - 采样次数：`30000`
  - 间隔：`2ms`
  - 总耗时约 `60s`
  - 精度更高，但启动时间长

- `imu_calibrate_gyro_temp()`
  - 使用临时写死的零偏值
  - 启动快
  - 适合调试阶段

当前如果你的目标是验证 IMU 数据准确性，建议优先使用：

- `imu_calibrate_gyro()`
  - 使用当前上电环境下的真实零偏
  - 更适合判断 IMU 数据是否准确

在 `user/cpu0_main.c` 中可以二选一：

```c
// imu_calibrate_gyro_temp();   // 快速启动
imu_calibrate_gyro();          // 实时校准
```

### 5.3 IMU 中断处理

当前 `CCU60_CH1` 的 PIT 中断位于 `user/isr.c`，中断中只做固定节拍采样：

```c
imu_sample_isr();
```

`imu_proc()` 当前完成：

- 复制 ISR 最近一次采样结果
- 数据单位换算
- 零偏补偿
- 姿态融合
- 欧拉角输出更新

说明：

- ISR 中仅保留采样，降低中断执行负担
- 姿态解算移动到主循环，并在 `scheduler_run()` 前优先执行
- IMU 统一时基由 `IMU_PERIOD_MS` / `IMU_DT_S` 提供，避免多处写死 `0.005f`

### 5.4 机械零位补偿

当前工程将 IMU 原始姿态与控制零位补偿分开处理：

- `roll_kalman` / `pitch_kalman`
  用于观察 IMU 原始姿态与安装方向是否正确
- `roll_ctrl_angle` / `pitch_ctrl_angle`
  用于扣除机械装配零位后的控制角度

设置方式如下：

```c
imu_set_control_zero(4.27f, 0.38f);
```

或在车体放到目标机械零位后调用：

```c
imu_capture_control_zero();
```

建议显示和调试继续看 `roll_kalman` / `pitch_kalman`，控制环优先使用 `roll_ctrl_angle` / `pitch_ctrl_angle`。

另外，建议上电时先关闭平衡控制，待 `zero_compensation()` 完成后再开启，避免舵机动作干扰零位捕获。

## 6. 调度器说明

当前调度器位于 `code/app/scheduler.c`，属于主循环轮询调度。

当前任务表：

- `key_scan()`：10ms
- `key_task()`：10ms
- `ips_app_task()`：200ms
- `imu_test()`：10ms

说明：

- `IMU` 采样由 5ms 中断触发
- `IMU` 解算不进入调度表，而是在主循环中优先处理
- `IPS` 刷新采用局部更新，避免整屏闪烁

### 6.1 测试统计

当前工程额外保留了一组仅用于调试观察的测试统计代码，不参与业务控制逻辑：

- `g_scheduler_stat`
  用于查看整个 `scheduler_run()` 的耗时统计
- `g_scheduler_task_diag[]`
  用于查看各个调度任务的单次耗时和最大耗时
- `g_imu_diag_stat`
  用于查看 IMU 采样次数、主循环处理次数和丢帧次数

说明：

- 这些统计变量仅用于调试验证主循环负载和 IMU 丢帧情况
- 代码中已使用 `TEST ONLY` 注释块明确标识，后续可整体移除

## 7. 编译与调试

### 7.1 工具链

根据工程配置，当前工程使用：

- AURIX Development Studio / Eclipse CDT 工程结构
- TASKING 编译工具链
- Debug / Release 两套构建配置

### 7.2 目标配置

从 `.cproject` 可见：

- Target CPU：`tc26xb`
- Package：`bga292`

### 7.3 调试入口

工程中包含共享启动配置文件：

- `LIUHE_bicycle1 Debug.launch`

当前停止入口配置为：

- `core0_main`

## 8. 目录结构

```text
LIUHE_bicycle1/
├─ code/
│  ├─ app/
│  │  ├─ imu_app.c
│  │  ├─ ips_app.c
│  │  ├─ key_app.c
│  │  ├─ led_app.c
│  │  ├─ schedule.c
│  │  └─ ...
│  └─ driver/
│     ├─ imu/
│     │  ├─ mcu_dmp.c
│     │  └─ ...
│     ├─ ips/
│     ├─ key/
│     └─ led/
├─ libraries/
│  ├─ infineon_libraries/
│  ├─ zf_common/
│  ├─ zf_driver/
│  ├─ zf_device/
│  └─ zf_components/
├─ user/
│  ├─ cpu0_main.c
│  ├─ cpu1_main.c
│  ├─ isr.c
│  └─ isr_config.h
├─ 推荐IO分配.txt
└─ README.md
```

## 9. 当前已知现状

- 当前工程主业务集中在 `CPU0`
- `CPU1` 入口已保留，但没有实际应用逻辑
- `yaw` 未引入磁力计绝对参考
- 上电实时零偏校准耗时较长
- IMU 采样位于 PIT ISR 中，主处理位于主循环

## 10. 后续建议

如果后续继续扩展本工程，建议优先考虑以下方向：

- 将 IMU 中断中的重计算与采样解耦
- 将校准流程改为可配置模式
- 为项目增加统一参数配置区
- 将关键模块的接口和数据流整理成独立设计文档

## 11. 相关文件

- IMU 应用层：`code/app/imu_app.c`
- 姿态算法：`code/driver/imu/mcu_dmp.c`
- 屏幕页面配置：`code/app/ips_app_config.h`
- 按键业务：`code/app/key_app.c`
- 调度器：`code/app/scheduler.c`
- 主入口：`user/cpu0_main.c`
- 中断入口：`user/isr.c`
#   L I U H E _ b i c y c l e  
 