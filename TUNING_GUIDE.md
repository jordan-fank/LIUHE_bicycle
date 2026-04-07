# LIUHE_bicycle1 调参文档

> 适用工程：`LIUHE_bicycle1`  
> 平台：Infineon AURIX TC264D  
> 版本基线：2026-04-06 当前工程代码

---

## 1. 文档目标

这份文档不是简单的“变量清单”，而是面向实车调试的完整调参说明，目标是回答四类问题：

1. 这个变量是干什么的。
2. 它改大或改小会发生什么。
3. 应该按什么顺序调。
4. 看到什么现象，应该优先改哪个参数。

本文覆盖两类参数：

- 运行时参数：可以通过 IPS 页面、无线调参、Flash 持久化动态修改。
- 编译期参数：需要改代码重新编译，属于结构参数、硬件参数、模型参数或比赛策略参数。

---

## 2. 总体调参原则

### 2.1 先后顺序

这个项目必须按下面顺序调，不能反过来：

1. IMU 安装方向、陀螺零偏、机械零位。
2. 舵机中值、左右极限、转向方向。
3. 电机方向、速度反馈、速度环。
4. 舵机平衡控制模式与 PID。
5. 导航增益。
6. 科目 1/2/3 的比赛策略参数。
7. 如果使用 LQR，再调物理模型与增益表。

### 2.2 一个阶段只动少量参数

不要同时改很多参数。建议：

- 同一轮只动 1 到 2 个参数。
- 每改一次都记录默认值、修改值、实车现象。
- 能在 IPS 上调的参数，优先用 IPS 调；稳定后再 `KEY1` 双击保存到 Flash。

### 2.3 先把“方向”调对，再谈“增益”

以下内容如果没先调对，后面的 PID、导航、科目参数都会变成假问题：

- IMU 坐标轴映射是否正确。
- `roll_ctrl_angle` 正负号是否符合车辆左右倾斜。
- 舵机打舵方向是否正确。
- 电机正方向是否和车体前进方向一致。
- GPS/IMU 导航误差正负号是否与实际转向一致。

---

## 3. 调参总表

下面先给出项目中所有值得调、需要调、或赛前必须确认的参数总表。

| 类别 | 参数/配置 | 默认值 | 调整方式 | 作用 |
|------|-----------|--------|----------|------|
| IMU | `imu_calibrate_gyro()` / `imu_calibrate_gyro_temp()` | 临时写死值 | 代码/流程 | 陀螺零偏 |
| IMU | `imu_capture_control_zero()` / `imu_set_control_zero()` | 当前姿态/手动值 | 代码/流程 | 机械零位补偿 |
| 舵机硬件 | `CURRENT_SERVO_TYPE` | `SERVO_TYPE_BDS300` | 编译期 | 选择舵机模型与 PWM 频率 |
| 舵机硬件 | `g_servo_left_limit` | 2186 或 250 | IPS/Flash | 左机械极限 |
| 舵机硬件 | `g_servo_mid_duty` | 3686 或 750 | IPS/Flash | 舵机中值 |
| 舵机硬件 | `g_servo_right_limit` | 5186 或 1250 | IPS/Flash | 右机械极限 |
| 舵机方向 | `g_servo_steer_dir` | `SERVO_STEER_DIR_REVERSED` | 代码/Flash | 整体打舵方向 |
| PID 模式 | `g_servo_control_mode` | `SIMPLE_PD` | IPS/无线/Flash | 选择 SIMPLE_PD / CASCADE / LOW_SPEED |
| PID 外环 | `g_balance_kp` | 20.0 | IPS/无线/Flash | 横滚角比例 |
| PID 外环 | `g_balance_ki` | 0.25 | IPS/无线/Flash | 横滚角积分 |
| PID D 项 | `g_balance_kd` | 0.05 | IPS/无线/Flash | 陀螺微分项，仅 SIMPLE_PD |
| 串级内环 | `g_balance_inner_kp` | 1.0 | IPS/无线/Flash | 角速度环比例 |
| 串级内环 | `g_balance_inner_ki` | 0.0 | IPS/无线/Flash | 角速度环积分 |
| 输出约束 | `g_balance_output_limit` | 250.0 | IPS/Flash | 舵机控制限幅 |
| 积分约束 | `g_balance_integral_limit` | 200.0 | IPS/Flash | 外环/内环积分限幅 |
| 电机方向 | `MOTOR_FORWARD_SIGN` | `-1.0f` | 编译期 | 统一车体前进方向 |
| 车轮参数 | `MOTOR_WHEEL_DIAMETER_M` | 0.064 | 编译期 | RPM 到 m/s 换算 |
| 车轮参数 | `MOTOR_GEAR_RATIO` | 1.0 | 编译期 | 速度换算 |
| 电机 PID | `g_motor_kp` | 1.5 | IPS/Flash | 速度环比例 |
| 电机 PID | `g_motor_ki` | 0.4 | IPS/Flash | 速度环积分 |
| 电机 PID | `g_motor_kd` | 0.8 | IPS/Flash | 速度环微分 |
| 电机限幅 | `g_motor_output_limit` | 5000.0 | IPS/Flash | 速度环输出限幅 |
| 电机目标 | `target_motor_rpm` | 1000.0 | IPS/无线/Flash | 当前目标转速 |
| 速度辅助 | `speed_assist.base_speed` | 无默认初始化调用 | 代码 | 倾斜时加速辅助 |
| 速度辅助 | `speed_assist.assist_gain` | 无默认初始化调用 | 代码 | 每度倾角对应速度增益 |
| 速度辅助 | `speed_assist.max_boost` | 无默认初始化调用 | 代码 | 最大附加速度 |
| 导航 | `g_nav_heading_gain` | 0.3 | IPS/无线/Flash | 航向误差转期望倾角 |
| 导航 | `NAV_GPS_SPEED_MIN_KMH` | 2.0 | 编译期 | GPS 航向可信切换阈值 |
| 导航 | `NAV_ACCEPT_RADIUS_DEFAULT_M` | 2.0 | 编译期 | 默认路点到达半径 |
| 科目1 | `g_s1_high_rpm` | 600.0 | IPS/Flash | 去程/回程最高速度 |
| 科目1 | `g_s1_mid_rpm` | 400.0 | IPS/Flash | 进弯/出弯/终点过渡速度 |
| 科目1 | `g_s1_turn_rpm` | 280.0 | IPS/Flash | 掉头速度 |
| 科目1 | `g_s1_pre_brake_dist` | 6.0 | IPS/Flash | 预减速距离 |
| 科目1 | `g_s1_finish_brake_dist` | 4.0 | IPS/Flash | 终点制动距离 |
| 科目1 | `g_s1_resume_thresh` | 25.0 | IPS/Flash | 出弯恢复加速角度阈值 |
| 科目1 | `g_s1_accel_step` | 50.0 | IPS/Flash | 回程渐进加速步进 |
| 科目1 | `SUBJ1_WP_RADIUS` | 2.5 | 编译期 | 路点到达半径 |
| 科目2 | `g_s2_rpm` | 200.0 | IPS/Flash | 绕八字基础速度 |
| 科目2 | `g_s2_turn_radius` | 0.8 | IPS/Flash | 八字半径 |
| 科目2 | `SUBJ2_APPROACH_DIST_M` | 0.5 | 编译期 | 入八字前直线距离 |
| 科目2 | `SUBJ2_RETURN_DIST_M` | 0.5 | 编译期 | 出八字后回程距离 |
| 科目3 | `g_s3_go_rpm` | 500.0 | IPS/Flash | 去程/回程常速 |
| 科目3 | `g_s3_mid_rpm` | 350.0 | IPS/Flash | 进弯/出弯中速 |
| 科目3 | `g_s3_turn_rpm` | 250.0 | IPS/Flash | 掉头速度 |
| 科目3 | `g_s3_pre_brake_dist` | 5.0 | IPS/Flash | 掉头前减速距离 |
| 科目3 | `g_s3_finish_brake_dist` | 4.0 | IPS/Flash | 终点制动距离 |
| 科目3 | `g_s3_resume_thresh` | 30.0 | IPS/Flash | 出弯恢复加速阈值 |
| 科目3 | `g_s3_accel_step` | 40.0 | IPS/Flash | 回程渐进加速步进 |
| 科目3 | `SUBJ3_WP_RADIUS` | 2.0 | 编译期 | 路点到达半径 |
| LQR 方案选择 | `BALANCE_CONTROL_MODE` | `PID` | 编译期 | 选择 PID 还是 LQR |
| LQR 物理参数 | `LQR_DEFAULT_COM_HEIGHT_M` | 0.12 | 编译期 | 质心高度 |
| LQR 物理参数 | `LQR_DEFAULT_WHEELBASE_M` | 0.21 | 编译期 | 轴距 |
| LQR 物理参数 | `LQR_DEFAULT_TRAIL_M` | 0.02 | 编译期 | 拖曳距 |
| LQR 限幅 | `LQR_DELTA_MECHANICAL_MAX` | 23° | 编译期 | 最大机械转向角 |
| LQR 限幅 | `LQR_DEFAULT_DELTA_DOT_MAX_RAD_S` | 8.0 | 编译期 | 最大转向角速度 |
| LQR 低速阈值 | `LQR_DEFAULT_V_MIN_M_S` | 0.3 | 编译期 | 低速保护阈值 |
| LQR 增益 | `default_gain_entries[]` | 15 点表 | 编译期 | 速度自适应增益表 |

---

## 4. 实车调参总流程

推荐按下面流程执行：

### 阶段 A：静态基础校准

1. 确认 IMU 安装方向和代码映射一致。
2. 完成陀螺零偏校准。
3. 捕获机械零位。
4. 调准舵机中值和左右机械极限。
5. 确认“右倾时是否打同侧舵”。

### 阶段 B：基础闭环验证

1. 在 `SIMPLE_PD` 模式下做手持倾斜试验。
2. 确认舵机动作方向正确。
3. 电机单独测速，确认 `target_motor_rpm`、`motor_speed_rpm`、`g_motor_pid_output` 三者关系。

### 阶段 C：平衡控制调通

1. 先调舵机外环。
2. 再调电机速度环。
3. 再调导航增益。
4. 最后调科目策略参数。

### 阶段 D：按科目分别细调

1. 科目 1：先调掉头，再调回程，再拉高速。
2. 科目 2：先保证低速不倒，再收半径，再压时间。
3. 科目 3：先保证通过性，再考虑速度。

---

## 5. IMU 与姿态层调参

这一层不是“性能优化”，而是整个系统的基础。如果 IMU 层没调对，后面所有参数都会表现异常。

### 5.1 IMU 安装方向与坐标映射

当前工程在 [imu_app.c](E:\AURIX-v1.10.28-workspace\LIUHE_bicycle1\code\app\imu_app.c) 中假定：

- 芯片安装：`X 向左，Y 向后，Z 向上`
- 映射到车体坐标：
  - `body_x = -sensor_y`
  - `body_y =  sensor_x`
  - `body_z =  sensor_z`

如果你以后重新装 IMU，这部分必须重新确认。表现错误时常见现象：

- 左右倾斜时，`roll_kalman` 几乎不变。
- 俯仰和横滚混在一起。
- 舵机看起来“方向调不对”，但其实是 IMU 轴错了。

### 5.2 陀螺零偏

相关接口：

- `imu_calibrate_gyro()`
- `imu_calibrate_gyro_temp()`

原理：

- 陀螺仪静止时理论角速度应为 0。
- 实际有零偏，若不扣除，`yaw/roll/pitch` 会持续漂移。

当前项目现状：

- 默认用 `imu_calibrate_gyro_temp()` 直接写死历史零偏值，适合快速联调。
- 比赛前如果温度变化大，建议重新静态标定一次。

调参建议：

- 日常联调：先用 `imu_calibrate_gyro_temp()`，节省时间。
- 关键验证、正式赛前：用 `imu_calibrate_gyro()` 实测零偏。

现象判断：

- 静止时 `gyro_x_rate/gyro_y_rate` 长期不接近 0，说明零偏不准。
- 静止不动时 `yaw_kalman` 仍快速漂移，优先怀疑陀螺零偏。

### 5.3 机械零位补偿

相关接口：

- `imu_capture_control_zero()`
- `imu_set_control_zero(roll_zero_deg, pitch_zero_deg)`

相关变量：

- `roll_kalman`
- `pitch_kalman`
- `roll_ctrl_angle`
- `pitch_ctrl_angle`

原理：

- `roll_kalman/pitch_kalman` 是 IMU 原始姿态。
- 车辆装配后，实车“看起来摆正”时 IMU 原始值通常不等于 0。
- 控制器真正应该看的，是扣掉机械装配偏差后的 `roll_ctrl_angle`。

推荐做法：

1. 车静止摆正。
2. 等姿态稳定。
3. 执行 `imu_capture_control_zero()`。
4. 后续所有平衡控制都使用 `roll_ctrl_angle`。

目标状态：

- 车“机械摆正”时，`roll_ctrl_angle` 应接近 0。

如果不对会怎样：

- 明明车是正的，舵机会持续往一侧修正。
- PID 永远带着固定静差。
- 高速时车总往一边偏。

---

## 6. 舵机硬件层调参

### 6.1 舵机型号选择

相关配置：

- `CURRENT_SERVO_TYPE`

位置：

- [servo_app.h](E:\AURIX-v1.10.28-workspace\LIUHE_bicycle1\code\app\servo_app.h)

当前默认：

- `SERVO_TYPE_BDS300`

意义：

- 决定 PWM 频率、舵机脉宽范围、初始化等待时间。

建议：

- 调试阶段可以用 MG996R。
- 比赛阶段若用 BDS300，就必须按 BDS300 的频率和机械极限重新确认。

### 6.2 舵机中值与左右极限

相关变量：

- `g_servo_left_limit`
- `g_servo_mid_duty`
- `g_servo_right_limit`

调参原则：

- `mid`：前轮真正摆正时的占空比。
- `left/right`：在不顶死连杆、不撞车架、不硬顶舵机的前提下，可安全达到的极限值。

标准：

#### 中值标准

- 车轮正前方。
- 摇臂和拉杆无明显预紧。
- 左右可用打角余量尽量接近。

#### 左右极限标准

- 到机械极限前留一点余量。
- 舵机不发闷、不持续顶死。
- 左右极限最好按实车装配分别测，不要假设完全对称。

调法：

1. 先只调 `g_servo_mid_duty`，让前轮摆正。
2. 再慢慢调 `g_servo_left_limit`、`g_servo_right_limit`。
3. 确保 `servo_set()` 限幅后不会撞机械结构。

常见误区：

- 把电气极限当成机械极限。
- 只看舵机规格书，不看车架连杆的真实限位。
- 中值大致对了就不再精调，后续所有导航都会带偏。

### 6.3 舵机方向

相关变量：

- `g_servo_steer_dir`

意义：

- 只在最终输出层整体翻转转向方向。
- 不改 IMU 正负号，不改导航误差定义，不改 PID 误差定义。

判断方法：

- 手扶车身向右倾，如果你当前采用的是“同侧打舵平衡”策略，那么前轮应向右打。
- 如果实际向左，就翻转 `g_servo_steer_dir`。

注意：

- 如果 IMU 坐标轴本身就错了，不要靠 `g_servo_steer_dir` 去硬救。

---

## 7. 平衡控制调参

### 7.1 控制模式 `g_servo_control_mode`

取值：

- `0`：`SIMPLE_PD`
- `1`：`CASCADE`
- `2`：`LOW_SPEED`

推荐用途：

- `SIMPLE_PD`：台架联调、最初验证。
- `CASCADE`：科目 1、科目 3 推荐。
- `LOW_SPEED`：科目 2 推荐。

### 7.2 SIMPLE_PD 模式

相关变量：

- `g_balance_kp`
- `g_balance_ki`
- `g_balance_kd`
- `g_balance_output_limit`
- `g_balance_integral_limit`

原理：

- `P` 决定“倾斜了要多快打舵”。
- `I` 用来消除长时间小偏差。
- `D` 直接用陀螺角速度，抑制动态摆动。

调参方法：

#### 第一步：只调 `Kp`

- 暂时让 `Ki` 很小或接近 0。
- `Kd` 先保守。
- 逐步增大 `Kp`，直到车对倾斜有足够纠正能力。

`Kp` 太小：

- 车反应慢。
- 倾了不积极救。
- 容易慢慢倒。

`Kp` 太大：

- 舵机过于神经质。
- 左右高频摆动。
- 可能快速打满。

#### 第二步：补 `Kd`

目标：

- 压制晃动和过冲。

`Kd` 太小：

- 来回摆。
- 修正过头。

`Kd` 太大：

- 对陀螺噪声敏感。
- 舵机抖、细碎震荡。

#### 第三步：少量加 `Ki`

目标：

- 消除小静差。

`Ki` 太小：

- 长时间偏一点。

`Ki` 太大：

- 累积过多，突然猛打一把。
- 容易慢性振荡。

### 7.3 CASCADE 模式

相关变量：

- 外环：`g_balance_kp`、`g_balance_ki`
- 内环：`g_balance_inner_kp`、`g_balance_inner_ki`
- 约束：`g_balance_output_limit`、`g_balance_integral_limit`

原理：

- 外环根据角度误差算目标角速度。
- 内环负责跟踪这个目标角速度。

优点：

- 动态更清楚。
- 中高速更稳。
- 更适合科目 1、科目 3。

调参顺序必须是：

1. 外环 `Kp`
2. 内环 `inner_kp`
3. 外环 `Ki`
4. 最后才考虑 `inner_ki`

#### 外环 `g_balance_kp`

- 决定姿态偏差被转成多大的目标角速度。

太小：

- 车懒。
- 大角度也不积极救。

太大：

- 外环给内环下发过猛的角速度目标。
- 车容易左右抽动。

#### 内环 `g_balance_inner_kp`

- 决定角速度跟踪有多强。

太小：

- 外环说要转快一点，内环跟不上。
- 整体反应拖沓。

太大：

- 舵机动作生硬、冲。
- 容易噪声敏感。

#### 外环 `g_balance_ki`

- 消除角度静差。

原则：

- 只加一点点。
- 太大时会把整车往一边持续推。

#### 内环 `g_balance_inner_ki`

- 只有在明确存在角速度长期静差时再加。
- 大部分时候先保持很小甚至 0。

### 7.4 LOW_SPEED 模式

相关变量：

- `g_balance_kp`
- `g_balance_ki`
- `g_balance_output_limit`
- `g_balance_integral_limit`

原理：

- 去掉 `D` 项，只保留角度 PI。
- 低速时陀螺噪声影响更大，所以这样更稳。

适用：

- 科目 2 低速绕八字。

调法：

- `Kp` 不能太低，否则低速根本扶不住。
- `Ki` 也不能太大，否则小半径绕圈时会慢慢积累偏差。

典型现象：

- 低速轻微摆动：先加一点 `Kp`。
- 低速大幅左右抽：先降 `Kp` 或减小导航增益。
- 绕圈越久越歪：减小 `Ki`。

### 7.5 输出限幅与积分限幅

#### `g_balance_output_limit`

作用：

- 限制舵机控制输出最大幅度。

太小：

- 车需要大角度救车时打不出来。

太大：

- 一旦参数过猛就直接打到接近机械极限，车更容易翻。

#### `g_balance_integral_limit`

作用：

- 防止积分累太多。

太小：

- 无法消除静差。

太大：

- 会出现“前面都正常，过一会突然猛打一把”的现象。

---

## 8. 电机速度环调参

### 8.1 方向与速度换算参数

编译期配置：

- `MOTOR_FORWARD_SIGN`
- `MOTOR_WHEEL_DIAMETER_M`
- `MOTOR_GEAR_RATIO`

这些参数不对，速度闭环和导航里程都会错。

#### `MOTOR_FORWARD_SIGN`

作用：

- 统一“车辆前进为正”。

判断方法：

- 车向前滚动时，`motor_speed_rpm` 应为正。
- 如果为负，就要改方向定义。

#### `MOTOR_WHEEL_DIAMETER_M`

作用：

- 影响 RPM 到 m/s 的换算。

后果：

- 这个参数错，`motor_speed_m_s` 错。
- 科目 2 里程切段、LQR 速度调度、导航距离估计都会跟着错。

### 8.2 电机 PID

相关变量：

- `g_motor_kp`
- `g_motor_ki`
- `g_motor_kd`
- `g_motor_output_limit`
- `target_motor_rpm`

原理：

- 电机 PID 负责让实际速度追上目标速度。

#### `g_motor_kp`

作用：

- 主导速度误差的即时修正。

太小：

- 速度跟不上目标。
- 加速拖沓。

太大：

- 输出容易冲。
- 速度上下波动。

#### `g_motor_ki`

作用：

- 消除稳态速度偏差。

太小：

- 长时间达不到目标速度。

太大：

- 容易饱和。
- 慢性振荡明显。

#### `g_motor_kd`

作用：

- 抑制速度变化过冲。

太小：

- 加速/减速过冲明显。

太大：

- 速度信号噪声会被放大。
- 输出发抖。

#### `g_motor_output_limit`

作用：

- 限制 PID 输出给电机的最大占空比。

太小：

- 电机永远冲不上去。

太大：

- 一旦误差大，输出容易顶满，系统更难看出真实 PID 品质。

### 8.3 调法

推荐步骤：

1. 先确保编码器/速度反馈正确。
2. `Ki`、`Kd` 先保守，先调 `Kp`。
3. 速度能基本跟上后，再加 `Ki`。
4. 如果加减速波动明显，再微调 `Kd`。

观察量：

- `target_motor_rpm`
- `motor_speed_rpm`
- `g_motor_pid_output`

判断规则：

- `target_motor_rpm` 很高，`motor_speed_rpm` 一直 0：先查反馈链路，不是 PID。
- `g_motor_pid_output` 长时间顶 `5000`：先查反馈丢失、速度方向、驱动模式，不要盲调 PID。

### 8.4 速度辅助平衡

相关结构体：

- `speed_assist.base_speed`
- `speed_assist.assist_gain`
- `speed_assist.max_boost`
- `speed_assist.enabled`

目前工程中这套功能已保留接口，但默认流程里没有作为主链路使用。

建议：

- 在基础闭环未稳定前，不要启用。
- 如果以后要用于动态扶车，应最后再调。

---

## 9. 导航层调参

### 9.1 `g_nav_heading_gain`

这是整个导航层最关键的运行时参数。

原理：

- 导航只输出“航向误差”。
- 真正进入控制器的是：

```c
expect_angle = g_nav_heading_error * g_nav_heading_gain;
```

也就是说，它决定“偏航误差要转成多大的目标侧倾角”。

太小：

- 车知道自己偏了，但懒得修。
- 导航轨迹发散。

太大：

- 车修得太猛。
- 出现来回切线、蛇形。
- 科目 2 容易内外摆。

经验建议：

- 科目 1：中等。
- 科目 2：通常比科目 1 更大一点。
- 科目 3：要兼顾障碍路面，不能只追求灵敏。

### 9.2 `NAV_GPS_SPEED_MIN_KMH`

这是编译期参数。

作用：

- 低于这个速度时，GPS 航向不可靠，系统转而用 IMU 变化量补偿。

太低：

- 低速时仍然信 GPS 航向，容易抖。

太高：

- 切换过早，IMU 漂移影响更大。

这个值一般不是高频调参项，但如果你发现低速掉头时航向乱跳，可以检查它。

### 9.3 `NAV_ACCEPT_RADIUS_DEFAULT_M`

作用：

- 一般路点默认到达半径。

太小：

- 迟迟不到点。
- 容易在点附近来回磨。

太大：

- 提前判到点。
- 轨迹切角严重。

---

## 10. 科目 1 调参

相关运行时参数：

- `g_s1_high_rpm`
- `g_s1_mid_rpm`
- `g_s1_turn_rpm`
- `g_s1_pre_brake_dist`
- `g_s1_finish_brake_dist`
- `g_s1_resume_thresh`
- `g_s1_accel_step`

相关编译期参数：

- `SUBJ1_WP_RADIUS`

### 10.1 参数作用与调法

#### `g_s1_high_rpm`

作用：

- 去程和回程的最高目标转速。

调法：

- 永远最后再往上加。

太小：

- 稳，但慢。

太大：

- 直线本身可能还行，但掉头前后的失稳风险会急剧增加。

#### `g_s1_mid_rpm`

作用：

- 预减速进入掉头区时的过渡速度。
- 掉头结束、回程初段速度。
- 终点制动区速度。

太小：

- 稳，但会很肉。

太大：

- 进弯仍然冲。
- 出弯刚恢复时容易翻。

#### `g_s1_turn_rpm`

作用：

- 掉头阶段恒定低速。

这是科目 1 最先要调的速度参数之一。

太小：

- 掉头很稳，但慢。
- 有时速度过低又不利于平衡。

太大：

- 掉头半径大。
- 容易冲弯。

#### `g_s1_pre_brake_dist`

作用：

- 离掉头点多远开始线性减速。

太小：

- 来不及收速。
- 最容易冲过掉头点。

太大：

- 提前磨蹭，浪费时间。

#### `g_s1_finish_brake_dist`

作用：

- 回程终点前的收速区。

太小：

- 冲过起点。

太大：

- 回程最后一段变慢，浪费时间。

#### `g_s1_resume_thresh`

作用：

- 掉头后航向误差小到什么程度，才允许恢复加速。

太小：

- 太保守。
- 在掉头区附近磨很久。

太大：

- 尚未对准就开始拉速。
- 最容易出弯翻车。

#### `g_s1_accel_step`

作用：

- 回程每 100ms 增加多少转速。

太小：

- 很稳，但回程起不来。

太大：

- 出弯后瞬间又接近“暴力给速”，很容易翻。

#### `SUBJ1_WP_RADIUS`

作用：

- 掉头点与回程起点的到达判定半径。

太小：

- 点位判定太苛刻，容易在点附近磨。

太大：

- 提前开始切状态，掉头形状变差。

### 10.2 推荐调参顺序

1. 先调 `g_s1_turn_rpm`
2. 再调 `g_s1_pre_brake_dist`
3. 再调 `g_s1_resume_thresh`
4. 再调 `g_s1_accel_step`
5. 再调 `g_s1_mid_rpm`
6. 最后调 `g_s1_high_rpm`
7. 收尾修 `g_s1_finish_brake_dist`

---

## 11. 科目 2 调参

相关运行时参数：

- `g_s2_rpm`
- `g_s2_turn_radius`

相关编译期参数：

- `SUBJ2_APPROACH_DIST_M`
- `SUBJ2_RETURN_DIST_M`

### 11.1 `g_s2_rpm`

作用：

- 八字全程基础速度。

这不是越低越好。

太低：

- 虽然理论上更容易拿“慢速成绩”，但车可能根本扶不住。
- IMU 漂移影响时间变长。

太高：

- 八字半径容易变大。
- 低速稳定优势消失。

### 11.2 `g_s2_turn_radius`

作用：

- 决定八字轨迹圆弧半径。

太小：

- 转向太急。
- 对平衡和导航增益要求更高。

太大：

- 八字出圈。
- 场地不够。

### 11.3 `SUBJ2_APPROACH_DIST_M`

作用：

- 发车区进入八字交叉点前的直线段长度。

如果实车总是在入弯前位置不对，可以改这个值。

### 11.4 `SUBJ2_RETURN_DIST_M`

作用：

- 完成八字后回发车区的直线段长度。

如果最后总停不在合适位置，可以改这个值。

### 11.5 科目 2 核心调参逻辑

科目 2 真正关键的不只是 `g_s2_rpm` 和 `g_s2_turn_radius`，还包括：

- `g_servo_control_mode` 应为 `LOW_SPEED`
- `g_balance_kp`
- `g_balance_ki`
- `g_nav_heading_gain`

推荐顺序：

1. 先用 `LOW_SPEED` 模式把低速平衡调稳。
2. 再调 `g_nav_heading_gain`，让八字轨迹能收住。
3. 再调 `g_s2_turn_radius`，让路径落进圈内。
4. 最后微调 `g_s2_rpm`，在“不倒”的前提下尽量慢。

---

## 12. 科目 3 调参

相关运行时参数：

- `g_s3_go_rpm`
- `g_s3_mid_rpm`
- `g_s3_turn_rpm`
- `g_s3_pre_brake_dist`
- `g_s3_finish_brake_dist`
- `g_s3_resume_thresh`
- `g_s3_accel_step`

相关编译期参数：

- `SUBJ3_WP_RADIUS`

### 12.1 和科目 1 的本质区别

科目 3 调参目标不是“最快掉头”，而是“保证通过障碍路段且回程还能复现路线”。

因此科目 3 的调参优先级应是：

1. 通过性
2. 路径精度
3. 掉头稳定
4. 速度

### 12.2 各参数解释

#### `g_s3_go_rpm`

- 多路点去程/回程常速。
- 太高会让草地、坡道、锥桶之间通过性下降。

#### `g_s3_mid_rpm`

- 用于进弯、出弯和终点收速。
- 太高则地形路面上更容易抖和甩。

#### `g_s3_turn_rpm`

- 掉头阶段速度。
- 科目 3 的掉头通常比科目 1 更保守。

#### `g_s3_pre_brake_dist`

- 接近最后掉头点时的预减速距离。
- 太小仍会冲过掉头区。

#### `g_s3_finish_brake_dist`

- 回到起点前的收速区。

#### `g_s3_resume_thresh`

- 掉头后何时恢复回程加速。
- 地形复杂时通常要比科目 1 更保守。

#### `g_s3_accel_step`

- 回程逐步加速步进。
- 地形赛不宜太激进。

#### `SUBJ3_WP_RADIUS`

- 科目 3 多路点的到达判定半径。

太小：

- 在障碍附近磨蹭。

太大：

- 轨迹不贴障碍间隙。

### 12.3 推荐调法

1. 先低速完成整套路点流程。
2. 确认每个关键障碍前后的路点设置合理。
3. 再调 `SUBJ3_WP_RADIUS` 和 `g_nav_heading_gain`。
4. 最后才逐步提高 `g_s3_go_rpm`。

---

## 13. LQR 调参

如果你使用 PID 方案为主，这一节可作为高级参考。

### 13.1 方案开关

相关配置：

- `BALANCE_CONTROL_MODE`

位置：

- [balance_control_mode.h](E:\AURIX-v1.10.28-workspace\LIUHE_bicycle1\code\app\balance_control_mode.h)

取值：

- `BALANCE_CONTROL_MODE_PID`
- `BALANCE_CONTROL_MODE_LQR`

### 13.2 LQR 调参不是调一个 `Kp`

LQR 调参实际包括三层：

1. 物理模型参数
2. 限幅参数
3. 离线生成的增益表

### 13.3 物理模型参数

相关编译期参数：

- `LQR_DEFAULT_COM_HEIGHT_M`
- `LQR_DEFAULT_WHEELBASE_M`
- `LQR_DEFAULT_TRAIL_M`
- `LQR_DEFAULT_GRAVITY_M_S2`

意义：

- `h`：质心高度
- `L`：轴距
- `b`：拖曳距

这些参数错了会怎样：

- 增益表和实车不匹配。
- 某些速度点很好，另一些速度点很差。

### 13.4 机械与控制限幅

相关参数：

- `LQR_DELTA_MECHANICAL_MAX`
- `LQR_DEFAULT_DELTA_DOT_MAX_RAD_S`
- `LQR_DEFAULT_V_MIN_M_S`

#### `LQR_DELTA_MECHANICAL_MAX`

- 最大机械转向角。
- 必须与真实舵机/车架极限一致。

#### `LQR_DEFAULT_DELTA_DOT_MAX_RAD_S`

- 最大转向角速度。
- 太小：反应慢。
- 太大：容易冲。

#### `LQR_DEFAULT_V_MIN_M_S`

- 低速保护阈值。
- 低于这个速度时，LQR 会倾向缓慢回中。

### 13.5 增益表 `default_gain_entries[]`

这是真正决定 LQR 品质的核心。

要点：

- 它不是固定真理，只是当前实车模型的默认表。
- 换车架、换质心、换轴距、换速度区间，都应该重算。

正确做法：

1. 测量真实 `h`、`L`、`b`
2. 用 [lqr_gain_generator.py](E:\AURIX-v1.10.28-workspace\LIUHE_bicycle1\code\tools\lqr_gain_generator.py) 重算表
3. 替换 [lqr_driver.c](E:\AURIX-v1.10.28-workspace\LIUHE_bicycle1\code\driver\lqr\lqr_driver.c) 中的 `default_gain_entries[]`

### 13.6 `lqr_expect_phi`

这个量本身不是你手工主调的基础参数，而是导航层给 LQR 的期望侧倾角参考。

真正和它强相关的是：

- `g_nav_heading_gain`
- `LQR_DELTA_MECHANICAL_MAX`

---

## 14. IPS 与无线调参的角色分工

### 14.1 IPS 适合调什么

IPS 适合现场、车边、低频微调：

- 舵机中值与机械极限
- 科目 1/2/3 参数
- 基础 PID 参数

优点：

- 不依赖上位机。
- 改完可直接 `KEY1` 双击保存。

### 14.2 无线适合调什么

无线适合动态观察和少量核心闭环参数：

- `g_balance_kp`
- `g_balance_ki`
- `g_balance_kd`
- `g_balance_inner_kp`
- `g_balance_inner_ki`
- `g_nav_heading_gain`
- `target_motor_rpm`
- `g_servo_control_mode`

注意：

- 无线当前没有把科目 1/2/3 的运行时参数映射进去，科目参数以 IPS 为主。

---

## 15. 症状 -> 优先检查参数

### 15.1 手持倾斜时舵机方向不对

优先检查：

1. IMU 安装方向映射
2. `roll_ctrl_angle` 正负号
3. `g_servo_steer_dir`

### 15.2 车正放时舵机仍持续偏一边

优先检查：

1. `imu_capture_control_zero()`
2. `g_servo_mid_duty`
3. `g_balance_ki` 是否过大

### 15.3 舵机动作很小，倾很多也不怎么打

优先检查：

1. `g_balance_kp`
2. `g_balance_output_limit`
3. `g_servo_left_limit/right_limit`

### 15.4 电机目标很高，速度一直起不来

优先检查：

1. 电机速度反馈链路
2. `MOTOR_FORWARD_SIGN`
3. `g_motor_output_limit`
4. 再看 `g_motor_kp/ki/kd`

### 15.5 导航时左右蛇形

优先检查：

1. `g_nav_heading_gain` 太大
2. `g_balance_kp` 太大
3. `g_s1_accel_step` 或 `g_s3_accel_step` 太激进

### 15.6 科目 1 掉头总冲过

优先调：

1. `g_s1_pre_brake_dist`
2. `g_s1_turn_rpm`
3. `g_s1_mid_rpm`

### 15.7 科目 1 出弯容易翻

优先调：

1. `g_s1_accel_step`
2. `g_s1_resume_thresh`
3. `g_s1_mid_rpm`
4. `g_nav_heading_gain`

### 15.8 科目 2 八字半径太大

优先调：

1. `g_s2_turn_radius`
2. `g_nav_heading_gain`
3. `g_s2_rpm`

### 15.9 科目 2 低速容易倒

优先调：

1. `g_balance_kp`（LOW_SPEED）
2. `g_s2_rpm`
3. `g_balance_ki`

### 15.10 科目 3 容易撞障碍

优先调：

1. 路点位置本身
2. `SUBJ3_WP_RADIUS`
3. `g_nav_heading_gain`
4. `g_s3_go_rpm`

---

## 16. 推荐的实车调参顺序

### 第一天：底层链路

1. IMU 安装方向确认
2. 陀螺零偏
3. 机械零位
4. 舵机中值与极限
5. 电机速度反馈

### 第二天：PID 平衡

1. `SIMPLE_PD` 手持试验
2. 小速度直行
3. `CASCADE` 模式替换验证
4. `LOW_SPEED` 模式单独验证

### 第三天：导航

1. 单独调 `g_nav_heading_gain`
2. 先跑简单直线导航
3. 再进科目参数调试

### 第四天：科目策略

1. 科目 1 先稳后快
2. 科目 2 先不倒后缩圈
3. 科目 3 先通过后提速

---

## 17. 保存策略

当前项目支持 Flash 持久化。

### 17.1 全局配置 Flash

通过 `KEY1` 双击保存：

- 舵机 PID
- 电机 PID
- 舵机机械限位
- 控制模式
- 导航增益
- 科目 1/2/3 运行时参数
- 科目 1 的 GPS 路点数据

### 17.2 科目 3 专属 Flash

科目 3 路点单独存在 page 10。

所以需要区分：

- 全局参数保存
- 科目 3 专属路点保存

---

## 18. 最终结论

这个工程真正要调的，不是单一某个 `kp`，而是一整条链：

1. 传感器基线是否正确
2. 执行机构方向和尺度是否正确
3. 平衡闭环是否稳定
4. 速度闭环是否可信
5. 导航增益是否匹配
6. 各科目策略参数是否和当前车况、场地、速度等级匹配

如果只盯着 PID，不看机械零位、舵机极限、电机方向、导航增益，调参效率会非常低。  
正确的方法永远是：

- 先把“符号、零位、尺度”调对
- 再把“闭环”调稳
- 最后再去追“比赛成绩”

---

## 19. 相关文件索引

- [servo_pid.c](E:\AURIX-v1.10.28-workspace\LIUHE_bicycle1\code\app\servo_pid.c)
- [servo_app.c](E:\AURIX-v1.10.28-workspace\LIUHE_bicycle1\code\app\servo_app.c)
- [motor_pid.c](E:\AURIX-v1.10.28-workspace\LIUHE_bicycle1\code\app\motor_pid.c)
- [motor_app.c](E:\AURIX-v1.10.28-workspace\LIUHE_bicycle1\code\app\motor_app.c)
- [imu_app.c](E:\AURIX-v1.10.28-workspace\LIUHE_bicycle1\code\app\imu_app.c)
- [nav_app.c](E:\AURIX-v1.10.28-workspace\LIUHE_bicycle1\code\app\nav_app.c)
- [subject1_app.c](E:\AURIX-v1.10.28-workspace\LIUHE_bicycle1\code\app\subject1_app.c)
- [subject2_app.c](E:\AURIX-v1.10.28-workspace\LIUHE_bicycle1\code\app\subject2_app.c)
- [subject3_app.c](E:\AURIX-v1.10.28-workspace\LIUHE_bicycle1\code\app\subject3_app.c)
- [lqr_balance.c](E:\AURIX-v1.10.28-workspace\LIUHE_bicycle1\code\app\lqr_balance.c)
- [lqr_driver.c](E:\AURIX-v1.10.28-workspace\LIUHE_bicycle1\code\driver\lqr\lqr_driver.c)
- [wireless_debug_app.c](E:\AURIX-v1.10.28-workspace\LIUHE_bicycle1\code\app\wireless_debug_app.c)
- [ips_app_config.h](E:\AURIX-v1.10.28-workspace\LIUHE_bicycle1\code\app\ips_app_config.h)
- [config_flash.c](E:\AURIX-v1.10.28-workspace\LIUHE_bicycle1\code\app\config_flash.c)

