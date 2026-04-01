# LQR 在本项目中的接入与使用说明

## 1. 先看结论

这次接入做的是“最小修改版”：

- 没有删除你原来的 PID 方案
- 当前 5ms 中断里，已经支持通过开关在 `balance_control()` 和 `lqr_balance_control()` 之间一键切换
- 原 PID 代码仍完整保留，后续可以直接做 A/B 对比
- LQR 已经改成读取你项目里真正用于左右平衡的 IMU 轴：
  - `roll_ctrl_angle`
  - `gyro_x_rate`

要特别注意：

- 你现在移植进来的 LQR 参数只能算“能接上项目的初值”
- 它不是你的智能单车最终参数
- 如果不做实测和重新整定，效果大概率只能到“有响应”，不一定“能稳定”

---

## 2. 这次我改了什么

本次实际修改点：

- [user/isr.c](/E:/AURIX-v1.10.28-workspace/LIUHE_bicycle1/user/isr.c)
  - 增加基于 `balance_control_mode.h` 的条件编译切换
  - 5ms 中断里可在 `balance_control()` 和 `lqr_balance_control()` 之间一键切换
- [user/cpu0_main.c](/E:/AURIX-v1.10.28-workspace/LIUHE_bicycle1/user/cpu0_main.c)
  - 增加 `lqr_balance_init()`
  - 增加 `lqr_balance_set_enable(0/1)`
  - 用同一套模式开关决定最终启用 PID 还是 LQR
  - 保持你原来“先零位捕获，再放开控制”的启动节奏
- [code/app/lqr_balance.c](/E:/AURIX-v1.10.28-workspace/LIUHE_bicycle1/code/app/lqr_balance.c)
  - 增加 LQR 使能接口
  - 改成读取 `roll_ctrl_angle/gyro_x_rate`
  - 增加 `pwm_angle` 同步，保证 IPS 显示正常
  - 修正速度单位说明，当前已确认 `motor_get_raw_rpm()` 返回 `RPM`
- [code/app/motor_app.c](/E:/AURIX-v1.10.28-workspace/LIUHE_bicycle1/code/app/motor_app.c)
  - 修正电机车速换算逻辑，按 `RPM -> m/s` 计算
  - 新增“车体方向统一符号”的速度接口，不再让上层自己决定要不要乘 `-1`
- [code/app/motor_app.h](/E:/AURIX-v1.10.28-workspace/LIUHE_bicycle1/code/app/motor_app.h)
  - 修正轮驱参数：减速比 `1`、车轮直径 `64mm`
- [code/app/lqr_balance.h](/E:/AURIX-v1.10.28-workspace/LIUHE_bicycle1/code/app/lqr_balance.h)
  - 增加 `lqr_balance_set_enable()` 声明
  - 修正 LQR 默认轮半径为 `0.032m`
- [code/driver/lqr/lqr_driver.h](/E:/AURIX-v1.10.28-workspace/LIUHE_bicycle1/code/driver/lqr/lqr_driver.h)
  - 统一收口 LQR 默认物理参数、默认限幅参数和默认增益表生成条件
- [code/driver/lqr/lqr_driver.c](/E:/AURIX-v1.10.28-workspace/LIUHE_bicycle1/code/driver/lqr/lqr_driver.c)
  - `lqr_init()` 改为直接使用 `lqr_driver.h` 中的默认宏，避免驱动层和应用层各写一套默认值
- [README.md](/E:/AURIX-v1.10.28-workspace/LIUHE_bicycle1/README.md)
  - 增加 LQR 接入现状说明

---

## 3. LQR 到底是什么

LQR 全称是 `Linear Quadratic Regulator`，中文一般叫“线性二次型最优调节器”。

它的核心不是“看到误差就按比例打一点”，而是：

- 先建立一个状态空间模型
- 再定义你最想压制哪些状态误差
- 同时定义你有多讨厌控制量太大
- 最后算出一组最优反馈增益 `K`

控制律形式非常简单：

```text
u = -Kx
```

这里：

- `x` 是系统状态
- `K` 是反馈增益
- `u` 是控制输出

所以 LQR 本质上也是“状态反馈控制”，只是它的增益不是手调 `Kp/Ki/Kd`，而是从模型和权重矩阵里算出来的。

---

## 4. 你这个项目里的 LQR 模型是什么意思

你当前移植进来的 LQR 用的是 3 维状态：

```text
x = [phi, phi_dot, delta]
```

分别表示：

- `phi`：车身侧倾角
- `phi_dot`：车身侧倾角速度
- `delta`：当前转向角

控制输入是：

```text
u = delta_dot
```

也就是：

- LQR 不是直接输出“舵机打到多少角”
- 它先输出“转向角速度”
- 然后在代码里积分成“目标转向角”
- 再把目标转向角映射成舵机 PWM

---

## 5. 你的驱动层和应用层分别干什么

### 5.1 驱动层 `lqr_driver.c/.h`

路径：

- [lqr_driver.h](/E:/AURIX-v1.10.28-workspace/LIUHE_bicycle1/code/driver/lqr/lqr_driver.h)
- [lqr_driver.c](/E:/AURIX-v1.10.28-workspace/LIUHE_bicycle1/code/driver/lqr/lqr_driver.c)

驱动层做的是通用 LQR 计算，不关心你项目里的 IMU、舵机、电机接口。

它负责：

- 保存 LQR 控制器结构体
- 保存物理参数
- 保存增益表
- 根据速度查表 / 插值出当前 `K`
- 根据状态计算 `u = -Kx`
- 对 `delta_dot`、`delta_cmd` 做限幅

### 5.2 应用层 `lqr_balance.c/.h`

路径：

- [lqr_balance.h](/E:/AURIX-v1.10.28-workspace/LIUHE_bicycle1/code/app/lqr_balance.h)
- [lqr_balance.c](/E:/AURIX-v1.10.28-workspace/LIUHE_bicycle1/code/app/lqr_balance.c)

应用层负责把你项目里的真实接口接到 LQR 算法上。

它负责：

- 从 IMU 读侧倾角和角速度
- 从电机接口估算速度
- 调 `lqr_update_gain()` 选当前速度对应的增益
- 调 `lqr_compute()` 算控制输出
- 把 `delta_cmd` 转成舵机 PWM
- 调 `servo_set()` 真正输出

---

## 6. 现在这套 LQR 在项目里怎么跑

当前执行链路是：

1. `imu_sample_isr()` 在 5ms PIT 里采样 IMU 原始数据
2. 主循环里的 `imu_proc()` 解算出：
   - `roll_ctrl_angle`
   - `gyro_x_rate`
3. `cc60_pit_ch1_isr()` 每 5ms 按 `balance_control_mode.h` 选择调用 `balance_control()` 或 `lqr_balance_control()`
4. 当模式切到 `LQR` 时，`lqr_balance_control()` 里做这些事：
   - 通过电机层接口读取速度大小
   - 根据速度查表更新 `K`
   - 读取当前侧倾角和角速度
   - 算出目标转向角
   - 转成舵机 PWM
   - 调 `servo_set()`

如果模式切到 `LQR`，对应入口是：

```c
lqr_balance_control();
```

如果模式切到 `PID`，对应入口是：

```c
balance_control();
```

两套入口都保留，没有删除。

补充一条已经确认的事实：

- 你这个项目里的 `motor_get_raw_rpm()` 返回的是电机转速 `RPM`
- 这个 RPM 本身带正负号，符号来自 FOC 驱动串口反馈
- 本车是轮驱一体，所以减速比按 `1` 处理
- 当前代码已经按车轮直径 `64mm` 修正了默认速度换算
- 当前应用层又增加了一层 `motor_get_vehicle_rpm()`，专门把速度符号统一成“车辆前进为正”

这里再补一条基于 PDF 重新核对后的结论：

- 理论文档里的示例应用层代码写的是 `pitch_kalman / gyro_y_rate`
- 这不是必须照搬的“标准答案”，而是原始移植包的示例轴选择
- 你当前工程已经根据现有 PID 平衡轴，改成 `roll_ctrl_angle / gyro_x_rate`
- 只要坐标系符号和实际平衡轴验证正确，这种替换是合理的项目级适配

---

## 7. 这次为什么要把 LQR 的 IMU 轴改掉

你当前项目里，PID 已经验证了真正用于左右平衡的是：

- `roll_ctrl_angle`
- `gyro_x_rate`

而你移植进来的 LQR 原来用的是：

- `pitch_kalman`
- `gyro_y_rate`

所以现在已经改成：

```c
phi = -(roll_ctrl_angle - lqr_expect_phi) * DEG_TO_RAD;
phi_dot = -gyro_x_rate * DEG_TO_RAD;
```

这里保留负号，是为了沿用原移植模型的符号约定：

- IMU 左倾为正
- LQR 模型按右倾为正

这个符号最终是否完全正确，仍然需要你上车验证。

---

## 8. LQR 和原 PID 的核心区别

### 8.1 PID

你原来的 PID 方案本质上是：

- 角度误差做 P
- 角度误差积分做 I
- 陀螺仪角速度做 D

优点：

- 易懂
- 易上手
- 调参直观

缺点：

- 很依赖经验
- 多状态耦合时不自然

### 8.2 LQR

LQR 更像是：

- 同时反馈多个状态
- 增益由模型和权重矩阵推导
- 可以按速度做增益调度

优点：

- 结构统一
- 更适合自行车这种耦合系统

缺点：

- 极度依赖模型是否靠谱
- 参数和符号一旦错，效果会很差

### 8.3 对智能单车最重要的一点

你的车是“靠转向维持平衡”的智能单车。

所以必须知道：

- 这类 LQR 模型一般依赖前进速度
- 低速、接近静止时，靠转向维持平衡会明显变差

当前代码里已经有低速保护：

- 速度低于 `v_min` 时，不走正常 LQR
- 转向角命令只做缓慢回中

所以静止表现不像 PID 那样积极，是模型特性，不一定是代码错误。

---

## 9. 你现在必须自己改、不能直接照搬的参数

### 9.1 质心高度 `h`

位置：

- `lqr_set_physical_params(&lqr_ctrl, 0.12f, 0.21f, 0.02f);`

问题：

- `0.12m` 很可能不是你车的真实质心高度

### 9.2 轴距 `L`

问题：

- `0.21m` 只是移植初值

### 9.3 拖曳距 `b`

问题：

- 这是前叉几何参数，不能乱填

### 9.4 轮子半径 `wheel_radius`

位置：

- `LQR_WHEEL_RADIUS`

当前你已经确认车轮直径为 `64mm`，所以当前默认半径应为：

- `0.032m`

问题：

- 如果以后换胎或者实际落地半径与静态外径不一致，仍然要按真实滚动半径修正

### 9.5 减速比 `gear_ratio`

位置：

- `LQR_GEAR_RATIO`

当前你已经确认：

- 本车为轮驱一体
- 当前减速比就是 `1`

问题：

- 如果以后更换成带减速机构的驱动，这里必须跟着改

### 9.6 最大机械转向角 `LQR_DELTA_MECHANICAL_MAX`

问题：

- 当前 `23°` 只是移植初值

### 9.7 最小有效速度 `v_min`

问题：

- 太低：模型失真阶段还在强控
- 太高：明明能控却过早退出

### 9.8 增益表 `default_gain_entries[]`

位置：

- [lqr_driver.c](/E:/AURIX-v1.10.28-workspace/LIUHE_bicycle1/code/driver/lqr/lqr_driver.c)

这是最不能直接迷信的一项。

因为这张表是别人基于别人的：

- 物理参数
- 速度范围
- `Q/R` 权重

离线算出来的。

只要车不一样，这张表就不是最终表。

补充两条和当前工程直接相关的事实：

- 这张表的来源脚本就是 [lqr_gain_generator.py](/E:/AURIX-v1.10.28-workspace/LIUHE_bicycle1/code/tools/lqr_gain_generator.py)
- 脚本里的 `w` 和工程代码里的 `L` 是同一个物理量，都是轴距

当前默认表对应的脚本默认条件是：

- `h=0.12`
- `w=0.21`
- `b=0.02`
- `Q=diag(100, 15, 30)`
- `R=2`
- `v_min=0.5`
- `v_max=7.0`
- `n_points=15`

需要特别说明一个“文档源之间的差异”：

- `README.pdf` / `纯舵机驱动单车控制系统的LQR模型.pdf` 里有示例代码片段使用 `np.arange(0.5, 7.5, 0.5)`
- 但当前仓库真正提供的 [lqr_gain_generator.py](/E:/AURIX-v1.10.28-workspace/LIUHE_bicycle1/code/tools/lqr_gain_generator.py) 是可配置点数的 GUI 版，默认 `15` 点
- 当前工程里的 `default_gain_entries[]` 也明显对应这份脚本，而不是 PDF 里的那段示例

所以以后你要重算增益表时，执行依据应当是：

1. 以仓库里的 `lqr_gain_generator.py` 为准
2. 以当前工程实际使用的 `default_gain_entries[]` 为核对对象
3. 不要直接照抄 PDF 中的示例脚本片段

所以你如果后面改了：

- `h / L / b`
- `Q / R`
- 速度范围
- 速度点数

就不能只改运行时宏，还必须重新生成并替换 `default_gain_entries[]`。

---

## 10. 这些量该怎么测

### 10.1 轴距 `L`

- 车辆自然落地
- 测前后轮接地点中心的水平距离
- 没条件时，可先用前后轮轴中心距近似

### 10.2 轮子半径 `r`

不要直接量悬空外径。

更实用的方法：

- 车辆带上电池和主要载荷
- 正常落地
- 测轮轴中心到地面的距离

### 10.3 最大机械转向角

方法：

1. 舵机打到左机械极限
2. 测前轮相对车架中心线偏转角
3. 再测右边
4. 取较小的一侧作为安全最大值

### 10.4 `motor_get_raw_rpm()` 单位确认

这一项你现在已经确认完成：

- `motor_get_raw_rpm()` 返回的是电机转速 `RPM`

因此当前工程里：

- `motor_app.c` 的车速换算已经改回 `RPM -> m/s`
- `lqr_get_velocity()` 不再自己重复写换算公式，而是直接调用电机层的 `motor_get_vehicle_speed_abs_m_s()`

你后续真正要核实的，不再是“它是不是 RPM”，而是：

- 当前显示的 RPM 和真实轮速是否一致
- 在 `减速比=1`、`车轮直径=64mm` 条件下，换算出来的 `m/s` 是否和实车速度接近

### 10.5 为什么 LQR 要取绝对值

现在电机层提供两种速度接口：

- `motor_get_vehicle_speed_m_s()`：带方向，前进为正，后退为负
- `motor_get_vehicle_speed_abs_m_s()`：只取速度大小，恒正

LQR 当前使用的是第二个。

原因不是“偷懒”，而是当前这张 LQR 增益表本来就是按自行车前进平衡模型接进来的：

- 增益调度主要关心车速大小
- 当前没有单独建立“倒车平衡模型”
- 所以 LQR 现在只按 `|v|` 调度更合理

如果你以后要认真支持倒车工况，应该重新审视模型和增益表，而不是简单把绝对值去掉。

### 10.6 质心高度 `h`

工程近似法：

- 整车装到实际工作状态
- 找出整车大致质心位置
- 测这个点到地面的高度

前期不用追求特别精细，但不能偏差太离谱。

### 10.7 拖曳距 `b`

方法：

1. 车侧面尽量正着拍照
2. 画出转向轴线
3. 延长到地面
4. 测该落点到前轮接地点中心的水平距离

有 CAD 就直接从模型取。

---

## 11. 接下来怎么调

建议按这个顺序，不要乱。

### 第一步：先验证方向

低风险验证：

1. 让车低速前进
2. 人手扶着
3. 轻轻向左推，让车左倾
4. 看舵机是否往“能救车”的方向打

如果出现“越扶越倒”，先查方向，不要先调增益。

重点查：

- `phi` 正负号
- `phi_dot` 正负号
- `delta -> servo_pwm` 左右映射

### 第二步：确认速度估计

如果速度量错了，整张增益表都会用错。

### 第三步：确认低速保护阈值

先保守一点，只在较稳定速度区间启用 LQR。

### 第四步：确认机械限位

如果 `delta_max` 写大了，舵机会频繁撞限位。

### 第五步：最后才是重算增益表

如果前面都确认了，效果还是一般，那就该：

- 用你自己的 `h/L/b`
- 用你自己的速度范围
- 重新离线算一张新的 LQR 增益表

而不是继续死磕别人的表。

---

## 12. 什么时候说明当前增益表不适合你

下面这些现象基本都说明移植表不匹配：

- 某一段速度稳，换个速度段就明显不稳
- 舵机经常打满边
- 能纠回来，但伴随明显蛇形摆动
- 调 `delta_max`、`v_min` 后仍然很不自然

这时优先怀疑：

- 速度单位
- 物理参数
- 增益表本身

不是先怀疑 IMU。

---

## 13. 如何切回 PID

当前 PID 和 LQR 都保留，切换不需要再去手改中断入口。

只需要修改 [balance_control_mode.h](/E:/AURIX-v1.10.28-workspace/LIUHE_bicycle1/code/app/balance_control_mode.h) 中的：

```c
// 设为 PID：台架联调、无速度测试
// 设为 LQR：实车前进平衡测试
#define BALANCE_CONTROL_MODE       (BALANCE_CONTROL_MODE_PID)
```

如果要切到 LQR，就改成：

```c
#define BALANCE_CONTROL_MODE       (BALANCE_CONTROL_MODE_LQR)
```

`user/isr.c` 和 `user/cpu0_main.c` 已经接好了这组宏，不需要再额外修改控制入口。

---

## 14. 作为初学者，先记住这 6 句话

1. LQR 不是魔法，它只是更系统的状态反馈。
2. LQR 好不好，先看模型对不对。
3. 对智能单车来说，速度是 LQR 成立的重要前提。
4. 速度单位错了，整套增益调度都会废。
5. 别人的增益表只能拿来上电试，不是你的最终答案。
6. 先验方向、再验速度、再验几何，最后才重算增益。

---

## 15. 下一步最值得做的事

如果你继续让我往下做，最有价值的不是继续盲调，而是：

1. 给 IPS 增加一个最小化的 LQR 调试页
   - `velocity`
   - `K_phi/K_phi_dot/K_delta`
   - `delta_cmd`
   - `u_raw`
2. 我帮你把“速度单位验证”和“转向方向验证”整理成一套上车测试流程

这两件事做完，你再调 LQR，效率会高很多。
