# IPS 显示屏使用指南

---

## 1. 系统架构

```
key_app.c                    （按键层：触发 ips_app 的 API）
    ↓
ips_app.c / ips_app.h        （应用层：页面逻辑、参数管理）
ips_app_config.h             （配置层：页面枚举、变量声明、参数表定义）
    ↓
ips_driver.c / ips_driver.h  （驱动层：封装逐飞库，提供绘图原语）
    ↓
逐飞库 ips200                （底层硬件驱动，不要动）
```

**用户只需要改 `ips_app_config.h` 和 `ips_app.c`，其余文件无需触碰。**

---

## 2. 快速接入

### 2.1 初始化

在主函数中调用一次：

```c
#include "ips_app.h"

ips_app_init();   // 放在所有外设初始化完成之后
```

### 2.2 加入调度器（schedule.c，已配置好）

```c
static task_t scheduler_task[] =
{
    {key_scan,      10,  0},   // 按键扫描 10ms
    {key_task,      10,  0},   // 按键处理 10ms
    {ips_app_task, 200,  0},   // 屏幕刷新 200ms（只刷新变化的只读数值）
};
```

### 2.3 按键功能（4个按键）

| 按键 | 操作 | 功能 |
|------|------|------|
| KEY1 | 短按 | 减小当前选中参数（单次） |
| KEY1 | 长按 | 连续减小当前选中参数 |
| KEY2 | 短按 | 增大当前选中参数（单次） |
| KEY2 | 长按 | 连续增大当前选中参数 |
| KEY3 | 短按 | 切换到下一页 |
| KEY3 | 双击 | 切换到上一页 |
| KEY4 | 短按 | 向下选参数 |
| KEY4 | 双击 | 向上选参数 |

> 长按超过 800ms 触发，之后每 10ms 连续触发一次，松手立即停止。

---

## 3. 显示效果

- 白底黑字，标题居中，下方有分隔线
- 参数行：`-> Kp:   15.00`，`->` 表示当前选中，数值右对齐
- 切页立即刷新；按键只局部刷新受影响的行，无闪烁

---

## 4. 填写参数表：PARAM_F 和 PARAM_I 两个宏

这是本系统的核心用法。填参数表时，根据变量类型选择宏：

```c
// 浮点变量 → 用 PARAM_F
PARAM_F(标签, &float变量, 步长, 最小值, 最大值, 是否只读, 小数位数)

// 整数变量 → 用 PARAM_I
PARAM_I(标签, &int32_t变量, 步长, 最小值, 最大值, 是否只读)
```

### PARAM_F 参数说明

| 参数 | 类型 | 说明 |
|------|------|------|
| 标签 | `"xxx:"` | 屏幕上显示的名称 |
| &变量 | `float *` | 指向 float 变量 |
| 步长 | `float` | 每次按键增减的量，只读时填 `0` |
| 最小值 | `float` | 下限，只读时填 `0` |
| 最大值 | `float` | 上限，只读时填 `0` |
| 是否只读 | `0` / `1` | `1` = 只读，不响应增减按键 |
| 小数位数 | `1` / `2` | `1` → `25.6`，`2` → `15.00` |

### PARAM_I 参数说明

| 参数 | 类型 | 说明 |
|------|------|------|
| 标签 | `"xxx:"` | 屏幕上显示的名称 |
| &变量 | `int32_t *` | 指向 int32_t 变量 |
| 步长 | `float` | 每次按键增减的量（会取整），只读时填 `0` |
| 最小值 | `float` | 下限，只读时填 `0` |
| 最大值 | `float` | 上限，只读时填 `0` |
| 是否只读 | `0` / `1` | `1` = 只读，不响应增减按键 |

> `PARAM_I` 没有小数位参数，始终显示整数。

### 示例

```c
#define MY_PARAM_LIST \
    PARAM_F("Kp:",    &g_kp,      0.1f, 0.0f, 100.0f, 0, 2), \  // 可调浮点，显示 x.xx
    PARAM_F("Speed:", &g_speed,   0,    0,    0,       1, 1), \  // 只读浮点，显示 x.x
    PARAM_I("RPM:",   &g_rpm,     10,   0,    9000,    0),    \  // 可调整数，步长10
    PARAM_I("Count:", &g_cnt,     0,    0,    0,       1)        // 只读整数
```

---

## 5. 添加新页面（完整步骤）

以新增"传感器页面"（显示温度/转速）为例，共 6 步：

### 第一步：`ips_app_config.h` — 注册页面枚举

在 `PAGE_COUNT` 前插入新枚举值：

```c
typedef enum
{
    PAGE_MOTOR = 0,
    PAGE_SERVO,
    PAGE_BATTERY,
    PAGE_SENSOR,    // ← 新增
    PAGE_COUNT      // 不要删除
} page_e;
```

### 第二步：`ips_app_config.h` — 声明变量

根据变量类型用 `float` 或 `int32_t`：

```c
extern float   g_sensor_temp;   // 温度，浮点
extern int32_t g_sensor_rpm;    // 转速，整数
```

### 第三步：`ips_app_config.h` — 定义参数表

```c
#define SENSOR_PARAM_LIST \
    PARAM_F("Temp:",  &g_sensor_temp, 0, 0, 0, 1, 1), \  // 只读，显示 x.x
    PARAM_I("RPM:",   &g_sensor_rpm,  0, 0, 0, 1)         // 只读，整数显示
```

### 第四步：`ips_app.c` — 定义变量和参数数组

```c
// 变量定义（找到其他变量定义的区域，照着加）
float   g_sensor_temp = 0.0f;
int32_t g_sensor_rpm  = 0;

// 参数数组（找到其他 s_xxx_params 的区域，照着加）
static param_t s_sensor_params[] = { SENSOR_PARAM_LIST };
```

### 第五步：`ips_app.c` — 在 `get_params()` 注册

```c
case PAGE_SENSOR:
    *count = ARRAY_LEN(s_sensor_params);
    return s_sensor_params;
```

### 第六步：`ips_app.c` — 在 `get_title()` 注册标题

```c
case PAGE_SENSOR: return "Sensor";
```

**完成！** 按 KEY3 切页即可看到新页面。

---

## 6. 实时更新只读数值

只读变量（`rdonly=1`）由 `ips_app_task()` 每 200ms 检测一次，**值有变化才刷新显示**。在任何任务里直接赋值即可：

```c
// float 变量直接赋值
g_bat_voltage  = adc_read_voltage();
g_sensor_temp  = imu_get_temp();

// int32_t 变量直接赋值
g_sensor_rpm   = encoder_get_rpm();
g_cnt          = get_lap_count();
```

---

## 7. 常见问题

| 问题 | 原因 | 解决 |
|------|------|------|
| 屏幕一直闪烁 | 每次任务都清全屏 | 已修复：切页才清屏，按键只局部刷新 |
| 只读页面闪烁 | task 每次都重绘只读行 | 已修复：只有数值变化时才重绘 |
| 按键无反应/有延迟 | 等任务周期才重绘 | 已修复：按键 API 内部直接绘制，立即响应 |
| 长按没有连续增减 | 驱动阻塞等待松手 | 已修复：驱动改为非阻塞，松手才停止 |
| 整数变量显示为 0 | 误用 `uint32_t`/`int` 而非 `int32_t`，或变量声明类型不匹配 | 整数变量统一用 `int32_t`，用 `PI` 宏填表 |
| 新页面按 KEY3 找不到 | `get_params()` 或 `get_title()` 漏加 case | 检查第五、六步是否都加了 |
| 只读参数按键无效 | 正常现象 | `rdonly=1` 的参数不响应增减按键 |
