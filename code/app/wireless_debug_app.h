/*
 * 文件: wireless_debug_app.h
 * 功能: 无线调试应用层 - 声明初始化和任务接口
 *       通过高速 WiFi SPI 模块 + 逐飞助手 PC 软件实现：
 *         1. 无线参数调试（PC → MCU，最多 8 通道 float）
 *         2. 无线波形显示（MCU → PC，最多 8 通道 float）
 *         两者可同时工作，参数下行、波形上行互不干扰。
 *
 * 使用方法：
 *   1. 修改下方 WiFi/TCP 配置宏，填入真实的 SSID、密码、PC IP
 *   2. 在 cpu0_main.c 的电机初始化之后调用 wireless_debug_init()
 *   3. 在 scheduler.c 的任务表中加入 wireless_debug_task, 50ms 周期
 *   4. PC 端打开逐飞助手，选 TCP 服务器模式，端口与下方 PORT 一致，点击连接
 *
 * 参数通道分配（逐飞助手 PC 端 CH1~CH8 对应关系）：
 *   CH1 → g_balance_kp  (舵机平衡 Kp)
 *   CH2 → g_balance_ki  (舵机平衡 Ki)
 *   CH3 → g_balance_kd  (舵机平衡 Kd)
 *   CH4 → g_motor_kp    (电机速度 Kp)
 *   CH5 → g_motor_ki    (电机速度 Ki)
 *   CH6 → g_motor_kd    (电机速度 Kd)
 *   CH7 → target_motor_rpm (目标转速 RPM)
 *   CH8 → 预留
 *
 * 波形通道分配（逐飞助手 PC 端示波器 CH1~CH6）：
 *   CH1 → roll_ctrl_angle     (补偿后横滚角 °)
 *   CH2 → balance_pid.out     (舵机 PID 输出)
 *   CH3 → motor_speed_rpm     (电机实际转速 RPM)
 *   CH4 → g_motor_pid_output  (电机 PID 输出)
 *   CH5 → target_motor_rpm    (目标转速 RPM)
 *   CH6 → pitch_ctrl_angle    (补偿后俯仰角 °)
 *
 * 作者: 闫锦
 * 日期: 2026-04-01
 */

#ifndef CODE_APP_WIRELESS_DEBUG_APP_H_
#define CODE_APP_WIRELESS_DEBUG_APP_H_

#include "zf_common_headfile.h"

/* ===========================================================
 * WiFi 连接配置（必须填写，否则 init 会直接退出）
 * =========================================================== */
#define WIRELESS_WIFI_SSID          "YOUR_WIFI_SSID_HERE"      // 路由器 SSID
#define WIRELESS_WIFI_PASSWORD      "YOUR_WIFI_PASSWORD_HERE"  // 路由器密码
#define WIRELESS_TCP_SERVER_IP      "YOUR_PC_IP_HERE"          // PC 的 IP（如 "192.168.1.100"）
#define WIRELESS_TCP_SERVER_PORT    "8086"                     // 逐飞助手 TCP 端口（默认 8086）

/* ===========================================================
 * 调试任务周期（ms）
 * 建议范围：20~100ms。越小波形越实时，但会占用更多主循环时间。
 * =========================================================== */
#define WIRELESS_DEBUG_TASK_PERIOD_MS   50U

/* ===========================================================
 * 公开 API
 * =========================================================== */

/**
 * @brief  初始化无线调试模块
 * @detail 完成 WiFi SPI 硬件初始化、TCP 连接、逐飞助手接口绑定。
 *         调用位置：cpu0_main.c，motor_pid_init() 之后，scheduler_init() 之前。
 *         若连接失败，仅打印提示，不阻塞后续系统启动。
 */
void wireless_debug_init(void);

/**
 * @brief  无线调试周期任务（由 scheduler 驱动，50ms 周期）
 * @detail 每次调用完成两件事：
 *           1. 调用 seekfree_assistant_data_analysis() 解析并应用 PC 下发的参数
 *           2. 打包当前状态量，调用 seekfree_assistant_oscilloscope_send() 发送波形
 *         若 WiFi 未连接，立即返回，不影响系统运行。
 */
void wireless_debug_task(void);

#endif /* CODE_APP_WIRELESS_DEBUG_APP_H_ */
