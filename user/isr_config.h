/*
 * 文件: isr_config.h
 * 功能: 中断服务对象和优先级配置
 * 作者: 闫锦
 * 日期: 2026-03-31
 */

#ifndef _isr_config_h
#define _isr_config_h

/*
 * TC264 中断配置说明
 *
 * 1. *_INT_SERVICE
 *    指定该中断由谁来响应。
 *    常见取值：
 *    - IfxSrc_Tos_cpu0：挂到 CPU0
 *    - IfxSrc_Tos_cpu1：挂到 CPU1
 *    - IfxSrc_Tos_dma ：挂到 DMA
 *
 * 2. *_ISR_PRIORITY / *_INT_PRIO
 *    中断优先级。
 *    对 AURIX 来说，数值越大优先级越高。
 *
 * 3. 建议：
 *    - 固定节拍、核心控制链路的中断优先级应高于普通外设
 *    - 串口、屏幕、调试类业务优先级通常不要抢过核心采样 / 控制链路
 */

// ============================ PIT 定时中断配置 ============================
// CCU6_0 / CCU6_1 的 4 个 PIT 通道

#define CCU6_0_CH0_INT_SERVICE  IfxSrc_Tos_cpu0
#define CCU6_0_CH0_ISR_PRIORITY 30

#define CCU6_0_CH1_INT_SERVICE  IfxSrc_Tos_cpu0
#define CCU6_0_CH1_ISR_PRIORITY 31

#define CCU6_1_CH0_INT_SERVICE  IfxSrc_Tos_cpu0
#define CCU6_1_CH0_ISR_PRIORITY 32

#define CCU6_1_CH1_INT_SERVICE  IfxSrc_Tos_cpu0
#define CCU6_1_CH1_ISR_PRIORITY 33

// ============================ ERU / GPIO 外部中断配置 ============================
// 注意：
// - 通道0 与 通道4 共用一个 ISR
// - 通道1 与 通道5 共用一个 ISR
// - 通道2 与 通道6 共用一个 ISR
// - 通道3 与 通道7 共用一个 ISR
// 因此在 ISR 内必须再通过标志位判断究竟是哪一路触发。

#define EXTI_CH0_CH4_INT_SERVICE IfxSrc_Tos_cpu0
#define EXTI_CH0_CH4_INT_PRIO    40

#define EXTI_CH1_CH5_INT_SERVICE IfxSrc_Tos_cpu0
#define EXTI_CH1_CH5_INT_PRIO    41

// 这一组通常与摄像头 / DMA 触发链路相关，服务对象设置为 DMA
#define EXTI_CH2_CH6_INT_SERVICE IfxSrc_Tos_dma
#define EXTI_CH2_CH6_INT_PRIO    5

#define EXTI_CH3_CH7_INT_SERVICE IfxSrc_Tos_cpu0
#define EXTI_CH3_CH7_INT_PRIO    43

// ============================ DMA 中断配置 ============================
#define DMA_INT_SERVICE          IfxSrc_Tos_cpu0
#define DMA_INT_PRIO             60

// ============================ UART 中断配置 ============================
// UART0：通常作为调试串口
#define UART0_INT_SERVICE        IfxSrc_Tos_cpu0
#define UART0_TX_INT_PRIO        11
#define UART0_RX_INT_PRIO        10
#define UART0_ER_INT_PRIO        12

// UART1：当前工程默认连接摄像头相关串口
#define UART1_INT_SERVICE        IfxSrc_Tos_cpu0
#define UART1_TX_INT_PRIO        13
#define UART1_RX_INT_PRIO        14
#define UART1_ER_INT_PRIO        15

// UART2：当前工程默认连接无线模块
// 当前 UART2 的 ISR 入口均定义在 CPU0，因此服务核必须与 ISR 归属保持一致。
#define UART2_INT_SERVICE        IfxSrc_Tos_cpu0
#define UART2_TX_INT_PRIO        16
#define UART2_RX_INT_PRIO        17
#define UART2_ER_INT_PRIO        18

// UART3：当前工程默认连接 GNSS / GPS 模块
#define UART3_INT_SERVICE        IfxSrc_Tos_cpu0
#define UART3_TX_INT_PRIO        19
#define UART3_RX_INT_PRIO        20
#define UART3_ER_INT_PRIO        21

#endif
