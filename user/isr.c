/*
 * 文件: isr.c
 * 功能: 中断服务入口实现，负责周期采样、控制节拍和外设中断分发
 * 作者: 闫锦
 * 日期: 2026-03-31
 */

#include "isr_config.h"
#include "isr.h"
#include "balance_control_mode.h"
#include "nav_app.h"
#include "servo_pid.h"



/*========================================================*/
//中断处理时间测试系统
/*========================================================*/

volatile uint32_t g_ctrl_5ms_last_us  = 0u;
volatile uint32_t g_ctrl_5ms_max_us   = 0u;
volatile uint32_t g_ctrl_5ms_min_us   = 0xFFFFFFFFu;
volatile uint32_t g_ctrl_5ms_run_count = 0u;


static uint32_t isr_elapsed_us(uint32_t start_tick)
{
    /* system_getval() 返回 10ns 计数，先做无符号差分再换算为 us。 */
    return (system_getval() - start_tick) / 100U;
}

void isr_diag_reset(void)
{
    g_ctrl_5ms_last_us   = 0u;
    g_ctrl_5ms_max_us    = 0u;
    g_ctrl_5ms_min_us    = 0xFFFFFFFFu;
    g_ctrl_5ms_run_count = 0u;
}




/*
 * TC264 / TC264D 中断文件说明
 *
 * 1. 本文件只负责“中断入口”。
 *    每个 IFX_INTERRUPT(...) 定义一个中断服务函数，函数名必须与启动配置一致。
 *
 * 2. 中断里最重要的三件事：
 *    - 如有需要，先重新打开全局中断，允许更高优先级中断嵌套
 *    - 清除本次中断源的标志位，避免重复进入
 *    - 只做最小必要工作：搬运数据、置位标志、调用极短的回调
 *
 * 3. 当前工程的设计原则：
 *    - IMU 的重计算不放在 ISR 内
 *    - ISR 只负责固定节拍采样
 *    - 主循环中的 imu_proc() 负责姿态解算
 *
 * 4. IFX_INTERRUPT(函数名, CPU核号, 优先级) 含义：
 *    - 函数名：中断入口函数名
 *    - CPU核号：当前中断挂在哪个 CPU 上，这里均为 CPU0
 *    - 优先级：数值越大优先级越高（由 isr_config.h 统一配置）
 *
 * 5. interrupt_global_enable(0) 的作用：
 *    TC264 进入中断后，硬件会默认关全局中断。
 *    如果希望当前 ISR 执行时，仍允许更高优先级中断抢占，就需要在 ISR 开头手动重新开中断。
 *    当前工程沿用了逐飞库的这种写法。
 */

// ============================ PIT 定时中断 ============================
// 这类中断由 CCU6/PIT 周期触发，常用于固定节拍任务。
// 当前工程里：
// - CCU60_CH1 用于 IMU 5ms 周期采样

//PIT1 -- 优先级30
IFX_INTERRUPT(cc60_pit_ch0_isr, 0, CCU6_0_CH0_ISR_PRIORITY)             
{
    interrupt_global_enable(0);   // 允许更高优先级中断在本 ISR 执行期间抢占
    pit_clear_flag(CCU60_CH0);    // 清除 CCU60 通道0 PIT 中断标志

    // 预留通道：当前未绑定业务逻辑
}

//PIT2 -- 优先级31  --5ms
IFX_INTERRUPT(cc60_pit_ch1_isr, 0, CCU6_0_CH1_ISR_PRIORITY)
{
    float nav_expect_phi = 0.0f;

    uint32_t start_tick  = system_getval();

    interrupt_global_enable(0);   // 允许更高优先级中断嵌套
    pit_clear_flag(CCU60_CH1);    // 清除 CCU60 通道1 PIT 中断标志

    // IMU 固定节拍采样：
    // 这里只采原始数据，不做姿态解算，避免 ISR 过重。
    imu_sample_isr();

    /* 导航闭环接线：
       统一在 5ms 控制节拍中把导航输出转换为控制器参考量。
       非导航态明确写 0，避免上一次导航残留参考值继续作用。 */
    if (g_nav_state == NAV_STATE_NAVIGATING)
    {
        nav_expect_phi = g_nav_heading_error * g_nav_heading_gain;
    }

    balance_set_expect_angle(nav_expect_phi);
    lqr_set_expect_phi(nav_expect_phi);

    /* ==================== 平衡控制方案一键切换 ====================
       实际调用哪套平衡控制，由 balance_control_mode.h 中的宏统一决定。
    */
#if (BALANCE_CONTROL_MODE == BALANCE_CONTROL_MODE_PID)
    balance_control();
#else
    lqr_balance_control();
#endif

    //测试中断耗时测试，发现目前max==39us,中断状态很好
    {
        uint32_t elapsed_us = isr_elapsed_us(start_tick);
        g_ctrl_5ms_last_us = elapsed_us;
        if (elapsed_us > g_ctrl_5ms_max_us)
        {
            g_ctrl_5ms_max_us = elapsed_us;
        }
        if (elapsed_us < g_ctrl_5ms_min_us)
        {
            g_ctrl_5ms_min_us = elapsed_us;
        }
        g_ctrl_5ms_run_count++;
    }
}

//PIT3 -- 优先级32
IFX_INTERRUPT(cc61_pit_ch0_isr, 0, CCU6_1_CH0_ISR_PRIORITY)
{
    interrupt_global_enable(0);   // 允许更高优先级中断嵌套
    pit_clear_flag(CCU61_CH0);    // 清除 CCU61 通道0 PIT 中断标志

    // 预留通道：当前未绑定业务逻辑
    motor_control();                                // 电机速度PID控制（20ms周期）
}

//PIT4 -- 优先级33
IFX_INTERRUPT(cc61_pit_ch1_isr, 0, CCU6_1_CH1_ISR_PRIORITY)
{
    interrupt_global_enable(0);   // 允许更高优先级中断嵌套
    pit_clear_flag(CCU61_CH1);    // 清除 CCU61 通道1 PIT 中断标志

    // 预留通道：当前未绑定业务逻辑
}




// ============================ 外部中断 ERU ============================
// AURIX 的 ERU（External Request Unit）会把多个外部请求线分组到同一个 ISR。
// 所以一个 ISR 内通常要靠 exti_flag_get() 判断“究竟是哪一路触发”。

IFX_INTERRUPT(exti_ch0_ch4_isr, 0, EXTI_CH0_CH4_INT_PRIO)
{
    interrupt_global_enable(0);   // 允许更高优先级中断嵌套

    // ERU 通道0：P15.4
    if (exti_flag_get(ERU_CH0_REQ0_P15_4))
    {
        exti_flag_clear(ERU_CH0_REQ0_P15_4);   // 先清标志，再处理回调

        // IMU660RC 模块的 INT 引脚中断回调
        imu660rc_callback();
    }

    // ERU 通道4：P15.5
    if (exti_flag_get(ERU_CH4_REQ13_P15_5))
    {
        exti_flag_clear(ERU_CH4_REQ13_P15_5);

        // 预留外部中断通道，当前未绑定业务
    }
}

IFX_INTERRUPT(exti_ch1_ch5_isr, 0, EXTI_CH1_CH5_INT_PRIO)
{
    interrupt_global_enable(0);   // 允许更高优先级中断嵌套

    // ERU 通道1：P14.3
    if (exti_flag_get(ERU_CH1_REQ10_P14_3))
    {
        exti_flag_clear(ERU_CH1_REQ10_P14_3);

        // ToF 模块的 INT 引脚中断回调
        tof_module_exti_handler();
    }

    // ERU 通道5：P15.8
    if (exti_flag_get(ERU_CH5_REQ1_P15_8))
    {
        exti_flag_clear(ERU_CH5_REQ1_P15_8);

        // 预留外部中断通道，当前未绑定业务
    }
}

/*
 * 通道2 / 通道6 默认与摄像头 PCLK / DMA 触发链路相关，
 * 当前工程不在这里定义对应 ISR。
 *
 * IFX_INTERRUPT(exti_ch2_ch6_isr, 0, EXTI_CH2_CH6_INT_PRIO)
 * {
 *     interrupt_global_enable(0);
 *     if (exti_flag_get(ERU_CH2_REQ7_P00_4))
 *     {
 *         exti_flag_clear(ERU_CH2_REQ7_P00_4);
 *     }
 *     if (exti_flag_get(ERU_CH6_REQ9_P20_0))
 *     {
 *         exti_flag_clear(ERU_CH6_REQ9_P20_0);
 *     }
 * }
 */

IFX_INTERRUPT(exti_ch3_ch7_isr, 0, EXTI_CH3_CH7_INT_PRIO)
{
    interrupt_global_enable(0);   // 允许更高优先级中断嵌套

    // ERU 通道3：P02.0
    if (exti_flag_get(ERU_CH3_REQ6_P02_0))
    {
        exti_flag_clear(ERU_CH3_REQ6_P02_0);

        // 摄像头 VSYNC 外部中断回调
        camera_vsync_handler();
    }

    // ERU 通道7：P15.1
    if (exti_flag_get(ERU_CH7_REQ16_P15_1))
    {
        exti_flag_clear(ERU_CH7_REQ16_P15_1);

        // 预留外部中断通道，当前未绑定业务
    }
}



// ============================ DMA 中断 ============================
// DMA 中断通常表示一次 DMA 传输完成，ISR 里应尽量只做很短的收尾与回调。

IFX_INTERRUPT(dma_ch5_isr, 0, DMA_INT_PRIO)
{
    interrupt_global_enable(0);   // 允许更高优先级中断嵌套

    // 摄像头 DMA 采集完成回调
    camera_dma_handler();
}





// ============================ 串口中断 ============================
// 这里分别处理 4 路串口的发送、接收和错误中断。
// 一般来说：
// - TX ISR：发送 FIFO 空、发送完成等场景
// - RX ISR：收到新字节时触发
// - ER ISR：帧错误、溢出等异常

// UART0 默认作为调试串口
IFX_INTERRUPT(uart0_tx_isr, 0, UART0_TX_INT_PRIO)
{
    interrupt_global_enable(0);   // 允许更高优先级中断嵌套

    // 当前未使用 UART0 发送中断
}

IFX_INTERRUPT(uart0_rx_isr, 0, UART0_RX_INT_PRIO)
{
    interrupt_global_enable(0);   // 允许更高优先级中断嵌套

#if DEBUG_UART_USE_INTERRUPT
    // 调试串口接收中断处理：
    // 把收到的数据写入 debug 环形缓冲区，供上层读取
    debug_interrupr_handler();
#endif
}

// UART1 默认连接摄像头配置/通信串口
IFX_INTERRUPT(uart1_tx_isr, 0, UART1_TX_INT_PRIO)
{
    interrupt_global_enable(0);   // 允许更高优先级中断嵌套

    // 当前未使用 UART1 发送中断
}

//电机串口1接收中断-获取电机速度
IFX_INTERRUPT(uart1_rx_isr, 0, UART1_RX_INT_PRIO)
{
    interrupt_global_enable(0);   // 允许更高优先级中断嵌套

    uart_control_callback();                        // FOC驱动串口接收回调

    // // 摄像头串口接收回调
    // camera_uart_handler();
}

// UART2 默认连接无线模块
IFX_INTERRUPT(uart2_tx_isr, 0, UART2_TX_INT_PRIO)
{
    interrupt_global_enable(0);   // 允许更高优先级中断嵌套

    // 当前未使用 UART2 发送中断
}

IFX_INTERRUPT(uart2_rx_isr, 0, UART2_RX_INT_PRIO)
{
    interrupt_global_enable(0);   // 允许更高优先级中断嵌套

    // 无线模块串口接收回调
    wireless_module_uart_handler();
}

// UART3 默认连接 GNSS / GPS 模块
IFX_INTERRUPT(uart3_tx_isr, 0, UART3_TX_INT_PRIO)
{
    interrupt_global_enable(0);   // 允许更高优先级中断嵌套

    // [修复 C3] TX ISR 不应调用接收回调，已移除 gnss_uart_callback()
    // 原错误：发送完成中断中调用 GNSS 接收回调，可能导致数据帧解析重入
}

IFX_INTERRUPT(uart3_rx_isr, 0, UART3_RX_INT_PRIO)
{
    interrupt_global_enable(0);   // 允许更高优先级中断嵌套

    // GNSS 串口接收回调
    gnss_uart_callback();
}



// 串口错误中断：用于处理溢出、帧错误等异常
IFX_INTERRUPT(uart0_er_isr, 0, UART0_ER_INT_PRIO)
{
    interrupt_global_enable(0);
    IfxAsclin_Asc_isrError(&uart0_handle);
}

IFX_INTERRUPT(uart1_er_isr, 0, UART1_ER_INT_PRIO)
{
    interrupt_global_enable(0);
    IfxAsclin_Asc_isrError(&uart1_handle);
}

IFX_INTERRUPT(uart2_er_isr, 0, UART2_ER_INT_PRIO)
{
    interrupt_global_enable(0);
    IfxAsclin_Asc_isrError(&uart2_handle);
}

IFX_INTERRUPT(uart3_er_isr, 0, UART3_ER_INT_PRIO)
{
    interrupt_global_enable(0);
    IfxAsclin_Asc_isrError(&uart3_handle);
}
