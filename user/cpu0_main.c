/*
 * 文件: cpu0_main.c
 * 功能: CPU0 主入口，负责系统初始化、控制链启动和主循环调度
 * 作者: 闫锦
 * 日期: 2026-03-31
 */

#include "cpu0_main.h"
#include "servo_app.h"
#include "balance_control_mode.h"



int core0_main(void)
{

    clock_init();                                   // 时钟初始化
    debug_init();                                   // 串口初始化

    led_init();                                     // 灯初始化
    key_app_init();                                 // 按键初始化
    ips_app_init();                                 //初始化ips
    gps_init();


    pwm_init(ATOM1_CH1_P33_9, SERVO_FREQ, g_servo_mid_duty);     //舵机PWM初始化
    servo_init();                                   // 舵机初始化
    balance_pid_init();                             //舵机PID平衡调节
    lqr_balance_init();                             // ==================== NEW: LQR平衡控制初始化 ====================


    balance_control_set_enable(0U);                 //关闭舵机平衡控制，防止干扰IMU校准
    lqr_balance_set_enable(0U);                     // ==================== NEW: 关闭LQR控制，防止干扰IMU零位捕获 ====================




    imu_all_init();                                  // IMU硬件初始化 + 四元数初始化
    //imu_calibrate_gyro();                            //IMU精确校准耗时60S

    imu_calibrate_gyro_temp();                     // 临时零偏值仅适合快速联调，不适合作为精度验证基准


    pit_ms_init(CCU60_CH1, IMU_PERIOD_MS);          // 定时中断初始化->5MS IMU数据采集
    zero_compensation();                            //机械零位捕获，在平衡控制关闭状态下完成





    /* ==================== 平衡控制方案一键切换 ====================
       这里只根据 balance_control_mode.h 中的宏，决定最终启用 PID 还是 LQR。
       两套控制器都保留初始化，不打乱现有工程结构。
    */
#if (BALANCE_CONTROL_MODE == BALANCE_CONTROL_MODE_PID)
    balance_control_set_enable(1U);
#else
    lqr_balance_set_enable(1U);
#endif



    motor_init();                                    //电机初始化--UART1初始化+开启UART1接受中断        
    motor_pid_init();                               // 初始化电机速度PID
    uart_receiver_init();
    
    
    pit_ms_init(CCU61_CH0, 20);     // 定时中断初始化->20MS 串口数据处理

    



    scheduler_init();                               // 调度初始化
    

    cpu_wait_event_ready();             // 等待事件就绪

    led_all_set(LED_ON);                    //LED 测试->初始化完成标志


    //printf("Init OK\r\n");

    while (1)
    {
        imu_proc();      // 事件驱动，优先级最高，放在while，减轻中断压力
        scheduler_run();  // 调度运行
    }
}
