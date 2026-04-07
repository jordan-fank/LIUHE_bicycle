/*
 * 文件: cpu0_main.c
 * 功能: CPU0 主入口，负责系统初始化、控制链启动和主循环调度
 * 作者: 闫锦
 * 日期: 2026-03-31
 */

#include "cpu0_main.h"
#include "servo_app.h"
#include "motor_pid.h"
#include "balance_control_mode.h"
/* [新增] 无线调试模块头文件 */
#include "wireless_debug_app.h"
/* [新增] GPS/IMU 导航模块头文件 */
#include "nav_app.h"
/* [新增 科目1] */
#include "subject1_app.h"
/* [新增 科目2] */
#include "subject2_app.h"
/* [新增 科目3] */
#include "subject3_app.h"
#include "key_app.h"
/* [新增 Flash持久化] */
#include "config_flash.h"
#include "isr.h"

uint8_t g_home_init_imu_ok    = 0u;
uint8_t g_home_init_gps_ok    = 0u;
uint8_t g_home_init_motor_ok  = 0u;
uint8_t g_home_init_wifi_ok   = 0u;
uint8_t g_home_init_nav_ok    = 0u;
uint8_t g_home_flash_loaded   = 0u;


int core0_main(void)
{

    clock_init();                                   // 时钟初始化
    debug_init();                                   // 串口初始化
    isr_diag_reset();                               // 清零 5ms 控制 ISR 耗时统计

    led_init();                                     // 灯初始化
    key_app_init();                                 // 按键初始化
    ips_app_init();                                 //初始化ips
    gps_init();
    g_home_init_gps_ok = 1u;


    pwm_init(ATOM1_CH1_P33_9, SERVO_FREQ, g_servo_mid_duty);     //舵机PWM初始化
    servo_init();                                   // 舵机初始化

    balance_pid_init();                             //舵机PID平衡调节
    lqr_balance_init();                             // LQR平衡控制初始化 


    balance_control_set_enable(0U);                 //关闭舵机平衡控制，防止干扰IMU校准
    lqr_balance_set_enable(0U);                     // 关闭LQR控制，防止干扰IMU零位捕获




    imu_all_init();                                  // IMU硬件初始化 + 四元数初始化
    imu_calibrate_gyro();                            //IMU精确校准耗时60S--比赛用
    //imu_calibrate_gyro_temp();                     // 临时零偏值仅适合快速联调，不适合作为精度验证基准
    g_home_init_imu_ok = 1u;


    pit_ms_init(CCU60_CH1, IMU_PERIOD_MS);          // 定时中断初始化->5MS IMU数据采集
    zero_compensation();                            //机械零位捕获，在平衡控制关闭状态下完成--小BUG-无超时保护





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
    motor_pid_init();                                // 初始化电机速度PID
    motor_output_set_enable(0u);                     // 统一启停框架：所有模式上电默认不放行电机输出
    uart_receiver_init();
    
    pit_ms_init(CCU61_CH0, 20);                      // 定时中断初始化->20MS 串口数据处理
    g_home_init_motor_ok = 1u;

    


    wireless_debug_init();                          //无线调试模块初始化-WiFi SPI + 逐飞助手，连接失败打印失败
    g_home_init_wifi_ok = 1u;

    /* ========================================================
     * [新增] GPS/IMU 导航模块初始化
     * 放在 GPS 驱动初始化之后，WiFi 初始化之后，调度器之前。
     * nav_app_init 仅清零内部状态，不启动导航（需手动调用
     * nav_start_gps() 或 nav_start_imu() 启动）。
     * ======================================================== */
    nav_app_init();                                 //GPS/IMU 导航模块初始化
    g_home_init_nav_ok = 1u;

    /* [新增 Flash持久化] 从 Flash 加载保存的参数（PID + GPS路点）
     * 须在 nav_app_init() 之后（否则 init 会清零刚恢复的路点），
     * 内部会调用 motor_pid_init() 重新应用已加载的电机 PID 参数。
     * 若 Flash 无有效配置（首次上电），保持编译期默认值不变。 */
    g_home_flash_loaded = config_flash_load();

    subject1_app_init();                            // [新增 科目1] 科目1状态机初始化
    subject2_app_init();                            // [新增 科目2] 科目2状态机初始化（无Flash，段序列由参数计算）
    subject3_app_init();                            // [新增 科目3] 科目3状态机初始化（从Flash page 10 恢复路点）
    active_subject_set(g_active_subject);           // 根据当前活跃科目自动同步舵机控制模式

    scheduler_init();                               // 调度初始化
    ips_app_init();    // Flash 加载和各科目初始化完成后重绘 IPS，HOME 页显示最终状态
    

    cpu_wait_event_ready();                         // 等待事件就绪

    led_all_set(LED_ON);                            //LED 测试->初始化完成标志


    printf("Init OK\r\n");

    while (1)
    {
        imu_proc();      // 事件驱动，优先级最高，放在while，减轻中断压力
        scheduler_run();  // 调度运行
    }
}
