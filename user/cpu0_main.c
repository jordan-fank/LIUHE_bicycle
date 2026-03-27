#include "cpu0_main.h"
#include "servo_app.h"



int core0_main(void)
{

    clock_init();                                   // 时钟初始化
    debug_init();                                   // 串口初始化

    led_init();                                     // 灯初始化
    key_app_init();                                 // 按键初始化
    ips_app_init();                                 //初始化ips


    pwm_init(ATOM1_CH1_P33_9, SERVO_FREQ, mid);     //舵机PWM初始化
    servo_init();                                   // 舵机初始化
    balance_pid_init();                             //舵机PID平衡调节


    balance_control_set_enable(0U);                 //关闭舵机平衡控制，防止干扰IMU校准




    imu_all_init();                                  // IMU硬件初始化 + 四元数初始化
    //imu_calibrate_gyro();                            //IMU精确校准耗时60S

    imu_calibrate_gyro_temp();                     // 临时零偏值仅适合快速联调，不适合作为精度验证基准


    pit_ms_init(CCU60_CH1, IMU_PERIOD_MS);          // 定时中断初始化->5MS IMU数据采集
    zero_compensation();                            //机械零位捕获，在平衡控制关闭状态下完成



    balance_control_set_enable(1U);                  //重启舵机平衡控制




    scheduler_init();                               // 调度初始化
    

    cpu_wait_event_ready();             // 等待事件就绪

    led_test_all();                     //LED 测试->初始化完成标志


    printf("Init OK\r\n");

    while (1)
    {
        imu_proc();       // 事件驱动，优先级最高，放在while，减轻中断压力
        scheduler_run();  // 调度运行
    }
}


