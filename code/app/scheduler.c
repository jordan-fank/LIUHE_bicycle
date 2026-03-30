#include "scheduler.h"

//scheduler_stat_t g_scheduler_stat = {
//    .min_us = 0xFFFFFFFFu
//};

uint8_t task_num;

typedef struct
{
    void (*task_func)(void);
    uint32_t rate_ms;
    uint32_t last_run;
} task_t;

static task_t scheduler_task[] =
{
        {key_scan,       10, 0},   // 按键扫描任务，10ms
        {key_task,       10, 0},   // 按键处理任务，10ms
        {ips_app_task,   200, 0},  // 屏幕刷新任务，200ms（只刷新只读数值，按键操作立即响应不依赖此周期）
        {motor_calculate,10, 0},
        {gps_task,  10, 0},
        // {servo_test,     10, 0},

};

/* ==================== TEST ONLY: 调度任务级耗时统计 ==================== */
//scheduler_task_diag_t g_scheduler_task_diag[sizeof(scheduler_task) / sizeof(task_t)];
//const char * const g_scheduler_task_diag_name[] =
//{
//    "key_scan",
//    "key_task",
//    "ips_app_task",
//    "imu_test",
//};
//uint8_t g_scheduler_task_diag_count = sizeof(scheduler_task) / sizeof(task_t);
/* ==================== TEST ONLY: 调度任务级耗时统计 ==================== */

//static uint32_t scheduler_elapsed_us(uint32_t start_tick)
//{
//    /* 先对原始10ns tick做无符号差分，再换算成us。
//       这样 system_getval() 即使32位回绕，单次调度耗时统计仍然正确。 */
//    return (system_getval() - start_tick) / 100U;
//}
//
//void scheduler_stat_reset(void)
//{
//    uint8_t i;
//
//    g_scheduler_stat.last_us = 0;
//    g_scheduler_stat.max_us = 0;
//    g_scheduler_stat.min_us = 0xFFFFFFFFu;
//    g_scheduler_stat.run_count = 0;
//    g_scheduler_stat.total_us = 0;
//
//    /* ==================== TEST ONLY: 调度任务级耗时统计复位 ==================== */
//    for (i = 0; i < g_scheduler_task_diag_count; i++)
//    {
//        g_scheduler_task_diag[i].last_us = 0;
//        g_scheduler_task_diag[i].max_us = 0;
//        g_scheduler_task_diag[i].run_count = 0;
//    }
//    /* ==================== TEST ONLY: 调度任务级耗时统计复位 ==================== */
//}

void scheduler_init(void)
{
    task_num = sizeof(scheduler_task) / sizeof(task_t);
    //scheduler_stat_reset();
}

void scheduler_run(void)
{
//    uint32_t start_tick = system_getval();
    uint32_t now_time = system_getval_ms();

    for (uint8_t i = 0; i < task_num; i++)
    {
        // 首次运行时，last_run=0，需要特殊处理
        if (scheduler_task[i].last_run == 0)
        {
            scheduler_task[i].last_run = now_time;
            continue;  // 跳过第一次，避免立即执行
        }

        if ((now_time - scheduler_task[i].last_run) >= scheduler_task[i].rate_ms)
        {
//            uint32_t task_start_tick;
//            uint32_t task_elapsed_us;

            scheduler_task[i].last_run = now_time;

            scheduler_task[i].task_func();

//            task_start_tick = system_getval();
//            task_elapsed_us = scheduler_elapsed_us(task_start_tick);

            /* ==================== TEST ONLY: 单任务耗时统计 ==================== */
//            g_scheduler_task_diag[i].last_us = task_elapsed_us;
//            if (task_elapsed_us > g_scheduler_task_diag[i].max_us)
//                g_scheduler_task_diag[i].max_us = task_elapsed_us;
//            g_scheduler_task_diag[i].run_count++;
            /* ==================== TEST ONLY: 单任务耗时统计 ==================== */
        }
    }

    /*-------------用于测试整个调度表执行的时间以及最大执行时间---------*/
//    {
//        uint32_t elapsed_us = scheduler_elapsed_us(start_tick);
//
//        g_scheduler_stat.last_us = elapsed_us;
//        if (elapsed_us > g_scheduler_stat.max_us)
//            g_scheduler_stat.max_us = elapsed_us;
//        if (elapsed_us < g_scheduler_stat.min_us)
//            g_scheduler_stat.min_us = elapsed_us;
//
//        g_scheduler_stat.run_count++;
//        g_scheduler_stat.total_us += elapsed_us;
//    }

    /*-------------用于测试整个调度表执行的时间以及最大执行时间----------------*/
}
