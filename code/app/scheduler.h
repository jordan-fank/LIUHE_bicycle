#ifndef CODE_APP_SCHEDULER_H_
#define CODE_APP_SCHEDULER_H_

#include "zf_common_headfile.h"

//// 头文件声明结构体，让所有源文件都知道什么样的结构体
//typedef struct
//{
//    uint32_t last_us;
//    uint32_t max_us;
//    uint32_t min_us;
//    uint32_t run_count;
//    uint64_t total_us;
//} scheduler_stat_t;
//
//extern scheduler_stat_t g_scheduler_stat;

/* ==================== TEST ONLY: 调度任务级耗时统计 ==================== */
//typedef struct
//{
//    uint32_t last_us;
//    uint32_t max_us;
//    uint32_t run_count;
//} scheduler_task_diag_t;
//
//extern uint8_t g_scheduler_task_diag_count;
//extern scheduler_task_diag_t g_scheduler_task_diag[];
//extern const char * const g_scheduler_task_diag_name[];
/* ==================== TEST ONLY: 调度任务级耗时统计 ==================== */

void scheduler_init(void);
void scheduler_run(void);
//void scheduler_stat_reset(void);

#endif
