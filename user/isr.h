/*
 * 文件: isr.h
 * 功能: 中断模块头文件，统一包含 ISR 相关公共依赖
 * 作者: 闫锦
 * 日期: 2026-03-31
 */

#ifndef _isr_h
#define _isr_h

#include "zf_common_headfile.h"

extern volatile uint32_t g_ctrl_5ms_last_us;
extern volatile uint32_t g_ctrl_5ms_max_us;
extern volatile uint32_t g_ctrl_5ms_min_us;
extern volatile uint32_t g_ctrl_5ms_run_count;

void isr_diag_reset(void);











#endif

