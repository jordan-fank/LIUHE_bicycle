
/*
 * 文件: cpu0_main.h
 * 功能: 声明 CPU0 主入口相关类型和外部对象
 * 作者: 闫锦
 * 日期: 2026-03-31
 */

#ifndef CPU0_MAIN_H
#define CPU0_MAIN_H


#include "zf_common_headfile.h"

#include "Cpu/Std/Ifx_Types.h"


typedef struct
{
    float32 sysFreq;                /**< \brief Actual SPB frequency */
    float32 cpuFreq;                /**< \brief Actual CPU frequency */
    float32 pllFreq;                /**< \brief Actual PLL frequency */
    float32 stmFreq;                /**< \brief Actual STM frequency */
} AppInfo;

/** \brief Application information */
typedef struct
{
    AppInfo info;                               /**< \brief Info object */
} App_Cpu0;



IFX_EXTERN App_Cpu0 g_AppCpu0;

#endif
