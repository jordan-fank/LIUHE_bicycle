/**
 * @file ips_app.h
 * @brief IPS 应用层接口
 *
 * 外部调用顺序：
 *   1. ips_app_init()          —— 系统初始化阶段调用一次
 *   2. ips_app_task()          —— 加入调度器，建议 200ms 刷新（仅刷新实时数值行）
 *
 * 按键回调（在 key_app.c 中调用）：
 *   ips_app_next_page()        —— 切换到下一页（KEY3）
 *   ips_app_sel_up()           —— 向上选参数（KEY5）
 *   ips_app_sel_down()         —— 向下选参数（KEY4）
 *   ips_app_adjust(+1/-1)      —— 增/减当前选中参数（KEY2/KEY1）
 */

#ifndef IPS_APP_H
#define IPS_APP_H

#include "zf_common_headfile.h"
#include "ips_app_config.h"

void  ips_app_init(void);
void  ips_app_task(void);

void  ips_app_next_page(void);
void  ips_app_prev_page(void);
void  ips_app_sel_up(void);
void  ips_app_sel_down(void);
void  ips_app_adjust(int8 direction);

#endif /* IPS_APP_H */
