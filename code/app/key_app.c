#include "key_app.h"
#include "ips_app.h"

/**
 * @file key_app.c
 * @brief 按键应用层 - 4个按键控制IPS显示
 *
 * 按键映射：
 *   KEY1 短按/长按：减小当前选中参数（长按连续减）
 *   KEY2 短按/长按：增大当前选中参数（长按连续增）
 *   KEY3 短按：切换下一页
 *   KEY4 短按：向下选参数
 *   KEY4 双击：向上选参数
 *
 * 长按连续增减原理：
 *   key_driver 检测到长按后设置 KEY_EVENT_LONG（仅触发一次），
 *   此后按键物理状态持续按下，通过 key_read() 轮询实现连续触发，
 *   每 10ms（key_task 周期）调用一次 adjust，实现连续增减。
 */

// 长按连续触发的防抖计数（避免长按识别瞬间和连续触发重叠）
static uint8_t s_key1_hold = 0;
static uint8_t s_key2_hold = 0;

void key_task(void)
{
    //==========================================================================
    // KEY1: 减小参数值（短按单次，长按连续）
    //==========================================================================
    switch (key_get_event(KEY_1))
    {
        case KEY_EVENT_SHORT:
            ips_app_adjust(-1);
            s_key1_hold = 0;
            break;
        case KEY_EVENT_LONG:
            // 长按识别后开始连续模式
            s_key1_hold = 1;
            break;
        default:
            break;
    }
    // 长按持续按下时连续触发（key_read 检测物理电平，按住=1）
    if (s_key1_hold)
    {
        if (key_read(KEY_1))
            ips_app_adjust(-1);
        else
            s_key1_hold = 0;  // 松手则退出连续模式
    }

    //==========================================================================
    // KEY2: 增大参数值（短按单次，长按连续）
    //==========================================================================
    switch (key_get_event(KEY_2))
    {
        case KEY_EVENT_SHORT:
            ips_app_adjust(1);
            s_key2_hold = 0;
            break;
        case KEY_EVENT_LONG:
            s_key2_hold = 1;
            break;
        default:
            break;
    }
    if (s_key2_hold)
    {
        if (key_read(KEY_2))
            ips_app_adjust(1);
        else
            s_key2_hold = 0;
    }

    //==========================================================================
    // KEY3: 切换页面（短按下一页，双击上一页）
    //==========================================================================
    switch (key_get_event(KEY_3))
    {
        case KEY_EVENT_SHORT:
            ips_app_next_page();
            break;
        case KEY_EVENT_DOUBLE:
            ips_app_prev_page();
            break;
        default:
            break;
    }

    //==========================================================================
    // KEY4: 向下选参数（短按），向上选参数（双击）
    //==========================================================================
    switch (key_get_event(KEY_4))
    {
        case KEY_EVENT_SHORT:
            ips_app_sel_down();
            break;
        case KEY_EVENT_DOUBLE:
            ips_app_sel_up();
            break;
        default:
            break;
    }
}
