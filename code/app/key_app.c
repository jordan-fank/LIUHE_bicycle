#include "key_app.h"
#include "ips_app.h"
/* [新增 科目1] */
#include "subject1_app.h"
/* [新增 科目2] */
#include "subject2_app.h"
/* [新增 科目3] */
#include "subject3_app.h"
/* [新增 Flash持久化] */
#include "config_flash.h"

/**
 * @file key_app.c
 * @brief 按键应用层 - 4个按键控制IPS显示
 *
 * 按键映射：
 *   KEY1 短按/长按：减小当前选中参数（长按连续减）
 *   KEY1 双击：[新增 Flash持久化] 保存全部配置到 Flash
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

/* [新增] 活跃科目选择：1=科目1, 2=科目2, 3=科目3
   KEY2 双击循环切换，决定 KEY3/KEY4 长按分发到哪个科目 */
uint8_t g_active_subject = 1u;

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
        case KEY_EVENT_DOUBLE:
            /* [新增 Flash持久化] KEY1 双击 → 保存全部配置到 Flash
               适用场景：WiFi/IPS 调参完毕后快速持久化 */
            config_flash_save();
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
        case KEY_EVENT_DOUBLE:
            /* [新增] KEY2 双击 → 循环切换活跃科目 1→2→3→1...
               切换后 KEY3/KEY4 长按的行为跟随变化 */
            g_active_subject = (g_active_subject == 1u) ? 2u :
                               (g_active_subject == 2u) ? 3u : 1u;
            printf("[KEY] Active subject: %u\r\n", (unsigned)g_active_subject);
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
    //       [新增 科目1] 长按：启动科目1（IDLE时）或紧急停止（运行中）
    //==========================================================================
    switch (key_get_event(KEY_3))
    {
        case KEY_EVENT_SHORT:
            ips_app_next_page();
            break;
        case KEY_EVENT_DOUBLE:
            ips_app_prev_page();
            break;
        case KEY_EVENT_LONG:
            /* [新增] 根据活跃科目分发启动/停止 */
            if (g_active_subject == 1u)
            {
                if (g_subject1_state == SUBJ1_STATE_IDLE || g_subject1_state == SUBJ1_STATE_DONE)
                    subject1_start();
                else
                    subject1_stop();
            }
            else if (g_active_subject == 2u)
            {
                if (g_subject2_state == SUBJ2_STATE_IDLE || g_subject2_state == SUBJ2_STATE_DONE)
                    subject2_start();
                else
                    subject2_stop();
            }
            else
            {
                if (g_subject3_state == SUBJ3_STATE_IDLE || g_subject3_state == SUBJ3_STATE_DONE)
                    subject3_start();
                else
                    subject3_stop();
            }
            break;
        default:
            break;
    }

    //==========================================================================
    // KEY4: 向下选参数（短按），向上选参数（双击）
    //       [新增 科目1] 长按：勘线（第1次=记录起点，第2次=记录路点+自动保存）
    //==========================================================================
    switch (key_get_event(KEY_4))
    {
        case KEY_EVENT_SHORT:
            ips_app_sel_down();
            break;
        case KEY_EVENT_DOUBLE:
            ips_app_sel_up();
            break;
        case KEY_EVENT_LONG:
            /* [新增] 根据活跃科目分发勘线
               科目2 无需勘线（段序列由参数自动计算）*/
            if (g_active_subject == 1u)
                subject1_survey();
            else if (g_active_subject == 2u)
                printf("[KEY] Subject 2: no survey needed. Just KEY3 to start.\r\n");
            else
                subject3_survey();
            break;
        default:
            break;
    }

}
