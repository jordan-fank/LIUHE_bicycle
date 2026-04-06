#include "gps_app.h"

double g_gps_latitude_display  = 0.0;
double g_gps_longitude_display = 0.0;

static void gps_update_display_snapshot(void)
{
    uint32 irq_state = interrupt_global_disable();
    g_gps_latitude_display  = gnss.latitude;
    g_gps_longitude_display = gnss.longitude;
    interrupt_global_enable(irq_state);
}

void gps_init(void)
{
    gnss_init(TAU1201);
    gps_update_display_snapshot();
}


void gps_task(void)
{
    if(gnss_flag)
    {
        gnss_flag = 0;
        gnss_data_parse();           //开始解析数据
    }

    /* 将 ISR 更新的 GNSS 原始数据收口到主循环快照，避免 UI 直接读取 double。 */
    gps_update_display_snapshot();
}
