#include "gps_app.h"


void gps_init(void)
{
    gnss_init(TAU1201);
}


void gps_task(void)
{
    if(gnss_flag)
        {
            gnss_flag = 0;

            gnss_data_parse();           //开始解析数据


        }
}
