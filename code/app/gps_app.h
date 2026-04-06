#ifndef GPS_APP_H_
#define GPS_APP_H_


#include "zf_common_headfile.h"

extern double g_gps_latitude_display;
extern double g_gps_longitude_display;

void gps_init(void);

void gps_task(void);

#endif

