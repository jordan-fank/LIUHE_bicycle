#ifndef LED_DRIVER_H_
#define LED_DRIVER_H_


#include "zf_common_headfile.h"

#define LED_ON   GPIO_LOW
#define LED_OFF  GPIO_HIGH


void led_init(void);

void led_set(uint8 led_num, uint8 state);
void led_all_set(uint8 state);



#endif

