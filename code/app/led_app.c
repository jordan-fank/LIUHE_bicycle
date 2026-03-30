#include "led_app.h"




void led_test_all(void)
{
    uint8 i;
    

    led_all_set(LED_ON);
    system_delay_ms(500);

    led_all_set(LED_OFF);
    system_delay_ms(200);


    for (i = 1; i <= 4; i++)
    {
        led_set(i, LED_ON);
        system_delay_ms(200);
        led_set(i, LED_OFF);
        system_delay_ms(100);
    }
    

    led_all_set(LED_ON);
    system_delay_ms(300);

    led_all_set(LED_OFF);
}



void led_task(void)
{

}

