#include "led_driver.h"



#define LED1                    (P20_9)
#define LED2                    (P20_8)
#define LED3                    (P21_5)
#define LED4                    (P21_4)



//LED初始化
void led_init(void)
{
    gpio_init(LED1, GPO, GPIO_HIGH, GPO_PUSH_PULL);       
    gpio_init(LED2, GPO, GPIO_HIGH, GPO_PUSH_PULL);         
    gpio_init(LED3, GPO, GPIO_HIGH, GPO_PUSH_PULL);         
    gpio_init(LED4, GPO, GPIO_HIGH, GPO_PUSH_PULL);         
    
}


//设置一个LED的状态
void led_set(uint8 led_num, uint8 state)
{
    if (led_num > 4 || led_num < 1 ) return;
    
    switch (led_num)
    {
        case 1:
            gpio_set_level(LED1, state);
            break;
        case 2:
            gpio_set_level(LED2, state);
            break;
        case 3:
            gpio_set_level(LED3, state);
            break;
        case 4:
            gpio_set_level(LED4, state);
            break;
        default:
            break;
    }
}

//统一设置LED状态
void led_all_set(uint8 state)
{
    gpio_set_level(LED1, state);
    gpio_set_level(LED2, state);
    gpio_set_level(LED3, state);
    gpio_set_level(LED4, state);
}


