#include "key_driver.h"

static const gpio_pin_enum key_pins[KEY_NUMBER] = {P20_6, P20_7, P11_2, P11_3};

typedef enum {
    KS_IDLE = 0,
    KS_PRESSED,
    KS_WAIT_DOUBLE,
    KS_DOUBLE_PRESSED,
    KS_LONG_HELD,       // 长按持续状态：按键仍按下，等待松手
} key_state_t;

typedef struct {
    key_state_t  state;
    uint32_t     press_time;
    uint32_t     release_time;
    key_event_t  event;
} key_ctx_t;

static key_ctx_t key_ctx[KEY_NUMBER];


uint8_t key_read(key_index_enum id)
{
    return (gpio_get_level(key_pins[id]) == 0) ? 1u : 0u;  // 低电平=按下
}



void key_app_init(void)
{
    key_init(10);   // 逐飞库初始化 GPIO，传入扫描周期 10ms
    for (uint8_t i = 0; i < KEY_NUMBER; i++) {
        key_ctx[i].state = KS_IDLE;
        key_ctx[i].event = KEY_EVENT_NONE;
    }
}

void key_scan(void)
{
    uint32_t now = system_getval_ms();

    for (uint8_t i = 0; i < KEY_NUMBER; i++) {
        key_ctx_t *k   = &key_ctx[i];
        uint8_t    lvl = key_read((key_index_enum)i);

        switch (k->state) {
            case KS_IDLE:
                if (lvl) {
                    k->press_time = now;
                    k->state = KS_PRESSED;
                }
                break;

            case KS_PRESSED:
                if (!lvl) {
                    if ((now - k->press_time) < KEY_DEBOUNCE_MS) {
                        k->state = KS_IDLE;
                    } else {
                        k->release_time = now;
                        k->state = KS_WAIT_DOUBLE;
                    }
                } else if ((now - k->press_time) >= KEY_LONG_PRESS_MS) {
                    k->event = KEY_EVENT_LONG;
                    k->state = KS_LONG_HELD;  // 进入长按持续状态
                }
                break;

            case KS_LONG_HELD:
                // 按键仍按下：key_app 通过 key_read() 轮询持续触发
                // 松手后回到 IDLE，不产生 SHORT/DOUBLE 事件
                if (!lvl) {
                    k->state = KS_IDLE;
                }
                break;

            case KS_WAIT_DOUBLE:
                if (lvl) {
                    k->press_time = now;
                    k->state = KS_DOUBLE_PRESSED;
                } else if ((now - k->release_time) >= KEY_DOUBLE_CLICK_MS) {
                    k->event = KEY_EVENT_SHORT;
                    k->state = KS_IDLE;
                }
                break;

            case KS_DOUBLE_PRESSED:
                if (!lvl) {
                    k->event = KEY_EVENT_DOUBLE;
                    k->state = KS_IDLE;
                }
                break;

            default:
                k->state = KS_IDLE;
                break;
        }
    }
}

key_event_t key_get_event(key_index_enum id)
{
    key_event_t ev = key_ctx[id].event;
    key_ctx[id].event = KEY_EVENT_NONE;
    return ev;
}
