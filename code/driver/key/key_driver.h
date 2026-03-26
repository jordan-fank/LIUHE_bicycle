#ifndef KEY_DRIVER_H_
#define KEY_DRIVER_H_

#include "zf_common_headfile.h"


#define KEY_DEBOUNCE_MS      20u
#define KEY_LONG_PRESS_MS    800u
#define KEY_DOUBLE_CLICK_MS  300u

typedef enum {
    KEY_EVENT_NONE = 0,
    KEY_EVENT_SHORT,
    KEY_EVENT_DOUBLE,
    KEY_EVENT_LONG,
} key_event_t;

void        key_app_init(void);
void        key_scan(void);
key_event_t key_get_event(key_index_enum id);

uint8_t key_read(key_index_enum id);

#endif
