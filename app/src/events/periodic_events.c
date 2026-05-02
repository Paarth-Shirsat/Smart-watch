#include "periodic_events.h"

ZBUS_CHAN_DEFINE(periodic_event_100ms_chan,
    struct periodic_event, NULL, NULL,
    ZBUS_OBSERVERS_EMPTY,
    ZBUS_MSG_INIT(.tick = 0));

ZBUS_CHAN_DEFINE(periodic_event_1s_chan,
    struct periodic_event, NULL, NULL,
    ZBUS_OBSERVERS_EMPTY,
    ZBUS_MSG_INIT(.tick = 0));

static void timer_handler(struct k_timer *t){
    static uint8_t count = 0;
    static struct periodic_event e = {.tick = 0};

    e.tick++;
    zbus_chan_pub(&periodic_event_100ms_chan, &e, K_NO_WAIT);

    if (++count >= 10) {
        count = 0;
        zbus_chan_pub(&periodic_event_1s_chan, &e, K_NO_WAIT);
    }
}

K_TIMER_DEFINE(periodic_timer, timer_handler, NULL);

static int periodic_init(void){
    k_timer_start(&periodic_timer, K_MSEC(100), K_MSEC(100));
    return 0;
}

SYS_INIT(periodic_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);