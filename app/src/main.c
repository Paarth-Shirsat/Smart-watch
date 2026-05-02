#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/zbus/zbus.h>
#include "events/periodic_events.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

static void on_1s_tick(const struct zbus_channel *chan){
    const struct periodic_event *e = zbus_chan_const_msg(chan);
    LOG_INF("1s tick #%d", e->tick);
}

ZBUS_LISTENER_DEFINE(main_1s_listener, on_1s_tick);
ZBUS_CHAN_ADD_OBS(periodic_event_1s_chan, main_1s_listener, 0);

int main(void){
    LOG_INF("Zephyr RTOS Booting on Custom Smartwatch...");
    return 0;
}