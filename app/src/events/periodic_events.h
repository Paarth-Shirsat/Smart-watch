#ifndef PERIODIC_EVENTS
#define PERIODIC_EVENTS

#include <zephyr/zbus/zbus.h>

struct periodic_event { uint8_t tick; };

ZBUS_CHAN_DECLARE(periodic_event_100ms_chan);
ZBUS_CHAN_DECLARE(periodic_event_1s_chan);

#endif