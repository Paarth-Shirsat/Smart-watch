#ifndef BARO_EVENTS
#define BARO_EVENTS

#include <zephyr/zbus/zbus.h>

struct baro_data_event {
    float pressure_hpa;
    float altitude_m;
    float temperature_c;
};

ZBUS_CHAN_DECLARE(baro_data_chan);

#endif