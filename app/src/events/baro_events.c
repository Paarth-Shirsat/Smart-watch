#include "baro_events.h"

ZBUS_CHAN_DEFINE(baro_data_chan,
    struct baro_data_event, NULL, NULL,
    ZBUS_OBSERVERS_EMPTY,
    ZBUS_MSG_INIT(0));