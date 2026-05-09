#include "imu_events.h"

ZBUS_CHAN_DEFINE(imu_data_chan,
    struct imu_data_event, NULL, NULL,
    ZBUS_OBSERVERS_EMPTY,
    ZBUS_MSG_INIT(0));

ZBUS_CHAN_DEFINE(imu_gesture_chan,
    struct imu_gesture_event, NULL, NULL,
    ZBUS_OBSERVERS_EMPTY,
    ZBUS_MSG_INIT(0));