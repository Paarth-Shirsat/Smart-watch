#ifndef IMU_EVENTS
#define IMU_EVENTS

#include <zephyr/zbus/zbus.h>

struct imu_data_event{
    float accel_x, accel_y, accel_z;
    float gyro_x,  gyro_y,  gyro_z;
};

struct imu_gesture_event{
    bool tap_detected;
    bool double_tap_detected;
    uint8_t step_count_delta;
};

ZBUS_CHAN_DECLARE(imu_data_chan);
ZBUS_CHAN_DECLARE(imu_gesture_chan);

#endif