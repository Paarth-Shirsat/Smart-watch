#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/zbus/zbus.h>
#include "events/periodic_events.h"
#include "events/imu_events.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

static void on_imu_data(const struct zbus_channel *chan)
{
    const struct imu_data_event *e = zbus_chan_const_msg(chan);
    LOG_INF("IMU | Accel: %.2f %.2f %.2f | Gyro: %.2f %.2f %.2f",
        (double)e->accel_x, (double)e->accel_y, (double)e->accel_z,
        (double)e->gyro_x,  (double)e->gyro_y,  (double)e->gyro_z);
}

ZBUS_LISTENER_DEFINE(main_imu_listener, on_imu_data);
ZBUS_CHAN_ADD_OBS(imu_data_chan, main_imu_listener, 0);

int main(void)
{
    LOG_INF("Zephyr RTOS Booting on Custom Smartwatch...");
    return 0;
}