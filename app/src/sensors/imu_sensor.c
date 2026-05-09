#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/zbus/zbus.h>
#include "events/imu_events.h"
#include "events/periodic_events.h"

LOG_MODULE_REGISTER(imu_sensor, LOG_LEVEL_INF);

static const struct device *imu_dev = DEVICE_DT_GET(DT_NODELABEL(lsm6dso));

static void imu_read_and_publish(void)
{
    struct sensor_value accel[3], gyro[3];

    if (sensor_sample_fetch(imu_dev) < 0) {
        LOG_ERR("IMU fetch failed");
        return;
    }

    sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_XYZ, accel);
    sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_XYZ,  gyro);

    struct imu_data_event evt = {
        .accel_x = sensor_value_to_float(&accel[0]),
        .accel_y = sensor_value_to_float(&accel[1]),
        .accel_z = sensor_value_to_float(&accel[2]),
        .gyro_x  = sensor_value_to_float(&gyro[0]),
        .gyro_y  = sensor_value_to_float(&gyro[1]),
        .gyro_z  = sensor_value_to_float(&gyro[2]),
    };

    zbus_chan_pub(&imu_data_chan, &evt, K_MSEC(10));

    LOG_DBG("Accel: %.2f %.2f %.2f | Gyro: %.2f %.2f %.2f",
        (double)evt.accel_x, (double)evt.accel_y, (double)evt.accel_z,
        (double)evt.gyro_x,  (double)evt.gyro_y,  (double)evt.gyro_z);
}

static void imu_work_handler(struct k_work *work)
{
    imu_read_and_publish();
}

static K_WORK_DEFINE(imu_work, imu_work_handler);

static void on_1s_tick(const struct zbus_channel *chan)
{
    if (!device_is_ready(imu_dev)) {
        LOG_ERR("IMU not ready");
        return;
    }
    k_work_submit(&imu_work);
}

ZBUS_LISTENER_DEFINE(imu_1s_listener, on_1s_tick);
ZBUS_CHAN_ADD_OBS(periodic_event_1s_chan, imu_1s_listener, 0);

static int imu_sensor_init(void)
{
    if (!device_is_ready(imu_dev)) {
        LOG_ERR("LSM6DSO not ready");
        return -ENODEV;
    }
    LOG_INF("LSM6DSO IMU ready");
    return 0;
}
SYS_INIT(imu_sensor_init, APPLICATION, 99);