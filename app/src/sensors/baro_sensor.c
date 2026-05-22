#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/zbus/zbus.h>
#include <math.h>
#include "events/baro_events.h"
#include "events/periodic_events.h"

LOG_MODULE_REGISTER(baro_sensor, LOG_LEVEL_INF);

static const struct device *baro_dev = DEVICE_DT_GET(DT_NODELABEL(bmp388));

static float pressure_to_altitude(float pressure_hpa)
{
    const float sea_level_hpa = 1013.25f;
    return 44330.0f * (1.0f - powf(pressure_hpa / sea_level_hpa, 0.1903f));
}

static void baro_work_handler(struct k_work *work)
{
    struct sensor_value press, temp;

    if (sensor_sample_fetch(baro_dev) < 0) {
        LOG_ERR("BMP388 fetch failed");
        return;
    }

    sensor_channel_get(baro_dev, SENSOR_CHAN_PRESS, &press);
    sensor_channel_get(baro_dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);

    float pressure_hpa  = sensor_value_to_float(&press);
    float temperature_c = sensor_value_to_float(&temp);
    float altitude_m    = pressure_to_altitude(pressure_hpa);

    struct baro_data_event evt = {
        .pressure_hpa  = pressure_hpa,
        .altitude_m    = altitude_m,
        .temperature_c = temperature_c,
    };

    zbus_chan_pub(&baro_data_chan, &evt, K_MSEC(10));

    LOG_DBG("Pressure: %.2f hPa | Altitude: %.1f m | Temp: %.2f C",
        (double)pressure_hpa, (double)altitude_m, (double)temperature_c);
}

static K_WORK_DEFINE(baro_work, baro_work_handler);

static void on_1s_tick(const struct zbus_channel *chan)
{
    if (!device_is_ready(baro_dev)) {
        LOG_ERR("BMP388 not ready");
        return;
    }
    k_work_submit(&baro_work);
}

ZBUS_LISTENER_DEFINE(baro_1s_listener, on_1s_tick);
ZBUS_CHAN_ADD_OBS(periodic_event_1s_chan, baro_1s_listener, 0);

static int baro_sensor_init(void)
{
    if (!device_is_ready(baro_dev)) {
        LOG_ERR("BMP388 not ready — check I2C wiring, CSB=3V3, SDO=GND");
        return -ENODEV;
    }
    LOG_INF("BMP388 barometer ready");
    return 0;
}
SYS_INIT(baro_sensor_init, APPLICATION, 98);