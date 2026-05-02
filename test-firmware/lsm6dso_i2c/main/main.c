#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "lsm6dso.h"

static const char *TAG = "MAIN";

void app_main(void)
{
    lsm6dso_config_t cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_pin  = 21,
        .scl_pin  = 22,
        .i2c_freq = LSM6DSO_I2C_FREQ_HZ,
    };

    esp_err_t err = lsm6dso_init(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "LSM6DSO init failed: %s", esp_err_to_name(err));
        return;
    }

    uint32_t sample = 0;
    lsm6dso_raw_t accel, gyro;

    while (1) {
        err = lsm6dso_read_all(cfg.i2c_port, &accel, &gyro);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Read error: %s", esp_err_to_name(err));
        } else {
            sample++;
            /* Sensitivity: accel = 0.122 mg/LSB (±4 g), gyro = 70 mdps/LSB (±2000 dps) */
            ESP_LOGI(TAG, "Sample #%lu  "
                          "Accel[mg]: X=%d  Y=%d  Z=%d  |  "
                          "Gyro[mdps]: X=%d  Y=%d  Z=%d",
                     (unsigned long)sample,
                     (int)((int32_t)accel.x * 122 / 1000),
                     (int)((int32_t)accel.y * 122 / 1000),
                     (int)((int32_t)accel.z * 122 / 1000),
                     (int)((int32_t)gyro.x  * 70),
                     (int)((int32_t)gyro.y  * 70),
                     (int)((int32_t)gyro.z  * 70));
        }

        /* At 104 Hz ODR, poll every 100 ms (~10 new samples per iteration).
         * With BDU enabled each read returns a coherent X/Y/Z snapshot.        */
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    lsm6dso_deinit(cfg.i2c_port);
}
