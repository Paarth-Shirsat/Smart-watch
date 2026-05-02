#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "bmp388.h"

static const char *TAG = "MAIN";

void app_main(void)
{
    bmp388_config_t cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_pin  = 21,
        .scl_pin  = 22,
        .i2c_freq = BMP388_I2C_FREQ_HZ,
        .i2c_addr = BMP388_I2C_ADDR_DEFAULT,
    };

    esp_err_t err = bmp388_init(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "BMP388 init failed: %s", esp_err_to_name(err));
        return;
    }

    int reading = 0;
    while (1) {
        bmp388_data_t data;
        err = bmp388_read(cfg.i2c_port, cfg.i2c_addr, &data);
        if (err == ESP_OK) {
            reading++;
            ESP_LOGI(TAG, "Reading #%d  Temp=%.2f°C  Press=%.2fPa  Alt=%.2fm",
                     reading,
                     data.temperature,
                     data.pressure,
                     data.altitude);
        } else if (err == ESP_ERR_TIMEOUT) {
            ESP_LOGW(TAG, "Data not ready, retrying...");
        } else {
            ESP_LOGE(TAG, "Read error: %s", esp_err_to_name(err));
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    bmp388_deinit(cfg.i2c_port);
}
