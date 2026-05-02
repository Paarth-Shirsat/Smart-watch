#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "max30102.h"

static const char *TAG = "MAIN";

/* Sample buffer — holds up to one full FIFO worth of readings */
static max30102_sample_t s_samples[MAX30102_FIFO_DEPTH];

void app_main(void)
{
    max30102_config_t cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_pin  = 21,
        .scl_pin  = 22,
        .i2c_freq = MAX30102_I2C_FREQ_HZ,
        .led_pa   = MAX30102_LED_PA_7MA,
    };

    esp_err_t err = max30102_init(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "MAX30102 init failed: %s", esp_err_to_name(err));
        return;
    }

    uint32_t total = 0;
    while (1) {
        size_t count = 0;
        err = max30102_read_fifo(cfg.i2c_port, MAX30102_I2C_ADDR,
                                 s_samples, MAX30102_FIFO_DEPTH, &count);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "FIFO read error: %s", esp_err_to_name(err));
        } else {
            for (size_t i = 0; i < count; i++) {
                total++;
                ESP_LOGI(TAG, "Sample #%lu  Red=%lu  IR=%lu",
                         (unsigned long)total,
                         (unsigned long)s_samples[i].red,
                         (unsigned long)s_samples[i].ir);
            }
        }

        /* At 100 SPS with 4× averaging the effective rate is 25 SPS (40 ms).
         * Polling every 100 ms collects ~2–3 samples per iteration.           */
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    max30102_deinit(cfg.i2c_port);
}
