#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "max_m10s.h"

static const char *TAG = "APP";

/* ── Pin assignment — update to match your schematic ─────────────────────── */
#define GNSS_I2C_PORT   I2C_NUM_0
#define GNSS_SDA_PIN    21
#define GNSS_SCL_PIN    22

void app_main(void)
{
    max_m10s_config_t cfg = {
        .i2c_port    = GNSS_I2C_PORT,
        .sda_pin     = GNSS_SDA_PIN,
        .scl_pin     = GNSS_SCL_PIN,
        .i2c_freq_hz = MAX_M10S_I2C_FREQ_HZ,
        .i2c_addr    = MAX_M10S_I2C_ADDR_DEFAULT,
    };

    esp_err_t ret = max_m10s_init(&cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Init failed: %s", esp_err_to_name(ret));
        return;
    }

    max_m10s_data_t gps = {0};
    uint32_t fix_count = 0;

    while (1) {
        ret = max_m10s_read_nav_pvt(&gps);

        if (ret == ESP_OK) {
            if (gps.fix_type >= GPS_FIX_2D) {
                fix_count++;
                ESP_LOGI(TAG, "Fix #%lu", (unsigned long)fix_count);
                ESP_LOGI(TAG,
                         "Fix=%d Sats=%d  Lat=%.7f  Lon=%.7f  Alt=%.1fm  "
                         "Speed=%.2fm/s  Hdg=%.1fdeg  hAcc=%.1fm  vAcc=%.1fm",
                         gps.fix_type, gps.satellites,
                         gps.latitude, gps.longitude, gps.altitude_m,
                         gps.speed_mps, gps.heading_deg,
                         gps.h_acc_m, gps.v_acc_m);

                if (gps.date_valid && gps.time_valid) {
                    ESP_LOGI(TAG, "UTC %04d-%02d-%02d %02d:%02d:%02d",
                             gps.year, gps.month, gps.day,
                             gps.hour, gps.minute, gps.second);
                }
            } else {
                ESP_LOGW(TAG, "Acquiring fix (type=%d sats=%d)",
                         gps.fix_type, gps.satellites);
            }
        } else if (ret == ESP_ERR_TIMEOUT) {
            ESP_LOGW(TAG, "No response from MAX-M10S");
        } else {
            ESP_LOGE(TAG, "Read error: %s", esp_err_to_name(ret));
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
