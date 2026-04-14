#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"

#include "i2c_manager.h"
#include "bmp388.h"
#include "lsm6dso.h"
#include "max30102.h"
#include "max_m10s.h"

static const char *TAG = "INTEGRATION";

// ─── Sensor Handles ───────────────────────────────────────────
static bmp388_dev_t    barometer;
static lsm6dso_dev_t   imu;
static max30102_dev_t  pulse;
static max_m10s_dev_t  gps;

// ─── Sensor Integration Task ──────────────────────────────────
void sensor_integration_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Sensor Task started on Core %d", xPortGetCoreID());

    // Barometer data
    double         baro_temp = 0, baro_press = 0, baro_alt = 0;

    // IMU data
    lsm6dso_vec3_t accel = {0}, gyro = {0};
    float          imu_temp = 0, roll = 0, pitch = 0;

    // Pulse sensor data
    max30102_data_t pulse_data  = {0};
    bool   finger_was_absent    = true;
    uint8_t current_pa          = 0x1F;

    // GPS data
    max_m10s_data_t gps_data    = {0};
    bool   gps_valid            = false;
    uint32_t gps_poll_counter   = 0;    // poll GPS every ~1 s (50 × 20 ms loops)

    uint32_t loop_count = 0;

    while (1) {
        // ── 1. Barometer ──────────────────────────────────────
        if (bmp388_read_temperature_pressure(&barometer, &baro_temp, &baro_press) == ESP_OK) {
            baro_alt = bmp388_pressure_to_altitude(baro_press);
        }

        // ── 2. IMU ────────────────────────────────────────────
        if (lsm6dso_read_data(&imu, &accel, &gyro, &imu_temp) == ESP_OK) {
            lsm6dso_compute_tilt(&accel, &roll, &pitch);
        }

        // ── 3. Pulse Sensor ───────────────────────────────────
        if (max30102_read_fifo(&pulse, &pulse_data) == ESP_OK) {
            bool finger = max30102_finger_present(&pulse, pulse_data.ir_raw);
            if (!finger) {
                if (!finger_was_absent) {
                    ESP_LOGW(TAG, "PULSE | Finger removed");
                    max30102_sensor_reset(&pulse);
                    finger_was_absent = true;
                }
            } else if (finger_was_absent) {
                ESP_LOGI(TAG, "PULSE | Finger detected! Running AGC...");
                finger_was_absent = false;
                max30102_agc(&pulse, &current_pa);
                // Reset filters after AGC
                pulse.dc_red.w = pulse.dc_ir.w = 0.0f;
                pulse.settled = false; pulse.settle_count = 0;
                pulse.bpm_smooth = 0.0f; pulse.last_beat_us = 0;
                pulse.above_threshold = false;
                pulse.pp_ir.max_val = 1.0f; pulse.pp_ir.min_val = -1.0f;
                pulse.pp_ir.age = 0;
                memset(&pulse.ma_red, 0, sizeof(pulse.ma_red));
                memset(&pulse.ma_ir,  0, sizeof(pulse.ma_ir));
                i2c_manager_write_reg(pulse.i2c_addr, MAX30102_REG_FIFO_WR_PTR, 0x00);
                i2c_manager_write_reg(pulse.i2c_addr, MAX30102_REG_OVF_COUNTER,  0x00);
                i2c_manager_write_reg(pulse.i2c_addr, MAX30102_REG_FIFO_RD_PTR,  0x00);
                vTaskDelay(pdMS_TO_TICKS(50));
            }
        }

        // ── 4. GPS (poll every ~1 s) ──────────────────────────
        if (gps_poll_counter++ >= 50) {
            gps_poll_counter = 0;
            esp_err_t gps_ret = max_m10s_read_nav_pvt(&gps, &gps_data);
            gps_valid = (gps_ret == ESP_OK);
        }

        // ── 5. Combined Print every 5 loops (100 ms) ──────────
        if (loop_count % 5 == 0) {
            printf("\n════════════ SENSOR UPDATE ════════════\n");

            // Barometer
            printf("BARO | Temp: %.2f C | Press: %.4f atm | Alt: %.2f m\n",
                   baro_temp, baro_press / 101325.0, baro_alt);

            // IMU
            printf("IMU  | Accel(g): [%+.2f, %+.2f, %+.2f] | Tilt R:%.1f P:%.1f\n",
                   accel.x, accel.y, accel.z, roll, pitch);
            printf("IMU  | Gyro(dps): [%+.1f, %+.1f, %+.1f] | Temp: %.1f C\n",
                   gyro.x, gyro.y, gyro.z, imu_temp);

            // Pulse Sensor
            if (!max30102_finger_present(&pulse, pulse_data.ir_raw)) {
                printf("PULSE| No finger\n");
            } else if (!pulse.settled) {
                printf("PULSE| Settling... (%d/%d)\n",
                       pulse.settle_count, MAX30102_SETTLE_SAMPLES);
            } else {
                printf("PULSE| BPM: %.1f | SpO2: %.1f%% | P-P: %.1f\n",
                       pulse_data.bpm,
                       pulse_data.spo2 > 0.0f ? pulse_data.spo2 : 0.0f,
                       pulse_data.peak_to_peak);
            }

            // GPS
            if (!gps_valid) {
                printf("GPS  | Acquiring fix...\n");
            } else {
                static const char *fix_str[] = {
                    "None", "DR", "2D", "3D", "GNSS+DR", "Time"
                };
                const char *fix_name = (gps_data.fix_type <= 5)
                                       ? fix_str[gps_data.fix_type] : "?";
                printf("GPS  | Fix: %s | SVs: %d | Lat: %.6f  Lon: %.6f\n",
                       fix_name, gps_data.satellites,
                       gps_data.latitude, gps_data.longitude);
                printf("GPS  | Alt: %.1f m | Speed: %.2f m/s | HAcc: %.1f m\n",
                       gps_data.altitude_m, gps_data.speed_mps, gps_data.h_acc_m);
                if (gps_data.date_valid && gps_data.time_valid) {
                    printf("GPS  | UTC: %04d-%02d-%02d %02d:%02d:%02d\n",
                           gps_data.year, gps_data.month, gps_data.day,
                           gps_data.hour, gps_data.minute, gps_data.second);
                }
            }

            printf("═══════════════════════════════════════\n");
        }

        loop_count++;
        vTaskDelay(pdMS_TO_TICKS(20)); // 50 Hz main loop
    }
}

// ─── App Entry Point ──────────────────────────────────────────
void app_main(void)
{
    ESP_LOGI(TAG, "Initializing Integrated Sensor System (4 sensors)...");

    // 1. Shared I2C Bus
    ESP_ERROR_CHECK(i2c_manager_init());
    ESP_LOGI(TAG, "Shared I2C bus ready (GPIO 8 SDA / GPIO 9 SCL)");

    // 2. BMP388 Barometer
    ESP_ERROR_CHECK(bmp388_init(&barometer, BMP388_I2C_ADDR_SECONDARY));

    // 3. LSM6DSO IMU
    esp_err_t ret = lsm6dso_init(&imu, LSM6DSO_ADDR_LOW);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "LSM6DSO at 0x6A failed, trying 0x6B...");
        ESP_ERROR_CHECK(lsm6dso_init(&imu, LSM6DSO_ADDR_HIGH));
    }
    ESP_ERROR_CHECK(lsm6dso_calibrate(&imu));

    // 4. MAX30102 Pulse Sensor
    ESP_ERROR_CHECK(max30102_init(&pulse));
    float die_temp = 0.0f;
    if (max30102_read_temperature(&pulse, &die_temp) == ESP_OK) {
        ESP_LOGI(TAG, "MAX30102 die temp: %.2f C", die_temp);
    }

    // 5. MAX-M10S GPS
    ret = max_m10s_init(&gps, MAX_M10S_I2C_ADDR_DEFAULT);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "GPS init failed (%s) — will retry in task", esp_err_to_name(ret));
    }

    // 6. Launch pinned task on Core 0
    xTaskCreatePinnedToCore(
        sensor_integration_task,
        "SensorTask",
        4096 * 4,   // 16 KB — GPS uses a 256-byte stack buffer
        NULL,
        5,
        NULL,
        0           // Core 0
    );

    ESP_LOGI(TAG, "All sensors initialised. Task pinned to Core 0.");
}
