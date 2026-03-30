#include "lsm6dso.h"

#include <string.h>
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "LSM6DSO";

#define I2C_TIMEOUT_MS  pdMS_TO_TICKS(100)

/* ── I2C helpers ─────────────────────────────────────────────────────────── */

static esp_err_t i2c_read_reg(i2c_port_t port, uint8_t reg,
                               uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(port, LSM6DSO_I2C_ADDR,
                                        &reg, 1, data, len, I2C_TIMEOUT_MS);
}

static esp_err_t i2c_write_reg(i2c_port_t port, uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = { reg, value };
    return i2c_master_write_to_device(port, LSM6DSO_I2C_ADDR,
                                      buf, sizeof(buf), I2C_TIMEOUT_MS);
}

/* ── Public API ───────────────────────────────────────────────────────────── */

esp_err_t lsm6dso_init(const lsm6dso_config_t *cfg)
{
    /* 1. Install I2C driver */
    i2c_config_t i2c_cfg = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = cfg->sda_pin,
        .scl_io_num       = cfg->scl_pin,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = cfg->i2c_freq,
    };
    esp_err_t err = i2c_param_config(cfg->i2c_port, &i2c_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_param_config failed: %s", esp_err_to_name(err));
        return err;
    }
    err = i2c_driver_install(cfg->i2c_port, I2C_MODE_MASTER, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_driver_install failed: %s", esp_err_to_name(err));
        return err;
    }

    /* 2. Verify WHO_AM_I */
    uint8_t who = 0;
    err = i2c_read_reg(cfg->i2c_port, LSM6DSO_REG_WHO_AM_I, &who, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "WHO_AM_I read failed: %s", esp_err_to_name(err));
        i2c_driver_delete(cfg->i2c_port);
        return err;
    }
    if (who != LSM6DSO_WHO_AM_I_VAL) {
        ESP_LOGE(TAG, "Unexpected WHO_AM_I: 0x%02X (expected 0x%02X)",
                 who, LSM6DSO_WHO_AM_I_VAL);
        i2c_driver_delete(cfg->i2c_port);
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "LSM6DSO detected (WHO_AM_I=0x%02X)", who);

    /* 3. Soft reset — bit self-clears within ~50 µs */
    err = i2c_write_reg(cfg->i2c_port, LSM6DSO_REG_CTRL3_C, LSM6DSO_SW_RESET);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SW_RESET failed: %s", esp_err_to_name(err));
        i2c_driver_delete(cfg->i2c_port);
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    /* 4. Enable block data update (prevents reading stale half-words) */
    err = i2c_write_reg(cfg->i2c_port, LSM6DSO_REG_CTRL3_C, LSM6DSO_BDU);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "BDU config failed: %s", esp_err_to_name(err));
        i2c_driver_delete(cfg->i2c_port);
        return err;
    }

    /* 5. Configure accelerometer: 104 Hz, ±4 g */
    err = i2c_write_reg(cfg->i2c_port, LSM6DSO_REG_CTRL1_XL,
                        LSM6DSO_XL_ODR_104HZ | LSM6DSO_XL_FS_4G);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "CTRL1_XL config failed: %s", esp_err_to_name(err));
        i2c_driver_delete(cfg->i2c_port);
        return err;
    }

    /* 6. Configure gyroscope: 104 Hz, ±2000 dps */
    err = i2c_write_reg(cfg->i2c_port, LSM6DSO_REG_CTRL2_G,
                        LSM6DSO_G_ODR_104HZ | LSM6DSO_G_FS_2000DPS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "CTRL2_G config failed: %s", esp_err_to_name(err));
        i2c_driver_delete(cfg->i2c_port);
        return err;
    }

    ESP_LOGI(TAG, "Initialized (addr=0x%02X, Accel=104Hz/±4g, Gyro=104Hz/±2000dps)",
             LSM6DSO_I2C_ADDR);
    return ESP_OK;
}

esp_err_t lsm6dso_deinit(i2c_port_t port)
{
    return i2c_driver_delete(port);
}

esp_err_t lsm6dso_read_accel(i2c_port_t port, lsm6dso_raw_t *out)
{
    uint8_t raw[6];
    esp_err_t err = i2c_read_reg(port, LSM6DSO_REG_OUTX_L_A, raw, 6);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Accel read failed: %s", esp_err_to_name(err));
        return err;
    }

    out->x = (int16_t)((raw[1] << 8) | raw[0]);
    out->y = (int16_t)((raw[3] << 8) | raw[2]);
    out->z = (int16_t)((raw[5] << 8) | raw[4]);
    return ESP_OK;
}

esp_err_t lsm6dso_read_gyro(i2c_port_t port, lsm6dso_raw_t *out)
{
    uint8_t raw[6];
    esp_err_t err = i2c_read_reg(port, LSM6DSO_REG_OUTX_L_G, raw, 6);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Gyro read failed: %s", esp_err_to_name(err));
        return err;
    }

    out->x = (int16_t)((raw[1] << 8) | raw[0]);
    out->y = (int16_t)((raw[3] << 8) | raw[2]);
    out->z = (int16_t)((raw[5] << 8) | raw[4]);
    return ESP_OK;
}

esp_err_t lsm6dso_read_all(i2c_port_t port, lsm6dso_raw_t *accel, lsm6dso_raw_t *gyro)
{
    /* Gyro registers start at 0x22, accel at 0x28 — contiguous 12-byte burst */
    uint8_t raw[12];
    esp_err_t err = i2c_read_reg(port, LSM6DSO_REG_OUTX_L_G, raw, 12);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Burst read failed: %s", esp_err_to_name(err));
        return err;
    }

    gyro->x  = (int16_t)((raw[1]  << 8) | raw[0]);
    gyro->y  = (int16_t)((raw[3]  << 8) | raw[2]);
    gyro->z  = (int16_t)((raw[5]  << 8) | raw[4]);
    accel->x = (int16_t)((raw[7]  << 8) | raw[6]);
    accel->y = (int16_t)((raw[9]  << 8) | raw[8]);
    accel->z = (int16_t)((raw[11] << 8) | raw[10]);
    return ESP_OK;
}
