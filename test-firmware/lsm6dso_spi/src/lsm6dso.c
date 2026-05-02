#include "lsm6dso.h"

#include <string.h>
#include "driver/spi_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "LSM6DSO";

static spi_device_handle_t s_dev  = NULL;
static spi_host_device_t   s_host = SPI2_HOST;

/* ── SPI helpers ──────────────────────────────────────────────────────────── */

static esp_err_t spi_read_reg(uint8_t reg, uint8_t *data, size_t len)
{
    /* Allocate tx/rx on the stack — max burst here is 13 bytes (1 addr + 12) */
    uint8_t tx[13] = {0};
    uint8_t rx[13] = {0};

    tx[0] = reg | LSM6DSO_SPI_READ;

    spi_transaction_t t = {
        .length    = (1 + len) * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };

    esp_err_t err = spi_device_transmit(s_dev, &t);
    if (err == ESP_OK) {
        memcpy(data, &rx[1], len);
    }
    return err;
}

static esp_err_t spi_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = { reg & 0x7F, value };   /* Write: bit 7 = 0 */

    spi_transaction_t t = {
        .length    = 16,
        .tx_buffer = tx,
    };

    return spi_device_transmit(s_dev, &t);
}

/* ── Public API ───────────────────────────────────────────────────────────── */

esp_err_t lsm6dso_init(const lsm6dso_config_t *cfg)
{
    s_host = cfg->spi_host;

    /* 1. Initialise SPI bus */
    spi_bus_config_t bus_cfg = {
        .mosi_io_num     = cfg->mosi_pin,
        .miso_io_num     = cfg->miso_pin,
        .sclk_io_num     = cfg->sclk_pin,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = 16,
    };
    esp_err_t err = spi_bus_initialize(cfg->spi_host, &bus_cfg, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_initialize failed: %s", esp_err_to_name(err));
        return err;
    }

    /* 2. Add LSM6DSO device — Mode 3 (CPOL=1, CPHA=1) */
    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz = cfg->spi_freq_hz,
        .mode           = 3,
        .spics_io_num   = cfg->cs_pin,
        .queue_size     = 1,
    };
    err = spi_bus_add_device(cfg->spi_host, &dev_cfg, &s_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_add_device failed: %s", esp_err_to_name(err));
        spi_bus_free(cfg->spi_host);
        return err;
    }

    /* 3. Verify WHO_AM_I */
    uint8_t who = 0;
    err = spi_read_reg(LSM6DSO_REG_WHO_AM_I, &who, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "WHO_AM_I read failed: %s", esp_err_to_name(err));
        goto cleanup;
    }
    if (who != LSM6DSO_WHO_AM_I_VAL) {
        ESP_LOGE(TAG, "Unexpected WHO_AM_I: 0x%02X (expected 0x%02X)",
                 who, LSM6DSO_WHO_AM_I_VAL);
        err = ESP_ERR_NOT_FOUND;
        goto cleanup;
    }
    ESP_LOGI(TAG, "LSM6DSO detected (WHO_AM_I=0x%02X)", who);

    /* 4. Soft reset — bit self-clears within ~50 µs */
    err = spi_write_reg(LSM6DSO_REG_CTRL3_C, LSM6DSO_SW_RESET);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SW_RESET failed: %s", esp_err_to_name(err));
        goto cleanup;
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    /* 5. Enable block data update (prevents reading stale half-words) */
    err = spi_write_reg(LSM6DSO_REG_CTRL3_C, LSM6DSO_BDU);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "BDU config failed: %s", esp_err_to_name(err));
        goto cleanup;
    }

    /* 6. Configure accelerometer: 104 Hz, ±4 g */
    err = spi_write_reg(LSM6DSO_REG_CTRL1_XL,
                        LSM6DSO_XL_ODR_104HZ | LSM6DSO_XL_FS_4G);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "CTRL1_XL config failed: %s", esp_err_to_name(err));
        goto cleanup;
    }

    /* 7. Configure gyroscope: 104 Hz, ±2000 dps */
    err = spi_write_reg(LSM6DSO_REG_CTRL2_G,
                        LSM6DSO_G_ODR_104HZ | LSM6DSO_G_FS_2000DPS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "CTRL2_G config failed: %s", esp_err_to_name(err));
        goto cleanup;
    }

    ESP_LOGI(TAG, "Initialized (Accel=104Hz/±4g, Gyro=104Hz/±2000dps)");
    return ESP_OK;

cleanup:
    spi_bus_remove_device(s_dev);
    s_dev = NULL;
    spi_bus_free(cfg->spi_host);
    return err;
}

esp_err_t lsm6dso_deinit(spi_host_device_t host)
{
    esp_err_t err = spi_bus_remove_device(s_dev);
    s_dev = NULL;
    if (err != ESP_OK) {
        return err;
    }
    return spi_bus_free(host);
}

esp_err_t lsm6dso_read_accel(lsm6dso_raw_t *out)
{
    uint8_t raw[6];
    esp_err_t err = spi_read_reg(LSM6DSO_REG_OUTX_L_A, raw, 6);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Accel read failed: %s", esp_err_to_name(err));
        return err;
    }

    out->x = (int16_t)((raw[1] << 8) | raw[0]);
    out->y = (int16_t)((raw[3] << 8) | raw[2]);
    out->z = (int16_t)((raw[5] << 8) | raw[4]);
    return ESP_OK;
}

esp_err_t lsm6dso_read_gyro(lsm6dso_raw_t *out)
{
    uint8_t raw[6];
    esp_err_t err = spi_read_reg(LSM6DSO_REG_OUTX_L_G, raw, 6);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Gyro read failed: %s", esp_err_to_name(err));
        return err;
    }

    out->x = (int16_t)((raw[1] << 8) | raw[0]);
    out->y = (int16_t)((raw[3] << 8) | raw[2]);
    out->z = (int16_t)((raw[5] << 8) | raw[4]);
    return ESP_OK;
}

esp_err_t lsm6dso_read_all(lsm6dso_raw_t *accel, lsm6dso_raw_t *gyro)
{
    /* Gyro registers start at 0x22, accel at 0x28 — contiguous 12-byte burst */
    uint8_t raw[12];
    esp_err_t err = spi_read_reg(LSM6DSO_REG_OUTX_L_G, raw, 12);
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
