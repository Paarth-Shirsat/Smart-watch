#include "max30102.h"

#include <string.h>
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "MAX30102";

#define I2C_TIMEOUT_MS  pdMS_TO_TICKS(100)

/* ── I2C helpers ─────────────────────────────────────────────────────────── */

static esp_err_t i2c_read_reg(i2c_port_t port, uint8_t addr,
                               uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(port, addr, &reg, 1,
                                        data, len, I2C_TIMEOUT_MS);
}

static esp_err_t i2c_write_reg(i2c_port_t port, uint8_t addr,
                                uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = { reg, value };
    return i2c_master_write_to_device(port, addr, buf, sizeof(buf),
                                      I2C_TIMEOUT_MS);
}

/* ── Public API ───────────────────────────────────────────────────────────── */

esp_err_t max30102_init(const max30102_config_t *cfg)
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

    /* 2. Verify part ID */
    uint8_t part_id = 0;
    err = i2c_read_reg(cfg->i2c_port, MAX30102_I2C_ADDR,
                       MAX30102_REG_PART_ID, &part_id, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Part ID read failed: %s", esp_err_to_name(err));
        i2c_driver_delete(cfg->i2c_port);
        return err;
    }
    if (part_id != MAX30102_PART_ID) {
        ESP_LOGE(TAG, "Unexpected part ID: 0x%02X (expected 0x%02X)",
                 part_id, MAX30102_PART_ID);
        i2c_driver_delete(cfg->i2c_port);
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "MAX30102 detected (part_id=0x%02X)", part_id);

    /* 3. Soft reset and wait for device ready (~1 ms per datasheet) */
    err = i2c_write_reg(cfg->i2c_port, MAX30102_I2C_ADDR,
                        MAX30102_REG_MODE_CONFIG, MAX30102_MODE_RESET);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Reset failed: %s", esp_err_to_name(err));
        i2c_driver_delete(cfg->i2c_port);
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    /* 4. Configure FIFO: 4× sample averaging, rollover enabled */
    err = i2c_write_reg(cfg->i2c_port, MAX30102_I2C_ADDR,
                        MAX30102_REG_FIFO_CONFIG,
                        MAX30102_SMP_AVE_4 | MAX30102_FIFO_ROLLOVER_EN);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "FIFO config failed: %s", esp_err_to_name(err));
        i2c_driver_delete(cfg->i2c_port);
        return err;
    }

    /* 5. Configure SpO2: ADC range 4096 nA, 100 SPS, 411 µs pulse (18-bit) */
    err = i2c_write_reg(cfg->i2c_port, MAX30102_I2C_ADDR,
                        MAX30102_REG_SPO2_CONFIG,
                        MAX30102_SPO2_ADC_RGE_4096 |
                        MAX30102_SPO2_SR_100        |
                        MAX30102_SPO2_PW_411US);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SpO2 config failed: %s", esp_err_to_name(err));
        i2c_driver_delete(cfg->i2c_port);
        return err;
    }

    /* 6. Set LED pulse amplitudes */
    err = i2c_write_reg(cfg->i2c_port, MAX30102_I2C_ADDR,
                        MAX30102_REG_LED1_PA, cfg->led_pa);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "LED1 PA config failed: %s", esp_err_to_name(err));
        i2c_driver_delete(cfg->i2c_port);
        return err;
    }
    err = i2c_write_reg(cfg->i2c_port, MAX30102_I2C_ADDR,
                        MAX30102_REG_LED2_PA, cfg->led_pa);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "LED2 PA config failed: %s", esp_err_to_name(err));
        i2c_driver_delete(cfg->i2c_port);
        return err;
    }

    /* 7. Enter SpO2 mode (starts sampling) */
    err = i2c_write_reg(cfg->i2c_port, MAX30102_I2C_ADDR,
                        MAX30102_REG_MODE_CONFIG, MAX30102_MODE_SPO2);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Mode config failed: %s", esp_err_to_name(err));
        i2c_driver_delete(cfg->i2c_port);
        return err;
    }

    ESP_LOGI(TAG, "Initialized (SpO2 mode, ADC=4096nA, SR=100SPS, PW=411us, LED_PA=0x%02X)",
             cfg->led_pa);
    return ESP_OK;
}

esp_err_t max30102_deinit(i2c_port_t port)
{
    return i2c_driver_delete(port);
}

esp_err_t max30102_read_fifo(i2c_port_t port, uint8_t addr,
                              max30102_sample_t *buf, size_t buf_len,
                              size_t *out_count)
{
    *out_count = 0;

    /* Read FIFO pointers to determine how many samples are available */
    uint8_t wr_ptr = 0, rd_ptr = 0;
    esp_err_t err = i2c_read_reg(port, addr, MAX30102_REG_FIFO_WR_PTR, &wr_ptr, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "FIFO WR_PTR read failed: %s", esp_err_to_name(err));
        return err;
    }
    err = i2c_read_reg(port, addr, MAX30102_REG_FIFO_RD_PTR, &rd_ptr, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "FIFO RD_PTR read failed: %s", esp_err_to_name(err));
        return err;
    }

    /* Number of unread samples, accounting for pointer wrap-around (depth=32) */
    int num_samples = ((int)wr_ptr - (int)rd_ptr + MAX30102_FIFO_DEPTH)
                      % MAX30102_FIFO_DEPTH;
    if (num_samples == 0) {
        return ESP_OK;  /* Nothing new yet */
    }

    /* Clamp to caller-supplied buffer length */
    if ((size_t)num_samples > buf_len) {
        num_samples = (int)buf_len;
    }

    /* Burst-read: 6 bytes per sample (3 bytes Red + 3 bytes IR).
     * The FIFO_DATA register auto-advances the read pointer on each 3-byte
     * boundary, so a single i2c_read_reg call retrieves all samples.          */
    size_t raw_len = (size_t)num_samples * 6;
    uint8_t raw[MAX30102_FIFO_DEPTH * 6];  /* Stack-safe: 32 × 6 = 192 bytes  */

    err = i2c_read_reg(port, addr, MAX30102_REG_FIFO_DATA, raw, raw_len);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "FIFO data read failed: %s", esp_err_to_name(err));
        return err;
    }

    /* Unpack 18-bit Red and IR values from each 6-byte group.
     * Bits [23:18] of each 3-byte word are reserved (always 0).               */
    for (int i = 0; i < num_samples; i++) {
        const uint8_t *s = &raw[i * 6];
        buf[i].red = ((uint32_t)(s[0] & 0x03) << 16)
                   | ((uint32_t) s[1]          <<  8)
                   |  (uint32_t) s[2];
        buf[i].ir  = ((uint32_t)(s[3] & 0x03) << 16)
                   | ((uint32_t) s[4]          <<  8)
                   |  (uint32_t) s[5];
    }

    *out_count = (size_t)num_samples;
    return ESP_OK;
}
