#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_err.h"
#include "bmp388.h"
#include "math.h"

static const char *TAG = "BMP388";
#define BMP388_SEA_LEVEL_PA       100000.0

// ═══════════════════════════════════════════════════════════════
//  I2C LOW-LEVEL HELPERS
// ═══════════════════════════════════════════════════════════════

static esp_err_t i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd,
                        pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t i2c_read_regs(uint8_t addr, uint8_t reg,
                                uint8_t *buf, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    // Write register address
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    // Repeated START then read
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true);
    if (len > 1) {
        i2c_master_read(cmd, buf, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, buf + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd,
                        pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    return ret;
}

// ═══════════════════════════════════════════════════════════════
//  I2C MASTER INIT
// ═══════════════════════════════════════════════════════════════

esp_err_t bmp388_i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = I2C_MASTER_SDA_IO,
        .scl_io_num       = I2C_MASTER_SCL_IO,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (ret != ESP_OK) return ret;
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

// ═══════════════════════════════════════════════════════════════
//  CALIBRATION PARSING  (BMP388 datasheet §8.4)
// ═══════════════════════════════════════════════════════════════

static esp_err_t bmp388_read_calibration(bmp388_dev_t *dev)
{
    uint8_t raw[21];
    esp_err_t ret = i2c_read_regs(dev->i2c_addr,
                                   BMP388_REG_CALIB_00, raw, 21);
    if (ret != ESP_OK) return ret;

    // Raw trim coefficients (little-endian)
    uint16_t par_T1 = (uint16_t)(raw[1]  << 8 | raw[0]);
    uint16_t par_T2 = (uint16_t)(raw[3]  << 8 | raw[2]);
    int8_t   par_T3 = (int8_t)raw[4];

    int16_t  par_P1  = (int16_t)(raw[6]  << 8 | raw[5]);
    int16_t  par_P2  = (int16_t)(raw[8]  << 8 | raw[7]);
    int8_t   par_P3  = (int8_t)raw[9];
    int8_t   par_P4  = (int8_t)raw[10];
    uint16_t par_P5  = (uint16_t)(raw[12] << 8 | raw[11]);
    uint16_t par_P6  = (uint16_t)(raw[14] << 8 | raw[13]);
    int8_t   par_P7  = (int8_t)raw[15];
    int8_t   par_P8  = (int8_t)raw[16];
    int16_t  par_P9  = (int16_t)(raw[18] << 8 | raw[17]);
    int8_t   par_P10 = (int8_t)raw[19];
    int8_t   par_P11 = (int8_t)raw[20];

    // Scale to floating-point (per BMP388 datasheet Table 15)
    bmp388_calib_t *c = &dev->calib;
    c->T1  = (double)par_T1  / 0.00390625;        // 2^-8
    c->T2  = (double)par_T2  / 1073741824.0;       // 2^30
    c->T3  = (double)par_T3  / 281474976710656.0;  // 2^48

    c->P1  = ((double)par_P1  - 16384.0) / 1048576.0;   // 2^20
    c->P2  = ((double)par_P2  - 16384.0) / 536870912.0; // 2^29
    c->P3  = (double)par_P3  / 4294967296.0;    // 2^32
    c->P4  = (double)par_P4  / 137438953472.0;  // 2^37
    c->P5  = (double)par_P5  / 0.125;           // 2^-3
    c->P6  = (double)par_P6  / 64.0;            // 2^6
    c->P7  = (double)par_P7  / 256.0;           // 2^8
    c->P8  = (double)par_P8  / 32768.0;         // 2^15
    c->P9  = (double)par_P9  / 281474976710656.0; // 2^48
    c->P10 = (double)par_P10 / 281474976710656.0; // 2^48
    c->P11 = (double)par_P11 / 36893488147419103232.0; // 2^65

    return ESP_OK;
}

// ═══════════════════════════════════════════════════════════════
//  BMP388 INIT
// ═══════════════════════════════════════════════════════════════

esp_err_t bmp388_init(bmp388_dev_t *dev, uint8_t addr)
{
    dev->i2c_addr = addr;

    // 1. Verify Chip ID
    uint8_t chip_id = 0;
    esp_err_t ret = i2c_read_regs(dev->i2c_addr,
                                   BMP388_REG_CHIP_ID, &chip_id, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C read failed (addr=0x%02X): %s",
                 addr, esp_err_to_name(ret));
        return ret;
    }
    if (chip_id != BMP388_CHIP_ID) {
        ESP_LOGE(TAG, "Wrong chip ID: 0x%02X (expected 0x%02X)",
                 chip_id, BMP388_CHIP_ID);
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "BMP388 found, Chip ID: 0x%02X", chip_id);

    // 2. Soft Reset
    ret = i2c_write_reg(dev->i2c_addr, BMP388_REG_CMD, BMP388_CMD_SOFTRESET);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(10));

    // 3. Read Calibration
    ret = bmp388_read_calibration(dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Calibration read failed");
        return ret;
    }
    ESP_LOGI(TAG, "Calibration data loaded");

    // 4. Oversampling: x8 pressure, x1 temperature
    ret = i2c_write_reg(dev->i2c_addr, BMP388_REG_OSR, 0x03);
    if (ret != ESP_OK) return ret;

    // 5. Output Data Rate: 50 Hz (ODR = 0x02)
    ret = i2c_write_reg(dev->i2c_addr, BMP388_REG_ODR, 0x02);
    if (ret != ESP_OK) return ret;

    // 6. IIR Filter: coefficient 3 (Config reg)
    ret = i2c_write_reg(dev->i2c_addr, BMP388_REG_CONFIG, 0x04);
    if (ret != ESP_OK) return ret;

    // 7. Power ON: pressure + temp enabled, Normal mode
    ret = i2c_write_reg(dev->i2c_addr, BMP388_REG_PWR_CTRL,
                         BMP388_PWR_NORMAL_MODE);
    if (ret != ESP_OK) return ret;

    ESP_LOGI(TAG, "BMP388 configured in Normal mode");
    return ESP_OK;
}

// ═══════════════════════════════════════════════════════════════
//  COMPENSATED TEMPERATURE  (BMP388 datasheet §8.5)
// ═══════════════════════════════════════════════════════════════

static double compensate_temperature(bmp388_calib_t *c,
                                     uint32_t raw_temp)
{
    double pd1 = (double)raw_temp - c->T1;
    double pd2 = pd1 * c->T2;
    return pd2 + (pd1 * pd1) * c->T3;
}

// ═══════════════════════════════════════════════════════════════
//  COMPENSATED PRESSURE  (BMP388 datasheet §8.5)
// ═══════════════════════════════════════════════════════════════

static double compensate_pressure(bmp388_calib_t *c,
                                  uint32_t raw_press,
                                  double comp_temp)
{
    double pd1  = c->P6 * comp_temp;
    double pd2  = c->P7 * (comp_temp * comp_temp);
    double pd3  = c->P8 * (comp_temp * comp_temp * comp_temp);
    double po1  = c->P5 + pd1 + pd2 + pd3;

    double pd4  = c->P2 * comp_temp;
    double pd5  = c->P3 * (comp_temp * comp_temp);
    double pd6  = c->P4 * (comp_temp * comp_temp * comp_temp);
    double po2  = (double)raw_press * (c->P1 + pd4 + pd5 + pd6);

    double pd7  = (double)raw_press * (double)raw_press;
    double pd8  = c->P9 + c->P10 * comp_temp;
    double pd9  = pd7 * pd8;
    double pd10 = pd9 + (double)(raw_press * raw_press * raw_press) * c->P11;

    return po1 + po2 + pd10;
}

// ═══════════════════════════════════════════════════════════════
//  PUBLIC READ FUNCTION
// ═══════════════════════════════════════════════════════════════

esp_err_t bmp388_read_temperature_pressure(bmp388_dev_t *dev,
                                            double *temperature_c,
                                            double *pressure_pa)
{
    // Wait for data ready (STATUS register bit 5=press_rdy, bit 6=temp_rdy)
    uint8_t status = 0;
    for (int i = 0; i < 10; i++) {
        i2c_read_regs(dev->i2c_addr, BMP388_REG_STATUS, &status, 1);
        if ((status & 0x60) == 0x60) break;
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    // Burst read 6 bytes: DATA_0..DATA_5
    uint8_t raw[6];
    esp_err_t ret = i2c_read_regs(dev->i2c_addr, BMP388_REG_DATA_0, raw, 6);
    if (ret != ESP_OK) return ret;

    // Combine bytes (24-bit little-endian)
    uint32_t raw_press = (uint32_t)raw[2] << 16 |
                         (uint32_t)raw[1] << 8  |
                         (uint32_t)raw[0];

    uint32_t raw_temp  = (uint32_t)raw[5] << 16 |
                         (uint32_t)raw[4] << 8  |
                         (uint32_t)raw[3];

    // Compensate
    double comp_temp = compensate_temperature(&dev->calib, raw_temp);
    *temperature_c   = comp_temp;
    *pressure_pa     = compensate_pressure(&dev->calib, raw_press, comp_temp);

    return ESP_OK;
}
static double pressure_to_altitude(double pressure_pa)
{
    return 44330.0 * (1.0 - pow((pressure_pa) / BMP388_SEA_LEVEL_PA, 1.0 / 5.255));
}

// ═══════════════════════════════════════════════════════════════
//  APP MAIN
// ═══════════════════════════════════════════════════════════════

void app_main(void)
{
    ESP_LOGI(TAG, "Initializing I2C on SDA=GPIO%d, SCL=GPIO%d",
             I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);

    ESP_ERROR_CHECK(bmp388_i2c_master_init());

    bmp388_dev_t sensor;
    ESP_ERROR_CHECK(bmp388_init(&sensor, BMP388_I2C_ADDR_SECONDARY));

    double temperature, pressure;
    esp_err_t ret;

    while (1) {
        ret = bmp388_read_temperature_pressure(&sensor,
                                               &temperature,
                                               &pressure);
        if (ret == ESP_OK) {
            double altitude = pressure_to_altitude(pressure);   
            ESP_LOGI(TAG, "Pressure raw: %.2f Pa", pressure);
            ESP_LOGI(TAG, "Temperature: %.2f °C | Pressure: %.2f atm | Altitude: %.2f m",
                     temperature, (pressure / 101325), altitude);
        } else {
            ESP_LOGE(TAG, "Read failed: %s", esp_err_to_name(ret));
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}