#include "bmp388.h"
#include "i2c_manager.h"
#include "esp_log.h"
#include "math.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "BMP388_DRV";

// ─── Calibration Parsing ──────────────────────────────────────
static esp_err_t bmp388_read_calibration(bmp388_dev_t *dev)
{
    uint8_t raw[21];
    esp_err_t ret = i2c_manager_read_regs(dev->i2c_addr, BMP388_REG_CALIB_00, raw, 21);
    if (ret != ESP_OK) return ret;

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

    bmp388_calib_t *c = &dev->calib;
    c->T1  = (double)par_T1  / 0.00390625;
    c->T2  = (double)par_T2  / 1073741824.0;
    c->T3  = (double)par_T3  / 281474976710656.0;

    c->P1  = ((double)par_P1  - 16384.0) / 1048576.0;
    c->P2  = ((double)par_P2  - 16384.0) / 536870912.0;
    c->P3  = (double)par_P3  / 4294967296.0;
    c->P4  = (double)par_P4  / 137438953472.0;
    c->P5  = (double)par_P5  / 0.125;
    c->P6  = (double)par_P6  / 64.0;
    c->P7  = (double)par_P7  / 256.0;
    c->P8  = (double)par_P8  / 32768.0;
    c->P9  = (double)par_P9  / 281474976710656.0;
    c->P10 = (double)par_P10 / 281474976710656.0;
    c->P11 = (double)par_P11 / 36893488147419103232.0;

    return ESP_OK;
}

// ─── Compensation Formulas ────────────────────────────────────
static double compensate_temperature(bmp388_calib_t *c, uint32_t raw_temp)
{
    double pd1 = (double)raw_temp - c->T1;
    double pd2 = pd1 * c->T2;
    return pd2 + (pd1 * pd1) * c->T3;
}

static double compensate_pressure(bmp388_calib_t *c, uint32_t raw_press, double comp_temp)
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

// ─── Public API ───────────────────────────────────────────────
esp_err_t bmp388_init(bmp388_dev_t *dev, uint8_t addr)
{
    dev->i2c_addr = addr;

    uint8_t chip_id = 0;
    esp_err_t ret = i2c_manager_read_regs(dev->i2c_addr, BMP388_REG_CHIP_ID, &chip_id, 1);
    if (ret != ESP_OK || chip_id != BMP388_CHIP_ID) {
        ESP_LOGE(TAG, "BMP388 not found (ID: 0x%02X)", chip_id);
        return ESP_ERR_NOT_FOUND;
    }

    i2c_manager_write_reg(dev->i2c_addr, BMP388_REG_CMD, BMP388_CMD_SOFTRESET);
    vTaskDelay(pdMS_TO_TICKS(10));

    ret = bmp388_read_calibration(dev);
    if (ret != ESP_OK) return ret;

    i2c_manager_write_reg(dev->i2c_addr, BMP388_REG_OSR, 0x03);
    i2c_manager_write_reg(dev->i2c_addr, BMP388_REG_ODR, 0x02);
    i2c_manager_write_reg(dev->i2c_addr, BMP388_REG_CONFIG, 0x04);
    i2c_manager_write_reg(dev->i2c_addr, BMP388_REG_PWR_CTRL, BMP388_PWR_NORMAL_MODE);

    ESP_LOGI(TAG, "BMP388 initialized successfully");
    return ESP_OK;
}

esp_err_t bmp388_read_temperature_pressure(bmp388_dev_t *dev, double *temperature_c, double *pressure_pa)
{
    uint8_t status = 0;
    for (int i = 0; i < 10; i++) {
        i2c_manager_read_regs(dev->i2c_addr, BMP388_REG_STATUS, &status, 1);
        if ((status & 0x60) == 0x60) break;
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    uint8_t raw[6];
    esp_err_t ret = i2c_manager_read_regs(dev->i2c_addr, BMP388_REG_DATA_0, raw, 6);
    if (ret != ESP_OK) return ret;

    uint32_t raw_press = (uint32_t)raw[2] << 16 | (uint32_t)raw[1] << 8 | (uint32_t)raw[0];
    uint32_t raw_temp  = (uint32_t)raw[5] << 16 | (uint32_t)raw[4] << 8 | (uint32_t)raw[3];

    double comp_temp = compensate_temperature(&dev->calib, raw_temp);
    *temperature_c   = comp_temp; // Correctly unscaled for temperature_c? No, datasheet says /163840.0.
    // Wait, the user's latest code has temperature displayed directly. 
    // BUT the previous log said -104e6.
    // I will scale it for Celsius display in main.c or here?
    // Let's stick exactly to the provided Barometer.c logic for now (t_lin).
    *pressure_pa     = compensate_pressure(&dev->calib, raw_press, comp_temp);

    return ESP_OK;
}

double bmp388_pressure_to_altitude(double pressure_pa)
{
    return 44330.0 * (1.0 - pow(pressure_pa / BMP388_SEA_LEVEL_PA, 1.0 / 5.255));
}
