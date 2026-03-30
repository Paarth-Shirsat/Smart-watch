#include "bmp388.h"

#include <math.h>
#include <string.h>
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "BMP388";

#define I2C_TIMEOUT_MS  pdMS_TO_TICKS(100)

/* ── Calibration data (module-level, loaded once during init) ─────────────── */
static bmp388_calib_t s_calib;

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

/* ── Calibration loading ──────────────────────────────────────────────────── */

static esp_err_t load_calibration(i2c_port_t port, uint8_t addr)
{
    uint8_t raw[21];
    esp_err_t err = i2c_read_reg(port, addr, BMP388_REG_CALIB_DATA, raw, sizeof(raw));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Calibration read failed: %s", esp_err_to_name(err));
        return err;
    }

    /* Raw NVM values */
    uint16_t T1  = (uint16_t)(raw[0]  | ((uint16_t)raw[1]  << 8));
    uint16_t T2  = (uint16_t)(raw[2]  | ((uint16_t)raw[3]  << 8));
    int8_t   T3  = (int8_t)raw[4];
    int16_t  P1  = (int16_t)(raw[5]   | ((uint16_t)raw[6]  << 8));
    int16_t  P2  = (int16_t)(raw[7]   | ((uint16_t)raw[8]  << 8));
    int8_t   P3  = (int8_t)raw[9];
    int8_t   P4  = (int8_t)raw[10];
    uint16_t P5  = (uint16_t)(raw[11] | ((uint16_t)raw[12] << 8));
    uint16_t P6  = (uint16_t)(raw[13] | ((uint16_t)raw[14] << 8));
    int8_t   P7  = (int8_t)raw[15];
    int8_t   P8  = (int8_t)raw[16];
    int16_t  P9  = (int16_t)(raw[17]  | ((uint16_t)raw[18] << 8));
    int8_t   P10 = (int8_t)raw[19];
    int8_t   P11 = (int8_t)raw[20];

    /* Convert to double using datasheet scaling factors */
    s_calib.par_T1  = (double)T1  * 256.0;                          /* / 2^-8  */
    s_calib.par_T2  = (double)T2  / 1073741824.0;                   /* / 2^30  */
    s_calib.par_T3  = (double)T3  / 281474976710656.0;              /* / 2^48  */
    s_calib.par_P1  = ((double)P1  - 16384.0) / 1048576.0;         /* / 2^20  */
    s_calib.par_P2  = ((double)P2  - 16384.0) / 536870912.0;       /* / 2^29  */
    s_calib.par_P3  = (double)P3  / 4294967296.0;                   /* / 2^32  */
    s_calib.par_P4  = (double)P4  / 137438953472.0;                 /* / 2^37  */
    s_calib.par_P5  = (double)P5  * 8.0;                            /* / 2^-3  */
    s_calib.par_P6  = (double)P6  / 64.0;                           /* / 2^6   */
    s_calib.par_P7  = (double)P7  / 256.0;                          /* / 2^8   */
    s_calib.par_P8  = (double)P8  / 32768.0;                        /* / 2^15  */
    s_calib.par_P9  = (double)P9  / 281474976710656.0;              /* / 2^48  */
    s_calib.par_P10 = (double)P10 / 281474976710656.0;              /* / 2^48  */
    s_calib.par_P11 = (double)P11 / 36893488147419103232.0;         /* / 2^65  */

    return ESP_OK;
}

/* ── Compensation formulas (from BMP388 datasheet section 8.6) ────────────── */

static double compensate_temperature(uint32_t raw_temp)
{
    double pd1 = (double)raw_temp - s_calib.par_T1;
    double pd2 = pd1 * s_calib.par_T2;
    double pd3 = pd1 * pd1 * s_calib.par_T3;
    return pd2 + pd3;
}

static double compensate_pressure(uint32_t raw_press, double comp_temp)
{
    /* Offset term */
    double pd1 = s_calib.par_P6 * comp_temp;
    double pd2 = s_calib.par_P7 * (comp_temp * comp_temp);
    double pd3 = s_calib.par_P8 * (comp_temp * comp_temp * comp_temp);
    double po1 = s_calib.par_P5 + pd1 + pd2 + pd3;

    /* Sensitivity term */
    pd1 = s_calib.par_P2 * comp_temp;
    pd2 = s_calib.par_P3 * (comp_temp * comp_temp);
    pd3 = s_calib.par_P4 * (comp_temp * comp_temp * comp_temp);
    double po2 = (double)raw_press * (s_calib.par_P1 + pd1 + pd2 + pd3);

    /* Higher-order terms */
    pd1 = (double)raw_press * (double)raw_press;
    pd2 = s_calib.par_P9 + s_calib.par_P10 * comp_temp;
    pd3 = pd1 * pd2;
    double pd4 = pd3 + (double)raw_press * (double)raw_press * (double)raw_press
                       * s_calib.par_P11;

    return po1 + po2 + pd4;
}

static double pressure_to_altitude(double pressure_pa)
{
    return 44330.0 * (1.0 - pow(pressure_pa / BMP388_SEA_LEVEL_PA, 1.0 / 5.255));
}

/* ── Public API ───────────────────────────────────────────────────────────── */

esp_err_t bmp388_init(const bmp388_config_t *cfg)
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

    /* 2. Verify chip ID */
    uint8_t chip_id = 0;
    err = i2c_read_reg(cfg->i2c_port, cfg->i2c_addr, BMP388_REG_CHIP_ID, &chip_id, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Chip ID read failed: %s", esp_err_to_name(err));
        i2c_driver_delete(cfg->i2c_port);
        return err;
    }
    if (chip_id != BMP388_CHIP_ID) {
        ESP_LOGE(TAG, "Unexpected chip ID: 0x%02X (expected 0x%02X)", chip_id, BMP388_CHIP_ID);
        i2c_driver_delete(cfg->i2c_port);
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "BMP388 detected (chip_id=0x%02X)", chip_id);

    /* 3. Soft reset and wait for device ready */
    err = i2c_write_reg(cfg->i2c_port, cfg->i2c_addr, BMP388_REG_CMD, BMP388_CMD_SOFT_RESET);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Soft reset failed: %s", esp_err_to_name(err));
        i2c_driver_delete(cfg->i2c_port);
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    /* 4. Load calibration data */
    err = load_calibration(cfg->i2c_port, cfg->i2c_addr);
    if (err != ESP_OK) {
        i2c_driver_delete(cfg->i2c_port);
        return err;
    }

    /* 5. Configure oversampling: pressure x8, temperature x1 */
    err = i2c_write_reg(cfg->i2c_port, cfg->i2c_addr, BMP388_REG_OSR,
                        BMP388_OSR_PRESS_x8 | BMP388_OSR_TEMP_x1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "OSR config failed: %s", esp_err_to_name(err));
        i2c_driver_delete(cfg->i2c_port);
        return err;
    }

    /* 6. Configure IIR filter coefficient = 3 */
    err = i2c_write_reg(cfg->i2c_port, cfg->i2c_addr, BMP388_REG_CONFIG, BMP388_IIR_COEFF_3);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "IIR config failed: %s", esp_err_to_name(err));
        i2c_driver_delete(cfg->i2c_port);
        return err;
    }

    /* 7. Set ODR to 25 Hz (t_standby = 40 ms).
     *    With osr_p=x8 the measurement takes ~20 ms.  The BMP388 datasheet
     *    states the device is held in standby (STATUS=0x10, DRDY never set)
     *    if t_standby < t_meas.  The default ODR is 200 Hz (5 ms standby)
     *    which violates this constraint, so it must be set explicitly.        */
    err = i2c_write_reg(cfg->i2c_port, cfg->i2c_addr, BMP388_REG_ODR, BMP388_ODR_25HZ);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ODR config failed: %s", esp_err_to_name(err));
        i2c_driver_delete(cfg->i2c_port);
        return err;
    }

    /* 8. Enable pressure + temperature, start normal mode */
    err = i2c_write_reg(cfg->i2c_port, cfg->i2c_addr, BMP388_REG_PWR_CTRL,
                        BMP388_PRESS_EN | BMP388_TEMP_EN | BMP388_MODE_NORMAL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "PWR_CTRL config failed: %s", esp_err_to_name(err));
        i2c_driver_delete(cfg->i2c_port);
        return err;
    }

    ESP_LOGI(TAG, "Initialized (OSR press=x8 temp=x1, IIR coeff=3, ODR=25Hz, normal mode)");
    return ESP_OK;
}

esp_err_t bmp388_deinit(i2c_port_t port)
{
    return i2c_driver_delete(port);
}

esp_err_t bmp388_read(i2c_port_t port, uint8_t addr, bmp388_data_t *out)
{
    /* Poll status until both pressure and temperature data are ready.
     * At ODR=25 Hz, a new sample arrives every ~60 ms (20 ms meas + 40 ms
     * standby).  Allow up to 200 ms before declaring timeout.                */
    const int max_retries = 10;
    uint8_t status = 0;
    for (int i = 0; i < max_retries; i++) {
        esp_err_t err = i2c_read_reg(port, addr, BMP388_REG_STATUS, &status, 1);
        if (err != ESP_OK) {
            return err;
        }
        if ((status & (BMP388_STATUS_DRDY_PRESS | BMP388_STATUS_DRDY_TEMP)) ==
                      (BMP388_STATUS_DRDY_PRESS | BMP388_STATUS_DRDY_TEMP)) {
            break;
        }
        if (i == max_retries - 1) {
            ESP_LOGW(TAG, "Data ready timeout (status=0x%02X)", status);
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    /* Read 6 bytes: DATA_0..DATA_5 (pressure xlsb/lsb/msb, temp xlsb/lsb/msb) */
    uint8_t raw[6];
    esp_err_t err = i2c_read_reg(port, addr, BMP388_REG_DATA_0, raw, sizeof(raw));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Data read failed: %s", esp_err_to_name(err));
        return err;
    }

    uint32_t raw_press = (uint32_t)raw[0] | ((uint32_t)raw[1] << 8) | ((uint32_t)raw[2] << 16);
    uint32_t raw_temp  = (uint32_t)raw[3] | ((uint32_t)raw[4] << 8) | ((uint32_t)raw[5] << 16);

    double comp_temp  = compensate_temperature(raw_temp);
    double comp_press = compensate_pressure(raw_press, comp_temp);

    out->temperature = comp_temp;
    out->pressure    = comp_press;
    out->altitude    = pressure_to_altitude(comp_press);

    return ESP_OK;
}
