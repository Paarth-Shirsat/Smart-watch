#pragma once

#include <stdint.h>
#include "driver/i2c.h"
#include "esp_err.h"

/* ── I2C defaults ─────────────────────────────────────────────────────────── */
#define BMP388_I2C_ADDR_DEFAULT   0x77   /* SDO high (most breakout boards)  */
#define BMP388_I2C_ADDR_ALT       0x76   /* SDO low                          */
#define BMP388_I2C_FREQ_HZ        400000 /* 400 kHz fast mode                */

/* ── Register map ─────────────────────────────────────────────────────────── */
#define BMP388_REG_CHIP_ID        0x00
#define BMP388_REG_ERR            0x02
#define BMP388_REG_STATUS         0x03
#define BMP388_REG_DATA_0         0x04   /* press xlsb                       */
#define BMP388_REG_DATA_3         0x07   /* temp  xlsb                       */
#define BMP388_REG_INT_CTRL       0x19
#define BMP388_REG_PWR_CTRL       0x1B
#define BMP388_REG_OSR            0x1C
#define BMP388_REG_ODR            0x1D
#define BMP388_REG_CONFIG         0x1F
#define BMP388_REG_CALIB_DATA     0x31   /* 21 bytes, 0x31–0x45              */
#define BMP388_REG_CMD            0x7E

/* ── Constants ────────────────────────────────────────────────────────────── */
#define BMP388_CHIP_ID            0x50
#define BMP388_CMD_SOFT_RESET     0xB6
#define BMP388_STATUS_CMD_RDY     (1 << 4)
#define BMP388_STATUS_DRDY_PRESS  (1 << 5)
#define BMP388_STATUS_DRDY_TEMP   (1 << 6)

/* PWR_CTRL bits — datasheet Table 26
 *   bits[1:0] = mode  (00=sleep, 01/10=forced, 11=normal)
 *   bit  4    = press_en
 *   bit  5    = temp_en                                                     */
#define BMP388_PRESS_EN           (1 << 4)
#define BMP388_TEMP_EN            (1 << 5)
#define BMP388_MODE_NORMAL        (3 << 0)

/* Oversampling (OSR register, 0x1C)
 *   bits[2:0] = osr_p  (0=x1, 1=x2, 2=x4, 3=x8, 4=x16, 5=x32)
 *   bits[5:3] = osr_t  (same encoding)                                      */
#define BMP388_OSR_PRESS_x8       (3 << 0)
#define BMP388_OSR_TEMP_x1        (0 << 3)

/* ODR register (0x1D) — odr_sel value
 * With osr_p=x8, t_meas ≈ 20 ms.  The BMP388 stays in standby if
 * t_standby < t_meas, so we must choose ODR ≤ 25 Hz (t_standby = 40 ms).   */
#define BMP388_ODR_25HZ           0x03   /* t_standby = 40 ms  */

/* IIR filter coefficient (CONFIG register 0x1F, bits [3:1])
 *   0=bypass, 1=coeff1, 2=coeff3, 3=coeff7, …                              */
#define BMP388_IIR_COEFF_3        (2 << 1)

/* Sea-level pressure reference (Pa) */
#define BMP388_SEA_LEVEL_PA       101325.0

/* ── Calibration coefficients (converted to double) ───────────────────────── */
typedef struct {
    double par_T1;
    double par_T2;
    double par_T3;
    double par_P1;
    double par_P2;
    double par_P3;
    double par_P4;
    double par_P5;
    double par_P6;
    double par_P7;
    double par_P8;
    double par_P9;
    double par_P10;
    double par_P11;
} bmp388_calib_t;

/* ── Driver configuration ─────────────────────────────────────────────────── */
typedef struct {
    i2c_port_t  i2c_port;
    int         sda_pin;
    int         scl_pin;
    uint32_t    i2c_freq;
    uint8_t     i2c_addr;
} bmp388_config_t;

/* ── Sensor reading ───────────────────────────────────────────────────────── */
typedef struct {
    double temperature; /* °C  */
    double pressure;    /* Pa  */
    double altitude;    /* m   */
} bmp388_data_t;

/* ── Public API ───────────────────────────────────────────────────────────── */

/**
 * @brief  Install I2C driver, verify chip ID, load calibration data,
 *         configure oversampling / IIR filter, and start normal mode.
 *
 * @param  cfg  Pointer to driver configuration.
 * @return ESP_OK on success, otherwise an esp_err_t error code.
 */
esp_err_t bmp388_init(const bmp388_config_t *cfg);

/**
 * @brief  Uninstall I2C driver and release resources.
 *
 * @param  port  I2C port used during init.
 * @return ESP_OK on success.
 */
esp_err_t bmp388_deinit(i2c_port_t port);

/**
 * @brief  Read compensated temperature, pressure, and altitude.
 *
 * @param  port  I2C port used during init.
 * @param  addr  Device I2C address.
 * @param  out   Pointer to bmp388_data_t to fill.
 * @return ESP_OK on success, ESP_ERR_TIMEOUT if data-ready times out.
 */
esp_err_t bmp388_read(i2c_port_t port, uint8_t addr, bmp388_data_t *out);
