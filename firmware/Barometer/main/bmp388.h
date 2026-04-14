#pragma once

#include <stdint.h>
#include "driver/i2c.h"
#include "esp_err.h"

// ─── I2C Config ──────────────────────────────────────────────
#define I2C_MASTER_NUM          I2C_NUM_0
#define I2C_MASTER_SDA_IO       8           // GPIO 8
#define I2C_MASTER_SCL_IO       9           // GPIO 9
#define I2C_MASTER_FREQ_HZ      400000      // 400 kHz Fast Mode
#define I2C_MASTER_TIMEOUT_MS   1000

// ─── BMP388 I2C Addresses ─────────────────────────────────────
#define BMP388_I2C_ADDR_SECONDARY  0x77     // SDO = VCC

// ─── BMP388 Register Map ──────────────────────────────────────
#define BMP388_REG_CHIP_ID         0x00
#define BMP388_REG_ERR             0x02
#define BMP388_REG_STATUS          0x03
#define BMP388_REG_DATA_0          0x04     // Press XLSB
#define BMP388_REG_DATA_1          0x05     // Press LSB
#define BMP388_REG_DATA_2          0x06     // Press MSB
#define BMP388_REG_DATA_3          0x07     // Temp XLSB
#define BMP388_REG_DATA_4          0x08     // Temp LSB
#define BMP388_REG_DATA_5          0x09     // Temp MSB
#define BMP388_REG_PWR_CTRL        0x1B
#define BMP388_REG_OSR             0x1C
#define BMP388_REG_ODR             0x1D
#define BMP388_REG_CONFIG          0x1F
#define BMP388_REG_CALIB_00        0x31     // Calibration start
#define BMP388_REG_CMD             0x7E

// ─── BMP388 Constants ─────────────────────────────────────────
#define BMP388_CHIP_ID             0x50
#define BMP388_CMD_SOFTRESET       0xB6
#define BMP388_PWR_NORMAL_MODE     0x33     // Press+Temp ON, Normal mode

// ─── Calibration Data Structure ───────────────────────────────
typedef struct {
    // Temperature coefficients
    double T1, T2, T3;
    // Pressure coefficients
    double P1, P2, P3, P4, P5, P6, P7, P8, P9, P10, P11;
} bmp388_calib_t;

// ─── Device Handle ────────────────────────────────────────────
typedef struct {
    uint8_t          i2c_addr;
    bmp388_calib_t   calib;
} bmp388_dev_t;

// ─── Function Declarations ────────────────────────────────────
esp_err_t bmp388_i2c_master_init(void);
esp_err_t bmp388_init(bmp388_dev_t *dev, uint8_t addr);
esp_err_t bmp388_read_temperature_pressure(bmp388_dev_t *dev,
                                           double *temperature_c,
                                           double *pressure_pa);
                                           