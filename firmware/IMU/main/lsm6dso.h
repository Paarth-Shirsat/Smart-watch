#pragma once

#include <stdint.h>
#include "driver/i2c.h"
#include "esp_err.h"

// ─── I2C Config ───────────────────────────────────────────────
#define I2C_MASTER_NUM          I2C_NUM_0
#define I2C_MASTER_SDA_IO       8
#define I2C_MASTER_SCL_IO       9
#define I2C_MASTER_FREQ_HZ      400000
#define I2C_MASTER_TIMEOUT_MS   1000

// ─── LSM6DSO I2C Addresses ────────────────────────────────────
#define LSM6DSO_ADDR_LOW        0x6A    // SA0 = GND
#define LSM6DSO_ADDR_HIGH       0x6B    // SA0 = VCC

// ─── Register Map ─────────────────────────────────────────────
#define LSM6DSO_REG_WHO_AM_I        0x0F
#define LSM6DSO_REG_CTRL1_XL        0x10    // Accel control
#define LSM6DSO_REG_CTRL2_G         0x11    // Gyro control
#define LSM6DSO_REG_CTRL3_C         0x12    // General control
#define LSM6DSO_REG_CTRL6_C         0x15    // Accel high-perf mode
#define LSM6DSO_REG_CTRL7_G         0x16    // Gyro high-perf mode
#define LSM6DSO_REG_STATUS          0x1E
#define LSM6DSO_REG_OUT_TEMP_L      0x20
#define LSM6DSO_REG_OUT_TEMP_H      0x21
#define LSM6DSO_REG_OUTX_L_G        0x22    // Gyro X LSB
#define LSM6DSO_REG_OUTX_H_G        0x23
#define LSM6DSO_REG_OUTY_L_G        0x24
#define LSM6DSO_REG_OUTY_H_G        0x25
#define LSM6DSO_REG_OUTZ_L_G        0x26
#define LSM6DSO_REG_OUTZ_H_G        0x27
#define LSM6DSO_REG_OUTX_L_A        0x28    // Accel X LSB
#define LSM6DSO_REG_OUTX_H_A        0x29
#define LSM6DSO_REG_OUTY_L_A        0x2A
#define LSM6DSO_REG_OUTY_H_A        0x2B
#define LSM6DSO_REG_OUTZ_L_A        0x2C
#define LSM6DSO_REG_OUTZ_H_A        0x2D

// ─── WHO_AM_I expected value ──────────────────────────────────
#define LSM6DSO_WHO_AM_I_VAL        0x6C

// ─── Calibration Config ───────────────────────────────────────
// Number of samples to average during calibration (sensor must be still & flat)
#define LSM6DSO_CALIB_SAMPLES       512

// ─── Sensitivity (depends on FS selection) ───────────────────
// Accel: ±2g  → 0.061 mg/LSB
// Gyro:  ±250 dps → 8.75 mdps/LSB
#define LSM6DSO_ACCEL_SENS_2G       0.000061f   // g/LSB
#define LSM6DSO_GYRO_SENS_250DPS    0.00875f    // dps/LSB

// ─── Structs ──────────────────────────────────────────────────

typedef struct {
    float x, y, z;
} lsm6dso_vec3_t;

typedef struct {
    lsm6dso_vec3_t accel_bias;  // in g
    lsm6dso_vec3_t gyro_bias;   // in dps
} lsm6dso_calib_t;

typedef struct {
    uint8_t         i2c_addr;
    lsm6dso_calib_t calib;
    float           accel_sens;     // g/LSB
    float           gyro_sens;      // dps/LSB
} lsm6dso_dev_t;

// ─── Function Declarations ────────────────────────────────────
esp_err_t lsm6dso_i2c_master_init(void);
esp_err_t lsm6dso_init(lsm6dso_dev_t *dev, uint8_t addr);
esp_err_t lsm6dso_calibrate(lsm6dso_dev_t *dev);
esp_err_t lsm6dso_read_raw(lsm6dso_dev_t *dev,
                            lsm6dso_vec3_t *accel_g,
                            lsm6dso_vec3_t *gyro_dps,
                            float *temp_c);