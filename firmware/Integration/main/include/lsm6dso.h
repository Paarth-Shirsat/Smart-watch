#pragma once

#include <stdint.h>
#include "esp_err.h"

// ─── LSM6DSO I2C Addresses ────────────────────────────────────
#define LSM6DSO_ADDR_LOW        0x6A    // SA0 = GND
#define LSM6DSO_ADDR_HIGH       0x6B    // SA0 = VCC

// ─── Register Map ─────────────────────────────────────────────
#define LSM6DSO_REG_WHO_AM_I        0x0F
#define LSM6DSO_REG_CTRL1_XL        0x10
#define LSM6DSO_REG_CTRL2_G         0x11
#define LSM6DSO_REG_CTRL3_C         0x12
#define LSM6DSO_REG_CTRL6_C         0x15
#define LSM6DSO_REG_CTRL7_G         0x16
#define LSM6DSO_REG_STATUS          0x1E
#define LSM6DSO_REG_OUT_TEMP_L      0x20
#define LSM6DSO_REG_OUT_TEMP_H      0x21
#define LSM6DSO_REG_OUTX_L_G        0x22
#define LSM6DSO_REG_OUTX_H_G        0x23
#define LSM6DSO_REG_OUTY_L_G        0x24
#define LSM6DSO_REG_OUTY_H_G        0x25
#define LSM6DSO_REG_OUTZ_L_G        0x26
#define LSM6DSO_REG_OUTZ_H_G        0x27
#define LSM6DSO_REG_OUTX_L_A        0x28
#define LSM6DSO_REG_OUTX_H_A        0x29
#define LSM6DSO_REG_OUTY_L_A        0x2A
#define LSM6DSO_REG_OUTY_H_A        0x2B
#define LSM6DSO_REG_OUTZ_L_A        0x2C
#define LSM6DSO_REG_OUTZ_H_A        0x2D

// ─── Constants ───────────────────────────────────────────────
#define LSM6DSO_WHO_AM_I_VAL        0x6C
#define LSM6DSO_CALIB_SAMPLES       512
#define LSM6DSO_ACCEL_SENS_2G       0.000061f
#define LSM6DSO_GYRO_SENS_250DPS    0.00875f

// ─── Structs ──────────────────────────────────────────────────
typedef struct {
    float x, y, z;
} lsm6dso_vec3_t;

typedef struct {
    lsm6dso_vec3_t accel_bias;
    lsm6dso_vec3_t gyro_bias;
} lsm6dso_calib_t;

typedef struct {
    uint8_t         i2c_addr;
    lsm6dso_calib_t calib;
    float           accel_sens;
    float           gyro_sens;
} lsm6dso_dev_t;

// ─── Function Declarations ────────────────────────────────────
esp_err_t lsm6dso_init(lsm6dso_dev_t *dev, uint8_t addr);
esp_err_t lsm6dso_calibrate(lsm6dso_dev_t *dev);
esp_err_t lsm6dso_read_data(lsm6dso_dev_t *dev,
                            lsm6dso_vec3_t *accel_g,
                            lsm6dso_vec3_t *gyro_dps,
                            float *temp_c);
void lsm6dso_compute_tilt(lsm6dso_vec3_t *a, float *roll, float *pitch);
