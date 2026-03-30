#pragma once

#include <stdint.h>
#include "driver/spi_master.h"
#include "esp_err.h"

/* ── SPI defaults ─────────────────────────────────────────────────────────── */
#define LSM6DSO_SPI_FREQ_HZ       8000000  /* 8 MHz (max 10 MHz per datasheet) */

/* ── Register map ─────────────────────────────────────────────────────────── */
#define LSM6DSO_REG_WHO_AM_I      0x0F
#define LSM6DSO_REG_CTRL1_XL      0x10   /* Accelerometer control              */
#define LSM6DSO_REG_CTRL2_G       0x11   /* Gyroscope control                  */
#define LSM6DSO_REG_CTRL3_C       0x12   /* General settings (BDU, SW_RESET)   */
#define LSM6DSO_REG_STATUS        0x1E
#define LSM6DSO_REG_OUTX_L_G      0x22   /* Gyro output registers (6 bytes)    */
#define LSM6DSO_REG_OUTX_L_A      0x28   /* Accel output registers (6 bytes)   */

/* ── Constants ────────────────────────────────────────────────────────────── */
#define LSM6DSO_WHO_AM_I_VAL      0x6C   /* Expected WHO_AM_I response         */

/* ── SPI protocol ─────────────────────────────────────────────────────────── */
#define LSM6DSO_SPI_READ          0x80   /* OR with register address to read   */

/* CTRL3_C (0x12) */
#define LSM6DSO_SW_RESET          (1 << 0)
#define LSM6DSO_BDU               (1 << 6)  /* Block data update                */

/* CTRL1_XL (0x10) — accelerometer
 *   bits[7:4]: ODR_XL  (0100 = 104 Hz)
 *   bits[3:2]: FS_XL   (10   = ±4 g  → sensitivity 0.122 mg/LSB)
 *   bits[1:0]: reserved                                                        */
#define LSM6DSO_XL_ODR_104HZ      (0x04 << 4)
#define LSM6DSO_XL_FS_4G          (0x02 << 2)

/* CTRL2_G (0x11) — gyroscope
 *   bits[7:4]: ODR_G   (0100 = 104 Hz)
 *   bits[3:1]: FS_G    (110  = ±2000 dps → sensitivity 70 mdps/LSB)
 *   bit  [0]:  FS_125  (0    = disabled)                                       */
#define LSM6DSO_G_ODR_104HZ       (0x04 << 4)
#define LSM6DSO_G_FS_2000DPS      (0x06 << 1)

/* ── Sensitivity constants ────────────────────────────────────────────────── */
#define LSM6DSO_ACCEL_SENS_MG     0.122f   /* mg per LSB at ±4 g               */
#define LSM6DSO_GYRO_SENS_MDPS    70.0f    /* mdps per LSB at ±2000 dps        */

/* ── Driver configuration ─────────────────────────────────────────────────── */
typedef struct {
    spi_host_device_t spi_host;   /* e.g. SPI2_HOST (HSPI)                     */
    int               mosi_pin;
    int               miso_pin;
    int               sclk_pin;
    int               cs_pin;
    int               spi_freq_hz;
} lsm6dso_config_t;

/* ── Raw 16-bit output (signed ADC counts) ────────────────────────────────── */
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} lsm6dso_raw_t;

/* ── Public API ───────────────────────────────────────────────────────────── */

/**
 * @brief  Initialise SPI bus and device, verify WHO_AM_I, soft-reset the
 *         device, enable BDU, and configure accelerometer and gyroscope.
 *
 * @param  cfg  Pointer to driver configuration.
 * @return ESP_OK on success, otherwise an esp_err_t error code.
 */
esp_err_t lsm6dso_init(const lsm6dso_config_t *cfg);

/**
 * @brief  Remove SPI device and free bus resources.
 *
 * @param  host  SPI host used during init.
 * @return ESP_OK on success.
 */
esp_err_t lsm6dso_deinit(spi_host_device_t host);

/**
 * @brief  Read raw accelerometer counts (±4 g, 0.122 mg/LSB).
 *
 * @param  out  Pointer to lsm6dso_raw_t to receive X/Y/Z counts.
 * @return ESP_OK on success.
 */
esp_err_t lsm6dso_read_accel(lsm6dso_raw_t *out);

/**
 * @brief  Read raw gyroscope counts (±2000 dps, 70 mdps/LSB).
 *
 * @param  out  Pointer to lsm6dso_raw_t to receive X/Y/Z counts.
 * @return ESP_OK on success.
 */
esp_err_t lsm6dso_read_gyro(lsm6dso_raw_t *out);

/**
 * @brief  Read both accelerometer and gyroscope in one 12-byte burst.
 *
 * @param  accel  Pointer to lsm6dso_raw_t for accelerometer data.
 * @param  gyro   Pointer to lsm6dso_raw_t for gyroscope data.
 * @return ESP_OK on success.
 */
esp_err_t lsm6dso_read_all(lsm6dso_raw_t *accel, lsm6dso_raw_t *gyro);
