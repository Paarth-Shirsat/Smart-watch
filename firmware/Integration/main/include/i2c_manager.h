#pragma once

#include "driver/i2c.h"
#include "esp_err.h"

// ─── I2C Configuration ─────────────────────────────────────────
#define I2C_MASTER_NUM          I2C_NUM_0
#define I2C_MASTER_SDA_IO       8
#define I2C_MASTER_SCL_IO       9
#define I2C_MASTER_FREQ_HZ      400000
#define I2C_MASTER_TIMEOUT_MS   100

// ─── Shared Helper Functions ───────────────────────────────────
/**
 * @brief Initialize the shared I2C bus
 */
esp_err_t i2c_manager_init(void);

/**
 * @brief Write a single byte to a register
 */
esp_err_t i2c_manager_write_reg(uint8_t addr, uint8_t reg, uint8_t data);

/**
 * @brief Write raw bytes to a device (no register prefix — used for UBX frames)
 */
esp_err_t i2c_manager_write_bytes(uint8_t addr, const uint8_t *buf, size_t len);

/**
 * @brief Read multiple bytes from a register
 */
esp_err_t i2c_manager_read_regs(uint8_t addr, uint8_t reg, uint8_t *buf, size_t len);

/**
 * @brief Bare stream read — no register address phase (used for DDC/UBX stream)
 */
esp_err_t i2c_manager_stream_read(uint8_t addr, uint8_t *buf, size_t len);
