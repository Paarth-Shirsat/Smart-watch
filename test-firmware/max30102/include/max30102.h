#pragma once

#include <stddef.h>
#include <stdint.h>
#include "driver/i2c.h"
#include "esp_err.h"

/* ── I2C defaults ─────────────────────────────────────────────────────────── */
#define MAX30102_I2C_ADDR         0x57   /* Fixed — no alternate address       */
#define MAX30102_I2C_FREQ_HZ      400000 /* 400 kHz fast mode                  */

/* ── Register map ─────────────────────────────────────────────────────────── */
#define MAX30102_REG_INT_STATUS1  0x00
#define MAX30102_REG_INT_STATUS2  0x01
#define MAX30102_REG_INT_ENABLE1  0x02
#define MAX30102_REG_INT_ENABLE2  0x03
#define MAX30102_REG_FIFO_WR_PTR 0x04
#define MAX30102_REG_OVF_COUNTER  0x05
#define MAX30102_REG_FIFO_RD_PTR 0x06
#define MAX30102_REG_FIFO_DATA    0x07
#define MAX30102_REG_FIFO_CONFIG  0x08
#define MAX30102_REG_MODE_CONFIG  0x09
#define MAX30102_REG_SPO2_CONFIG  0x0A
#define MAX30102_REG_LED1_PA      0x0C   /* Red LED pulse amplitude           */
#define MAX30102_REG_LED2_PA      0x0D   /* IR  LED pulse amplitude           */
#define MAX30102_REG_REV_ID       0xFE
#define MAX30102_REG_PART_ID      0xFF

/* ── Constants ────────────────────────────────────────────────────────────── */
#define MAX30102_PART_ID          0x15
#define MAX30102_FIFO_DEPTH       32     /* FIFO holds up to 32 samples        */

/* MODE_CONFIG (0x09)
 *   bit  7    = SHDN  (0 = active)
 *   bit  6    = RESET (self-clearing)
 *   bits[2:0] = MODE  (010=HR-only, 011=SpO2, 111=multi-LED)                 */
#define MAX30102_MODE_RESET       (1 << 6)
#define MAX30102_MODE_SPO2        0x03   /* Red + IR, two channels             */
#define MAX30102_MODE_HR          0x02   /* Red only, one channel              */

/* SPO2_CONFIG (0x0A)
 *   bits[6:5] = ADC_RGE  (00=2048 nA, 01=4096 nA, 10=8192 nA, 11=16384 nA)
 *   bits[4:2] = SR       (000=50 SPS … 111=3200 SPS)
 *   bits[1:0] = LED_PW   (00=69 µs/15-bit … 11=411 µs/18-bit)               */
#define MAX30102_SPO2_ADC_RGE_4096  (0x01 << 5)
#define MAX30102_SPO2_SR_100        (0x01 << 2)
#define MAX30102_SPO2_PW_411US      0x03   /* 18-bit ADC resolution            */

/* FIFO_CONFIG (0x08)
 *   bits[7:5] = SMP_AVE         (000=1, 001=2, 010=4, 011=8, …)
 *   bit  4    = FIFO_ROLLOVER_EN
 *   bits[3:0] = FIFO_A_FULL     (empty slots that trigger A_FULL interrupt)  */
#define MAX30102_SMP_AVE_4        (0x02 << 5)
#define MAX30102_FIFO_ROLLOVER_EN (1 << 4)

/* LED pulse amplitude — each LSB = 200 µA.
 *   0x24 = 36 × 200 µA = 7.2 mA  (safe default for both Red and IR)         */
#define MAX30102_LED_PA_7MA       0x24

/* ── Driver configuration ─────────────────────────────────────────────────── */
typedef struct {
    i2c_port_t i2c_port;
    int        sda_pin;
    int        scl_pin;
    uint32_t   i2c_freq;
    uint8_t    led_pa;   /* Pulse amplitude for both LEDs (Red & IR)          */
} max30102_config_t;

/* ── Per-sample data (raw 18-bit ADC counts) ──────────────────────────────── */
typedef struct {
    uint32_t red;  /* Red LED (660 nm) — proportional to oxygenated Hb       */
    uint32_t ir;   /* IR  LED (880 nm) — proportional to total Hb            */
} max30102_sample_t;

/* ── Public API ───────────────────────────────────────────────────────────── */

/**
 * @brief  Install I2C driver, verify part ID, reset the device, configure
 *         SpO2 mode with 100 SPS / 18-bit / 4× averaging, and set LED current.
 *
 * @param  cfg  Pointer to driver configuration.
 * @return ESP_OK on success, otherwise an esp_err_t error code.
 */
esp_err_t max30102_init(const max30102_config_t *cfg);

/**
 * @brief  Uninstall I2C driver and release resources.
 *
 * @param  port  I2C port used during init.
 * @return ESP_OK on success.
 */
esp_err_t max30102_deinit(i2c_port_t port);

/**
 * @brief  Read all available samples from the FIFO into a caller-supplied buffer.
 *
 *         Computes the number of new samples by comparing FIFO_WR_PTR and
 *         FIFO_RD_PTR, then burst-reads that many 6-byte entries.  Stops early
 *         if the buffer is full.
 *
 * @param  port       I2C port used during init.
 * @param  addr       Device I2C address (MAX30102_I2C_ADDR).
 * @param  buf        Caller-allocated array of max30102_sample_t.
 * @param  buf_len    Number of entries in buf.
 * @param  out_count  Set to the number of samples actually written into buf.
 * @return ESP_OK on success.
 */
esp_err_t max30102_read_fifo(i2c_port_t port, uint8_t addr,
                              max30102_sample_t *buf, size_t buf_len,
                              size_t *out_count);
