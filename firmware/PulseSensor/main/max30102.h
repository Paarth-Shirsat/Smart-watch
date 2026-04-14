#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "driver/i2c.h"
#include "esp_err.h"

// ─── I2C Config ───────────────────────────────────────────────
#define I2C_MASTER_NUM          I2C_NUM_0
#define I2C_MASTER_SDA_IO       8
#define I2C_MASTER_SCL_IO       9
#define I2C_MASTER_FREQ_HZ      400000
#define I2C_MASTER_TIMEOUT_MS   1000

// ─── MAX30102 I2C Address ─────────────────────────────────────
#define MAX30102_I2C_ADDR       0x57

// ─── Register Map ─────────────────────────────────────────────
#define MAX30102_REG_INT_STATUS1    0x00
#define MAX30102_REG_INT_STATUS2    0x01
#define MAX30102_REG_INT_ENABLE1    0x02
#define MAX30102_REG_INT_ENABLE2    0x03
#define MAX30102_REG_FIFO_WR_PTR    0x04
#define MAX30102_REG_OVF_COUNTER    0x05
#define MAX30102_REG_FIFO_RD_PTR    0x06
#define MAX30102_REG_FIFO_DATA      0x07
#define MAX30102_REG_FIFO_CONFIG    0x08
#define MAX30102_REG_MODE_CONFIG    0x09
#define MAX30102_REG_SPO2_CONFIG    0x0A
#define MAX30102_REG_LED1_PA        0x0C
#define MAX30102_REG_LED2_PA        0x0D
#define MAX30102_REG_PILOT_PA       0x10
#define MAX30102_REG_MULTI_LED1     0x11
#define MAX30102_REG_MULTI_LED2     0x12
#define MAX30102_REG_TEMP_INT       0x1F
#define MAX30102_REG_TEMP_FRAC      0x20
#define MAX30102_REG_TEMP_CONFIG    0x21
#define MAX30102_REG_REVISION       0xFE
#define MAX30102_REG_PART_ID        0xFF

#define MAX30102_PART_ID            0x15
#define MAX30102_MODE_HR_ONLY       0x02
#define MAX30102_MODE_SPO2          0x03
#define MAX30102_MODE_RESET         0x40
#define MAX30102_MODE_SHDN          0x80
#define MAX30102_FIFO_DEPTH         32

// ─── DC removal ───────────────────────────────────────────────
// Lowered from 0.93 → 0.85: converges faster after AGC so the
// startup transient drains before beat detection activates.
// Also preserves more AC amplitude of the slow ~1 Hz pulse wave.
#define MAX30102_DC_FILTER_ALPHA    0.85f

// ─── Noise smoothing ──────────────────────────────────────────
#define MAX30102_MA_SIZE            4

// ─── Auto-gain targets ────────────────────────────────────────
#define MAX30102_AGC_TARGET_LOW     150000UL
#define MAX30102_AGC_TARGET_HIGH    180000UL   // lowered from 220000 — avoids near-saturation
#define MAX30102_AGC_SETTLED_READS  8

// ─── Beat detection ───────────────────────────────────────────
#define MAX30102_BEAT_THRESHOLD_FRAC  0.3f
#define MAX30102_BEAT_MIN_AMPLITUDE   50.0f    // raised from 8.0 — requires real pulse signal
#define MAX30102_BEAT_MIN_MS          300
#define MAX30102_BEAT_MAX_MS          2000

// ─── BPM smoothing ────────────────────────────────────────────
#define MAX30102_BPM_ALPHA          0.25f      // raised from 0.15 — more responsive

// ─── Finger detection ─────────────────────────────────────────
#define MAX30102_IR_FINGER_THRESHOLD  5000

// ─── Settling samples before beat detection activates ─────────
#define MAX30102_SETTLE_SAMPLES     150        // raised from 100 — ~3s, ensures DC fully converges

// ─── Structs ──────────────────────────────────────────────────

typedef struct {
    float w;
    float alpha;
} dc_filter_t;

typedef struct {
    float  buf[MAX30102_MA_SIZE];
    int    idx;
    float  sum;
} ma_filter_t;

// Peak-to-peak tracker for dynamic threshold
typedef struct {
    float  max_val;
    float  min_val;
    int    age;         // samples since last reset
    int    reset_every; // reset window in samples
} pp_tracker_t;

typedef struct {
    uint32_t red_raw;
    uint32_t ir_raw;
    float    red_ac;
    float    ir_ac;
    float    red_filtered;
    float    ir_filtered;
    float    bpm;
    bool     beat_detected;
    float    spo2;
    float    peak_to_peak;   // current IR AC peak-to-peak (debug)
} max30102_data_t;

typedef struct {
    uint8_t       i2c_addr;
    dc_filter_t   dc_red;
    dc_filter_t   dc_ir;
    ma_filter_t   ma_red;
    ma_filter_t   ma_ir;
    pp_tracker_t  pp_ir;      // peak-to-peak tracker for IR

    // Beat detector state
    float    last_ir_filtered;
    bool     above_threshold;  // hysteresis state
    int64_t  last_beat_us;
    float    bpm_smooth;

    uint32_t ir_threshold;
    int      settle_count;     // counts up to SETTLE_SAMPLES
    bool     settled;
} max30102_dev_t;

esp_err_t max30102_i2c_master_init(void);
esp_err_t max30102_init(max30102_dev_t *dev);
esp_err_t max30102_read_fifo(max30102_dev_t *dev, max30102_data_t *out);
esp_err_t max30102_read_temperature(max30102_dev_t *dev, float *temp_c);
bool      max30102_finger_present(max30102_dev_t *dev, uint32_t ir_raw);