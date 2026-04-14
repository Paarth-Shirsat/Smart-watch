#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "max30102.h"

static const char *TAG = "MAX30102";

// ═══════════════════════════════════════════════════════════════
//  I2C HELPERS
// ═══════════════════════════════════════════════════════════════

static esp_err_t i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg,  true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd,
                        pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t i2c_read_regs(uint8_t addr, uint8_t reg,
                                uint8_t *buf, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true);
    if (len > 1)
        i2c_master_read(cmd, buf, len - 1, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, buf + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd,
                        pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t i2c_read_byte(uint8_t addr, uint8_t reg, uint8_t *val)
{
    return i2c_read_regs(addr, reg, val, 1);
}

// ═══════════════════════════════════════════════════════════════
//  I2C MASTER INIT
// ═══════════════════════════════════════════════════════════════

esp_err_t max30102_i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = I2C_MASTER_SDA_IO,
        .scl_io_num       = I2C_MASTER_SCL_IO,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (ret != ESP_OK) return ret;
    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

// ═══════════════════════════════════════════════════════════════
//  FILTERS
// ═══════════════════════════════════════════════════════════════

// High-pass IIR DC removal
// y[n] = (x[n] - x[n-1]) + alpha * y[n-1]
static float dc_filter(dc_filter_t *f, float x)
{
    float w_prev = f->w;
    f->w = x + f->alpha * f->w;
    return f->w - w_prev;
}

// Simple moving average — smooths shot noise
static float ma_filter(ma_filter_t *f, float x)
{
    f->sum -= f->buf[f->idx];
    f->buf[f->idx] = x;
    f->sum += x;
    f->idx = (f->idx + 1) % MAX30102_MA_SIZE;
    return f->sum / MAX30102_MA_SIZE;
}

// Peak-to-peak envelope tracker with soft decay.
// FIX: decay coefficient raised from 0.85→0.70 so a poisoned
// max/min (e.g. from the DC filter startup transient) drains in
// ~25 windows instead of never.
static void pp_update(pp_tracker_t *pp, float x)
{
    if (x > pp->max_val) pp->max_val = x;
    if (x < pp->min_val) pp->min_val = x;
    pp->age++;
    if (pp->age >= pp->reset_every) {
        pp->max_val = pp->max_val * 0.70f + x * 0.30f;  // was 0.85/0.15
        pp->min_val = pp->min_val * 0.70f + x * 0.30f;
        pp->age = 0;
    }
}

// ═══════════════════════════════════════════════════════════════
//  BEAT DETECTION
//  Dynamic threshold: fires at 30% above midpoint of recent P-P.
//  Refractory period blocks re-triggering for BEAT_MIN_MS after
//  each beat — suppresses dicrotic notch false triggers.
// ═══════════════════════════════════════════════════════════════

static bool detect_beat(max30102_dev_t *dev, float ir_f)
{
    bool beat = false;

    pp_update(&dev->pp_ir, ir_f);

    float pp_range = dev->pp_ir.max_val - dev->pp_ir.min_val;

    // Signal too weak — no valid heartbeat present
    if (pp_range < MAX30102_BEAT_MIN_AMPLITUDE) {
        dev->above_threshold = false;
        return false;
    }

    float midpoint  = dev->pp_ir.min_val + pp_range * 0.5f;
    float threshold = midpoint + pp_range * MAX30102_BEAT_THRESHOLD_FRAC;

    int64_t now_us        = esp_timer_get_time();
    int64_t since_last_ms = (now_us - dev->last_beat_us) / 1000LL;

    // Refractory period: ignore signal for MIN_MS after each beat
    bool in_refractory = (dev->last_beat_us != 0) &&
                         (since_last_ms < MAX30102_BEAT_MIN_MS);

    if (!in_refractory) {
        if (!dev->above_threshold && ir_f > threshold) {
            // Rising edge crossed threshold
            dev->above_threshold = true;

            if (dev->last_beat_us != 0 &&
                since_last_ms >= MAX30102_BEAT_MIN_MS &&
                since_last_ms <= MAX30102_BEAT_MAX_MS)
            {
                float instant_bpm = 60000.0f / (float)since_last_ms;

                // Plausibility check: reject if >40% from current smooth BPM
                bool plausible = true;
                if (dev->bpm_smooth > 20.0f) {
                    float ratio = instant_bpm / dev->bpm_smooth;
                    if (ratio < 0.6f || ratio > 1.4f) {
                        plausible = false;
                        ESP_LOGW("BEAT", "Rejected instant=%.1f smooth=%.1f",
                                 instant_bpm, dev->bpm_smooth);
                    }
                }

                if (plausible) {
                    if (dev->bpm_smooth < 10.0f) {
                        dev->bpm_smooth = instant_bpm;   // seed first value
                    } else {
                        dev->bpm_smooth += MAX30102_BPM_ALPHA *
                                           (instant_bpm - dev->bpm_smooth);
                    }
                    beat = true;
                }
            }
            dev->last_beat_us = now_us;

        } else if (dev->above_threshold && ir_f < midpoint) {
            // Signal fell back below midpoint — hysteresis reset
            dev->above_threshold = false;
        }
    }

    dev->last_ir_filtered = ir_f;
    return beat;
}

// ═══════════════════════════════════════════════════════════════
//  SpO2  (ratio-of-ratios, approximate ±3-5%)
// ═══════════════════════════════════════════════════════════════

static float compute_spo2(float red_ac, float red_dc,
                           float ir_ac,  float ir_dc)
{
    if (ir_dc < 1.0f || red_dc < 1.0f) return 0.0f;
    float R = (fabsf(red_ac) / red_dc) / (fabsf(ir_ac) / ir_dc);
    float spo2 = 110.0f - 25.0f * R;
    if (spo2 > 100.0f) spo2 = 100.0f;
    if (spo2 <  80.0f) return 0.0f;
    return spo2;
}

// ═══════════════════════════════════════════════════════════════
//  AUTO GAIN CONTROL
//  Binary-search the LED current PA register until IR raw sits
//  in [AGC_TARGET_LOW, AGC_TARGET_HIGH].
//  Must be called AFTER a finger is confirmed present.
// ═══════════════════════════════════════════════════════════════

static esp_err_t max30102_agc(max30102_dev_t *dev, uint8_t *pa_out)
{
    ESP_LOGI(TAG, "AGC: finding optimal LED current...");

    uint8_t pa    = 0x1F;   // start at ~6.4 mA
    int     settled = 0;

    for (int attempt = 0; attempt < 60; attempt++) {

        // Apply PA to both LEDs
        i2c_write_reg(dev->i2c_addr, MAX30102_REG_LED1_PA, pa);
        i2c_write_reg(dev->i2c_addr, MAX30102_REG_LED2_PA, pa);

        // Flush FIFO so the next read is with the new current
        i2c_write_reg(dev->i2c_addr, MAX30102_REG_FIFO_WR_PTR, 0x00);
        i2c_write_reg(dev->i2c_addr, MAX30102_REG_OVF_COUNTER,  0x00);
        i2c_write_reg(dev->i2c_addr, MAX30102_REG_FIFO_RD_PTR,  0x00);
        vTaskDelay(pdMS_TO_TICKS(60));   // ~3 samples at 50 sps

        // Check FIFO has data
        uint8_t wr = 0, rd = 0;
        i2c_read_byte(dev->i2c_addr, MAX30102_REG_FIFO_WR_PTR, &wr);
        i2c_read_byte(dev->i2c_addr, MAX30102_REG_FIFO_RD_PTR, &rd);
        wr &= 0x1F;
        rd &= 0x1F;

        if (((wr - rd + MAX30102_FIFO_DEPTH) % MAX30102_FIFO_DEPTH) == 0) {
            ESP_LOGW(TAG, "AGC: FIFO empty, retrying...");
            continue;
        }

        // Read one sample
        uint8_t raw[6] = {0};
        i2c_read_regs(dev->i2c_addr, MAX30102_REG_FIFO_DATA, raw, 6);
        uint32_t ir = ((uint32_t)(raw[3] & 0x03) << 16) |
                      ((uint32_t) raw[4]          <<  8) |
                       (uint32_t) raw[5];

        ESP_LOGI(TAG, "AGC: PA=0x%02X (%.1fmA)  IR=%-7"PRIu32
                 "  target=[%"PRIu32"–%"PRIu32"]",
                 pa, pa * 0.2f, ir,
                 (uint32_t)MAX30102_AGC_TARGET_LOW,
                 (uint32_t)MAX30102_AGC_TARGET_HIGH);

        if (ir >= MAX30102_AGC_TARGET_LOW &&
            ir <= MAX30102_AGC_TARGET_HIGH) {
            settled++;
            if (settled >= MAX30102_AGC_SETTLED_READS) {
                ESP_LOGI(TAG, "AGC done: PA=0x%02X (%.1fmA)  IR=%-7"PRIu32,
                         pa, pa * 0.2f, ir);
                *pa_out = pa;
                return ESP_OK;
            }
        } else {
            settled = 0;
            if (ir < MAX30102_AGC_TARGET_LOW) {
                // Too dark — step up
                uint8_t step = (pa < 0x10) ? 0x08 :
                               (pa < 0x40) ? 0x04 : 0x02;
                pa = (pa + step > 0xFF) ? 0xFF : pa + step;
            } else {
                // Saturated — step down
                uint8_t step = (pa > 0x40) ? 0x08 :
                               (pa > 0x10) ? 0x04 : 0x02;
                pa = (pa < step + 1) ? 0x01 : pa - step;
            }
        }
    }

    ESP_LOGW(TAG, "AGC: did not fully converge, using PA=0x%02X", pa);
    *pa_out = pa;
    return ESP_OK;
}

// ═══════════════════════════════════════════════════════════════
//  MAX30102 INIT
// ═══════════════════════════════════════════════════════════════

esp_err_t max30102_init(max30102_dev_t *dev)
{
    memset(dev, 0, sizeof(*dev));
    dev->i2c_addr          = MAX30102_I2C_ADDR;
    dev->dc_red.alpha      = MAX30102_DC_FILTER_ALPHA;
    dev->dc_ir.alpha       = MAX30102_DC_FILTER_ALPHA;
    dev->ir_threshold      = MAX30102_IR_FINGER_THRESHOLD;
    dev->bpm_smooth        = 0.0f;
    dev->last_beat_us      = 0;
    dev->above_threshold   = false;
    dev->settled           = false;
    dev->settle_count      = 0;

    // FIX: init pp tracker with a tiny real window (not 0/0) and
    // faster adapt window (25 instead of 50).
    dev->pp_ir.max_val     = 1.0f;
    dev->pp_ir.min_val     = -1.0f;
    dev->pp_ir.age         = 0;
    dev->pp_ir.reset_every = 25;   // was 50

    // 1. Verify Part ID
    uint8_t part_id = 0;
    esp_err_t ret = i2c_read_byte(dev->i2c_addr,
                                   MAX30102_REG_PART_ID, &part_id);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C failed: %s", esp_err_to_name(ret));
        return ret;
    }
    if (part_id != MAX30102_PART_ID) {
        ESP_LOGE(TAG, "Wrong Part ID: 0x%02X (expected 0x%02X)",
                 part_id, MAX30102_PART_ID);
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "MAX30102 found, Part ID: 0x%02X", part_id);

    // 2. Software reset — poll until bit self-clears (~2 ms)
    ret = i2c_write_reg(dev->i2c_addr,
                         MAX30102_REG_MODE_CONFIG, MAX30102_MODE_RESET);
    if (ret != ESP_OK) return ret;
    uint8_t mode_val = MAX30102_MODE_RESET;
    for (int i = 0; i < 20; i++) {
        vTaskDelay(pdMS_TO_TICKS(10));
        i2c_read_byte(dev->i2c_addr, MAX30102_REG_MODE_CONFIG, &mode_val);
        if (!(mode_val & MAX30102_MODE_RESET)) break;
    }

    // 3. FIFO config:
    //    SMP_AVE=4  (bits[7:5]=010)  — hardware averages 4 ADC samples
    //    ROLLOVER=1 (bit4)           — old data silently overwritten
    //    A_FULL=0xF (bits[3:0])
    ret = i2c_write_reg(dev->i2c_addr, MAX30102_REG_FIFO_CONFIG, 0x5F);
    if (ret != ESP_OK) return ret;

    // 4. SpO2 config: ADC 4096nA, 100 sps, 18-bit / 411 µs pulse width
    ret = i2c_write_reg(dev->i2c_addr, MAX30102_REG_SPO2_CONFIG, 0x27);
    if (ret != ESP_OK) return ret;

    // 5. LED: conservative 6.4 mA start — AGC tunes after finger placed
    ret = i2c_write_reg(dev->i2c_addr, MAX30102_REG_LED1_PA, 0x1F);
    if (ret != ESP_OK) return ret;
    ret = i2c_write_reg(dev->i2c_addr, MAX30102_REG_LED2_PA, 0x1F);
    if (ret != ESP_OK) return ret;

    // 6. SpO2 mode (Red + IR LEDs)
    ret = i2c_write_reg(dev->i2c_addr,
                         MAX30102_REG_MODE_CONFIG, MAX30102_MODE_SPO2);
    if (ret != ESP_OK) return ret;

    // 7. Clear FIFO pointers
    i2c_write_reg(dev->i2c_addr, MAX30102_REG_FIFO_WR_PTR, 0x00);
    i2c_write_reg(dev->i2c_addr, MAX30102_REG_OVF_COUNTER,  0x00);
    i2c_write_reg(dev->i2c_addr, MAX30102_REG_FIFO_RD_PTR,  0x00);

    vTaskDelay(pdMS_TO_TICKS(200));
    ESP_LOGI(TAG, "MAX30102 ready — place finger for AGC");
    return ESP_OK;
}

// ═══════════════════════════════════════════════════════════════
//  FINGER DETECTION
// ═══════════════════════════════════════════════════════════════

bool max30102_finger_present(max30102_dev_t *dev, uint32_t ir_raw)
{
    return (ir_raw >= dev->ir_threshold);
}

// ═══════════════════════════════════════════════════════════════
//  READ FIFO
// ═══════════════════════════════════════════════════════════════

esp_err_t max30102_read_fifo(max30102_dev_t *dev, max30102_data_t *out)
{
    memset(out, 0, sizeof(*out));

    uint8_t wr_ptr = 0, rd_ptr = 0;
    esp_err_t ret;
    ret = i2c_read_byte(dev->i2c_addr, MAX30102_REG_FIFO_WR_PTR, &wr_ptr);
    if (ret != ESP_OK) return ret;
    ret = i2c_read_byte(dev->i2c_addr, MAX30102_REG_FIFO_RD_PTR, &rd_ptr);
    if (ret != ESP_OK) return ret;

    wr_ptr &= 0x1F;
    rd_ptr &= 0x1F;

    int num_samples = ((int)wr_ptr - (int)rd_ptr + MAX30102_FIFO_DEPTH)
                       % MAX30102_FIFO_DEPTH;

    if (num_samples == 0) return ESP_ERR_NOT_FOUND;
    if (num_samples > MAX30102_FIFO_DEPTH) num_samples = MAX30102_FIFO_DEPTH;

    bool any_beat = false;

    for (int s = 0; s < num_samples; s++) {
        uint8_t raw[6] = {0};
        ret = i2c_read_regs(dev->i2c_addr, MAX30102_REG_FIFO_DATA, raw, 6);
        if (ret != ESP_OK) return ret;

        // 18-bit: top byte only has bits[1:0] valid
        uint32_t red = ((uint32_t)(raw[0] & 0x03) << 16) |
                       ((uint32_t) raw[1]          <<  8) |
                        (uint32_t) raw[2];
        uint32_t ir  = ((uint32_t)(raw[3] & 0x03) << 16) |
                       ((uint32_t) raw[4]          <<  8) |
                        (uint32_t) raw[5];

        out->red_raw = red;
        out->ir_raw  = ir;

        // Stage 1: DC removal
        float red_ac = dc_filter(&dev->dc_red, (float)red);
        float ir_ac  = dc_filter(&dev->dc_ir,  (float)ir);
        out->red_ac  = red_ac;
        out->ir_ac   = ir_ac;

        // Stage 2: noise smoothing
        float red_f = ma_filter(&dev->ma_red, red_ac);
        float ir_f  = ma_filter(&dev->ma_ir,  ir_ac);
        out->red_filtered = red_f;
        out->ir_filtered  = ir_f;

        // Stage 3: settling then beat detection
        if (!dev->settled) {
            dev->settle_count++;
            pp_update(&dev->pp_ir, ir_f);   // warm up pp tracker
            if (dev->settle_count >= MAX30102_SETTLE_SAMPLES) {
                dev->settled = true;

                // FIX: reset pp completely to zero so the DC filter
                // startup transient (which can be 100k+) does not
                // poison the beat threshold. Tracker will rebuild
                // from real pulse signal within a few beats.
                dev->pp_ir.max_val = 0.0f;
                dev->pp_ir.min_val = 0.0f;
                dev->pp_ir.age     = 0;

                // FIX: also clear beat state so a stale last_beat_us
                // from before settling doesn't corrupt the first BPM.
                dev->bpm_smooth      = 0.0f;
                dev->last_beat_us    = 0;
                dev->above_threshold = false;

                ESP_LOGI(TAG, "DC settled — beat detection active");
            }
        } else {
            if (detect_beat(dev, ir_f)) any_beat = true;
        }

        out->bpm          = dev->bpm_smooth;
        out->spo2         = compute_spo2(red_ac, (float)red,
                                          ir_ac,  (float)ir);
        out->peak_to_peak = dev->pp_ir.max_val - dev->pp_ir.min_val;
    }

    out->beat_detected = any_beat;
    return ESP_OK;
}

// ═══════════════════════════════════════════════════════════════
//  TEMPERATURE
// ═══════════════════════════════════════════════════════════════

esp_err_t max30102_read_temperature(max30102_dev_t *dev, float *temp_c)
{
    esp_err_t ret = i2c_write_reg(dev->i2c_addr,
                                   MAX30102_REG_TEMP_CONFIG, 0x01);
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(35));
    uint8_t t_int = 0, t_frac = 0;
    ret = i2c_read_byte(dev->i2c_addr, MAX30102_REG_TEMP_INT,  &t_int);
    if (ret != ESP_OK) return ret;
    ret = i2c_read_byte(dev->i2c_addr, MAX30102_REG_TEMP_FRAC, &t_frac);
    if (ret != ESP_OK) return ret;
    *temp_c = (float)(int8_t)t_int + ((float)(t_frac & 0x0F) * 0.0625f);
    return ESP_OK;
}

// ═══════════════════════════════════════════════════════════════
//  HELPER — full sensor state reset (called on finger removal)
// ═══════════════════════════════════════════════════════════════

static void sensor_state_reset(max30102_dev_t *dev)
{
    dev->bpm_smooth      = 0.0f;
    dev->last_beat_us    = 0;
    dev->above_threshold = false;
    dev->settled         = false;
    dev->settle_count    = 0;
    dev->pp_ir.max_val   = 1.0f;   // FIX: match init values (not 0/0)
    dev->pp_ir.min_val   = -1.0f;
    dev->pp_ir.age       = 0;
    dev->dc_red.w        = 0.0f;
    dev->dc_ir.w         = 0.0f;
    memset(&dev->ma_red, 0, sizeof(dev->ma_red));
    memset(&dev->ma_ir,  0, sizeof(dev->ma_ir));
    // Reset LED to safe low current for next finger placement
    i2c_write_reg(dev->i2c_addr, MAX30102_REG_LED1_PA, 0x1F);
    i2c_write_reg(dev->i2c_addr, MAX30102_REG_LED2_PA, 0x1F);
}

// ═══════════════════════════════════════════════════════════════
//  APP MAIN
// ═══════════════════════════════════════════════════════════════

void app_main(void)
{
    ESP_LOGI(TAG, "Initializing I2C: SDA=GPIO%d SCL=GPIO%d",
             I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);

    ESP_ERROR_CHECK(max30102_i2c_master_init());

    max30102_dev_t sensor;
    ESP_ERROR_CHECK(max30102_init(&sensor));

    float sensor_temp = 0.0f;
    if (max30102_read_temperature(&sensor, &sensor_temp) == ESP_OK) {
        ESP_LOGI(TAG, "Sensor die temp: %.2f °C", sensor_temp);
    }

    ESP_LOGI(TAG, "Ready — place finger gently and hold still");

    max30102_data_t data;
    uint32_t loop_count    = 0;
    uint32_t beat_count    = 0;
    bool finger_was_absent = true;
    uint8_t current_pa     = 0x1F;

    while (1) {
        esp_err_t ret = max30102_read_fifo(&sensor, &data);

        // ── FIFO not ready ────────────────────────────────────
        if (ret == ESP_ERR_NOT_FOUND) {
            if (loop_count++ % 50 == 0)
                ESP_LOGI(TAG, "Waiting for FIFO...");
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "FIFO error: %s", esp_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        bool finger = max30102_finger_present(&sensor, data.ir_raw);

        // ── No finger ─────────────────────────────────────────
        if (!finger) {
            if (!finger_was_absent) {
                ESP_LOGW(TAG, "Finger removed");
                sensor_state_reset(&sensor);
                beat_count        = 0;
                finger_was_absent = true;
            }
            ESP_LOGI(TAG, "No finger | IR=%-7"PRIu32" RED=%-7"PRIu32
                     "  (threshold=%"PRIu32")",
                     data.ir_raw, data.red_raw, sensor.ir_threshold);
            loop_count++;
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        // ── Finger just placed → run AGC first ───────────────
        if (finger_was_absent) {
            ESP_LOGI(TAG, "Finger detected! IR=%-7"PRIu32, data.ir_raw);
            finger_was_absent = false;

            // AGC: finds correct LED current so ADC is not saturated
            max30102_agc(&sensor, &current_pa);

            // FIX: full filter + beat state reset after AGC.
            // bpm_smooth and last_beat_us must be cleared here too,
            // not just at finger removal — AGC changes the light level
            // so any state from before is meaningless.
            sensor.dc_red.w        = 0.0f;
            sensor.dc_ir.w         = 0.0f;
            sensor.settled         = false;
            sensor.settle_count    = 0;
            sensor.bpm_smooth      = 0.0f;
            sensor.last_beat_us    = 0;
            sensor.above_threshold = false;
            sensor.pp_ir.max_val   = 1.0f;
            sensor.pp_ir.min_val   = -1.0f;
            sensor.pp_ir.age       = 0;
            memset(&sensor.ma_red, 0, sizeof(sensor.ma_red));
            memset(&sensor.ma_ir,  0, sizeof(sensor.ma_ir));

            // Flush FIFO of AGC-era samples
            i2c_write_reg(sensor.i2c_addr, MAX30102_REG_FIFO_WR_PTR, 0x00);
            i2c_write_reg(sensor.i2c_addr, MAX30102_REG_OVF_COUNTER,  0x00);
            i2c_write_reg(sensor.i2c_addr, MAX30102_REG_FIFO_RD_PTR,  0x00);
            vTaskDelay(pdMS_TO_TICKS(50));

            ESP_LOGI(TAG, "AGC done (PA=0x%02X, %.1fmA) — settling filters...",
                     current_pa, current_pa * 0.2f);
            loop_count++;
            continue;
        }

        // ── Beat event ────────────────────────────────────────
        if (data.beat_detected) {
            beat_count++;
            ESP_LOGI(TAG, "♥  BEAT #%-3"PRIu32
                     "  BPM: %.1f  P-P: %.1f",
                     beat_count, data.bpm, data.peak_to_peak);
        }

        // ── Periodic full data printout ───────────────────────
        if (loop_count % 5 == 0) {
            if (!sensor.settled) {
                // During settling — show progress so user knows it's working
                ESP_LOGI(TAG,
                    "[Settling %3d/%-3d] IR_RAW=%-7"PRIu32
                    " AC=%+7.1f  Filt=%+7.1f",
                    sensor.settle_count, MAX30102_SETTLE_SAMPLES,
                    data.ir_raw, data.ir_ac, data.ir_filtered);
            } else {
                // Normal operation
                ESP_LOGI(TAG,
                    "RAW  R=%-6"PRIu32" IR=%-6"PRIu32
                    " | AC  R=%+6.0f IR=%+6.0f"
                    " | Filt R=%+6.0f IR=%+6.0f"
                    " | P-P=%.1f",
                    data.red_raw,      data.ir_raw,
                    data.red_ac,       data.ir_ac,
                    data.red_filtered, data.ir_filtered,
                    data.peak_to_peak);

                if (data.bpm > 20.0f) {
                    ESP_LOGI(TAG,
                        ">>> BPM: %.1f  |  SpO2: %.1f%% <<<",
                        data.bpm,
                        data.spo2 > 0.0f ? data.spo2 : 0.0f);
                } else {
                    ESP_LOGI(TAG,
                        "Collecting beats... P-P=%.1f  PA=0x%02X (%.1fmA)",
                        data.peak_to_peak, current_pa,
                        current_pa * 0.2f);
                }
            }
        }

        loop_count++;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}