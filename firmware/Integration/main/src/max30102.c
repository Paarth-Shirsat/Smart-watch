#include "max30102.h"
#include "i2c_manager.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <math.h>
#include <inttypes.h>

static const char *TAG = "MAX30102_DRV";

// ─── Internal Filters ─────────────────────────────────────────

static float dc_filter(dc_filter_t *f, float x)
{
    float w_prev = f->w;
    f->w = x + f->alpha * f->w;
    return f->w - w_prev;
}

static float ma_filter(ma_filter_t *f, float x)
{
    f->sum -= f->buf[f->idx];
    f->buf[f->idx] = x;
    f->sum += x;
    f->idx = (f->idx + 1) % MAX30102_MA_SIZE;
    return f->sum / MAX30102_MA_SIZE;
}

static void pp_update(pp_tracker_t *pp, float x)
{
    if (x > pp->max_val) pp->max_val = x;
    if (x < pp->min_val) pp->min_val = x;
    pp->age++;
    if (pp->age >= pp->reset_every) {
        pp->max_val = pp->max_val * 0.70f + x * 0.30f;
        pp->min_val = pp->min_val * 0.70f + x * 0.30f;
        pp->age = 0;
    }
}

// ─── Beat Detection ───────────────────────────────────────────

static bool detect_beat(max30102_dev_t *dev, float ir_f)
{
    bool beat = false;
    pp_update(&dev->pp_ir, ir_f);
    float pp_range = dev->pp_ir.max_val - dev->pp_ir.min_val;

    if (pp_range < MAX30102_BEAT_MIN_AMPLITUDE) {
        dev->above_threshold = false;
        return false;
    }

    float midpoint  = dev->pp_ir.min_val + pp_range * 0.5f;
    float threshold = midpoint + pp_range * MAX30102_BEAT_THRESHOLD_FRAC;

    int64_t now_us        = esp_timer_get_time();
    int64_t since_last_ms = (now_us - dev->last_beat_us) / 1000LL;
    bool in_refractory    = (dev->last_beat_us != 0) &&
                            (since_last_ms < MAX30102_BEAT_MIN_MS);

    if (!in_refractory) {
        if (!dev->above_threshold && ir_f > threshold) {
            dev->above_threshold = true;
            if (dev->last_beat_us != 0 &&
                since_last_ms >= MAX30102_BEAT_MIN_MS &&
                since_last_ms <= MAX30102_BEAT_MAX_MS)
            {
                float instant_bpm = 60000.0f / (float)since_last_ms;
                bool plausible = true;
                if (dev->bpm_smooth > 20.0f) {
                    float ratio = instant_bpm / dev->bpm_smooth;
                    if (ratio < 0.6f || ratio > 1.4f) plausible = false;
                }
                if (plausible) {
                    if (dev->bpm_smooth < 10.0f)
                        dev->bpm_smooth = instant_bpm;
                    else
                        dev->bpm_smooth += MAX30102_BPM_ALPHA * (instant_bpm - dev->bpm_smooth);
                    beat = true;
                }
            }
            dev->last_beat_us = now_us;
        } else if (dev->above_threshold && ir_f < midpoint) {
            dev->above_threshold = false;
        }
    }
    dev->last_ir_filtered = ir_f;
    return beat;
}

// ─── SpO2 ─────────────────────────────────────────────────────

static float compute_spo2(float red_ac, float red_dc, float ir_ac, float ir_dc)
{
    if (ir_dc < 1.0f || red_dc < 1.0f) return 0.0f;
    float R = (fabsf(red_ac) / red_dc) / (fabsf(ir_ac) / ir_dc);
    float spo2 = 110.0f - 25.0f * R;
    if (spo2 > 100.0f) spo2 = 100.0f;
    if (spo2 <  80.0f) return 0.0f;
    return spo2;
}

// ─── AGC ──────────────────────────────────────────────────────

esp_err_t max30102_agc(max30102_dev_t *dev, uint8_t *pa_out)
{
    ESP_LOGI(TAG, "AGC: finding optimal LED current...");
    uint8_t pa = 0x1F;
    int settled = 0;

    for (int attempt = 0; attempt < 60; attempt++) {
        i2c_manager_write_reg(dev->i2c_addr, MAX30102_REG_LED1_PA, pa);
        i2c_manager_write_reg(dev->i2c_addr, MAX30102_REG_LED2_PA, pa);
        i2c_manager_write_reg(dev->i2c_addr, MAX30102_REG_FIFO_WR_PTR, 0x00);
        i2c_manager_write_reg(dev->i2c_addr, MAX30102_REG_OVF_COUNTER,  0x00);
        i2c_manager_write_reg(dev->i2c_addr, MAX30102_REG_FIFO_RD_PTR,  0x00);
        vTaskDelay(pdMS_TO_TICKS(60));

        uint8_t wr = 0, rd = 0;
        i2c_manager_read_regs(dev->i2c_addr, MAX30102_REG_FIFO_WR_PTR, &wr, 1);
        i2c_manager_read_regs(dev->i2c_addr, MAX30102_REG_FIFO_RD_PTR, &rd, 1);
        wr &= 0x1F; rd &= 0x1F;

        if (((wr - rd + MAX30102_FIFO_DEPTH) % MAX30102_FIFO_DEPTH) == 0) continue;

        uint8_t raw[6] = {0};
        i2c_manager_read_regs(dev->i2c_addr, MAX30102_REG_FIFO_DATA, raw, 6);
        uint32_t ir = ((uint32_t)(raw[3] & 0x03) << 16) |
                      ((uint32_t) raw[4]          <<  8) |
                       (uint32_t) raw[5];

        if (ir >= MAX30102_AGC_TARGET_LOW && ir <= MAX30102_AGC_TARGET_HIGH) {
            settled++;
            if (settled >= MAX30102_AGC_SETTLED_READS) {
                ESP_LOGI(TAG, "AGC done: PA=0x%02X (%.1fmA)", pa, pa * 0.2f);
                *pa_out = pa;
                return ESP_OK;
            }
        } else {
            settled = 0;
            if (ir < MAX30102_AGC_TARGET_LOW) {
                uint8_t step = (pa < 0x10) ? 0x08 : (pa < 0x40) ? 0x04 : 0x02;
                pa = (pa + step > 0xFF) ? 0xFF : pa + step;
            } else {
                uint8_t step = (pa > 0x40) ? 0x08 : (pa > 0x10) ? 0x04 : 0x02;
                pa = (pa < step + 1) ? 0x01 : pa - step;
            }
        }
    }
    ESP_LOGW(TAG, "AGC: did not converge, using PA=0x%02X", pa);
    *pa_out = pa;
    return ESP_OK;
}

// ─── Public API ───────────────────────────────────────────────

esp_err_t max30102_init(max30102_dev_t *dev)
{
    memset(dev, 0, sizeof(*dev));
    dev->i2c_addr        = MAX30102_I2C_ADDR;
    dev->dc_red.alpha    = MAX30102_DC_FILTER_ALPHA;
    dev->dc_ir.alpha     = MAX30102_DC_FILTER_ALPHA;
    dev->ir_threshold    = MAX30102_IR_FINGER_THRESHOLD;
    dev->pp_ir.max_val   =  1.0f;
    dev->pp_ir.min_val   = -1.0f;
    dev->pp_ir.reset_every = 25;

    uint8_t part_id = 0;
    esp_err_t ret = i2c_manager_read_regs(dev->i2c_addr, MAX30102_REG_PART_ID, &part_id, 1);
    if (ret != ESP_OK || part_id != MAX30102_PART_ID) {
        ESP_LOGE(TAG, "MAX30102 not found (Part ID: 0x%02X)", part_id);
        return ESP_ERR_NOT_FOUND;
    }

    i2c_manager_write_reg(dev->i2c_addr, MAX30102_REG_MODE_CONFIG, MAX30102_MODE_RESET);
    uint8_t mode_val = MAX30102_MODE_RESET;
    for (int i = 0; i < 20 && (mode_val & MAX30102_MODE_RESET); i++) {
        vTaskDelay(pdMS_TO_TICKS(10));
        i2c_manager_read_regs(dev->i2c_addr, MAX30102_REG_MODE_CONFIG, &mode_val, 1);
    }

    i2c_manager_write_reg(dev->i2c_addr, MAX30102_REG_FIFO_CONFIG,  0x5F);
    i2c_manager_write_reg(dev->i2c_addr, MAX30102_REG_SPO2_CONFIG,  0x27);
    i2c_manager_write_reg(dev->i2c_addr, MAX30102_REG_LED1_PA,      0x1F);
    i2c_manager_write_reg(dev->i2c_addr, MAX30102_REG_LED2_PA,      0x1F);
    i2c_manager_write_reg(dev->i2c_addr, MAX30102_REG_MODE_CONFIG,  MAX30102_MODE_SPO2);
    i2c_manager_write_reg(dev->i2c_addr, MAX30102_REG_FIFO_WR_PTR,  0x00);
    i2c_manager_write_reg(dev->i2c_addr, MAX30102_REG_OVF_COUNTER,  0x00);
    i2c_manager_write_reg(dev->i2c_addr, MAX30102_REG_FIFO_RD_PTR,  0x00);

    vTaskDelay(pdMS_TO_TICKS(200));
    ESP_LOGI(TAG, "MAX30102 initialized successfully");
    return ESP_OK;
}

bool max30102_finger_present(max30102_dev_t *dev, uint32_t ir_raw)
{
    return (ir_raw >= dev->ir_threshold);
}

void max30102_sensor_reset(max30102_dev_t *dev)
{
    dev->bpm_smooth      = 0.0f;
    dev->last_beat_us    = 0;
    dev->above_threshold = false;
    dev->settled         = false;
    dev->settle_count    = 0;
    dev->pp_ir.max_val   =  1.0f;
    dev->pp_ir.min_val   = -1.0f;
    dev->pp_ir.age       = 0;
    dev->dc_red.w        = 0.0f;
    dev->dc_ir.w         = 0.0f;
    memset(&dev->ma_red, 0, sizeof(dev->ma_red));
    memset(&dev->ma_ir,  0, sizeof(dev->ma_ir));
    i2c_manager_write_reg(dev->i2c_addr, MAX30102_REG_LED1_PA, 0x1F);
    i2c_manager_write_reg(dev->i2c_addr, MAX30102_REG_LED2_PA, 0x1F);
}

esp_err_t max30102_read_fifo(max30102_dev_t *dev, max30102_data_t *out)
{
    memset(out, 0, sizeof(*out));
    uint8_t wr_ptr = 0, rd_ptr = 0;
    i2c_manager_read_regs(dev->i2c_addr, MAX30102_REG_FIFO_WR_PTR, &wr_ptr, 1);
    i2c_manager_read_regs(dev->i2c_addr, MAX30102_REG_FIFO_RD_PTR, &rd_ptr, 1);
    wr_ptr &= 0x1F; rd_ptr &= 0x1F;

    int num_samples = ((int)wr_ptr - (int)rd_ptr + MAX30102_FIFO_DEPTH) % MAX30102_FIFO_DEPTH;
    if (num_samples == 0) return ESP_ERR_NOT_FOUND;
    if (num_samples > MAX30102_FIFO_DEPTH) num_samples = MAX30102_FIFO_DEPTH;

    bool any_beat = false;
    for (int s = 0; s < num_samples; s++) {
        uint8_t raw[6] = {0};
        esp_err_t ret = i2c_manager_read_regs(dev->i2c_addr, MAX30102_REG_FIFO_DATA, raw, 6);
        if (ret != ESP_OK) return ret;

        uint32_t red = ((uint32_t)(raw[0] & 0x03) << 16) | ((uint32_t)raw[1] << 8) | raw[2];
        uint32_t ir  = ((uint32_t)(raw[3] & 0x03) << 16) | ((uint32_t)raw[4] << 8) | raw[5];
        out->red_raw = red;
        out->ir_raw  = ir;

        float red_ac = dc_filter(&dev->dc_red, (float)red);
        float ir_ac  = dc_filter(&dev->dc_ir,  (float)ir);
        out->red_ac  = red_ac;
        out->ir_ac   = ir_ac;

        float red_f = ma_filter(&dev->ma_red, red_ac);
        float ir_f  = ma_filter(&dev->ma_ir,  ir_ac);
        out->red_filtered = red_f;
        out->ir_filtered  = ir_f;

        if (!dev->settled) {
            dev->settle_count++;
            pp_update(&dev->pp_ir, ir_f);
            if (dev->settle_count >= MAX30102_SETTLE_SAMPLES) {
                dev->settled             = true;
                dev->pp_ir.max_val       = 0.0f;
                dev->pp_ir.min_val       = 0.0f;
                dev->pp_ir.age           = 0;
                dev->bpm_smooth          = 0.0f;
                dev->last_beat_us        = 0;
                dev->above_threshold     = false;
                ESP_LOGI(TAG, "DC settled — beat detection active");
            }
        } else {
            if (detect_beat(dev, ir_f)) any_beat = true;
        }

        out->bpm          = dev->bpm_smooth;
        out->spo2         = compute_spo2(red_ac, (float)red, ir_ac, (float)ir);
        out->peak_to_peak = dev->pp_ir.max_val - dev->pp_ir.min_val;
    }
    out->beat_detected = any_beat;
    return ESP_OK;
}

esp_err_t max30102_read_temperature(max30102_dev_t *dev, float *temp_c)
{
    i2c_manager_write_reg(dev->i2c_addr, MAX30102_REG_TEMP_CONFIG, 0x01);
    vTaskDelay(pdMS_TO_TICKS(35));
    uint8_t t_int = 0, t_frac = 0;
    i2c_manager_read_regs(dev->i2c_addr, MAX30102_REG_TEMP_INT,  &t_int,  1);
    i2c_manager_read_regs(dev->i2c_addr, MAX30102_REG_TEMP_FRAC, &t_frac, 1);
    *temp_c = (float)(int8_t)t_int + ((float)(t_frac & 0x0F) * 0.0625f);
    return ESP_OK;
}
