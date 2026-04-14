#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_err.h"
#include "math.h"
#include "lsm6dso.h"

static const char *TAG = "LSM6DSO";

// ═══════════════════════════════════════════════════════════════
//  I2C LOW-LEVEL HELPERS
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
    if (len > 1) {
        i2c_master_read(cmd, buf, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, buf + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd,
                        pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    return ret;
}

// ═══════════════════════════════════════════════════════════════
//  I2C MASTER INIT
// ═══════════════════════════════════════════════════════════════

esp_err_t lsm6dso_i2c_master_init(void)
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
//  LSM6DSO INIT
// ═══════════════════════════════════════════════════════════════

esp_err_t lsm6dso_init(lsm6dso_dev_t *dev, uint8_t addr)
{
    dev->i2c_addr   = addr;
    dev->accel_sens = LSM6DSO_ACCEL_SENS_2G;
    dev->gyro_sens  = LSM6DSO_GYRO_SENS_250DPS;

    // Zero calibration biases
    memset(&dev->calib, 0, sizeof(dev->calib));

    // 1. Verify WHO_AM_I
    uint8_t who = 0;
    esp_err_t ret = i2c_read_regs(addr, LSM6DSO_REG_WHO_AM_I, &who, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C read failed (addr=0x%02X): %s", addr, esp_err_to_name(ret));
        return ret;
    }
    if (who != LSM6DSO_WHO_AM_I_VAL) {
        ESP_LOGE(TAG, "Wrong WHO_AM_I: 0x%02X (expected 0x%02X)", who, LSM6DSO_WHO_AM_I_VAL);
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "LSM6DSO found, WHO_AM_I=0x%02X", who);

    // 2. Software reset & reboot memory
    ret = i2c_write_reg(addr, LSM6DSO_REG_CTRL3_C, 0x01);  // SW_RESET bit
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(20));

    // 3. CTRL3_C: Block data update (BDU) ON, auto-increment ON
    //    BDU ensures high/low bytes belong to the same sample
    ret = i2c_write_reg(addr, LSM6DSO_REG_CTRL3_C, 0x44);
    if (ret != ESP_OK) return ret;

    // 4. CTRL1_XL: Accel — 104 Hz (0x40), ±2g (00), LPF1 default
    //    Bits[7:4]=ODR 0100=104Hz, Bits[3:2]=FS 00=±2g
    ret = i2c_write_reg(addr, LSM6DSO_REG_CTRL1_XL, 0x40);
    if (ret != ESP_OK) return ret;

    // 5. CTRL2_G: Gyro — 104 Hz (0x40), ±250 dps (00)
    //    Bits[7:4]=ODR 0100=104Hz, Bits[3:2]=FS 00=±250dps
    ret = i2c_write_reg(addr, LSM6DSO_REG_CTRL2_G, 0x40);
    if (ret != ESP_OK) return ret;

    // 6. CTRL6_C: Disable accel high-performance mode (bit4=0 = HP ON by default)
    //    Leave at 0x00 (HP mode ON — better noise performance)
    ret = i2c_write_reg(addr, LSM6DSO_REG_CTRL6_C, 0x00);
    if (ret != ESP_OK) return ret;

    // 7. CTRL7_G: Disable gyro high-performance mode suppression (HP ON)
    ret = i2c_write_reg(addr, LSM6DSO_REG_CTRL7_G, 0x00);
    if (ret != ESP_OK) return ret;

    // Wait for first samples at 104 Hz (~10ms period, give 3 cycles)
    vTaskDelay(pdMS_TO_TICKS(50));

    ESP_LOGI(TAG, "LSM6DSO configured: Accel ±2g @ 104Hz | Gyro ±250dps @ 104Hz");
    return ESP_OK;
}

// ═══════════════════════════════════════════════════════════════
//  READ RAW 16-BIT REGISTERS (accel + gyro + temp burst)
// ═══════════════════════════════════════════════════════════════

static esp_err_t lsm6dso_read_raw_lsb(lsm6dso_dev_t *dev,
                                        int16_t *ax, int16_t *ay, int16_t *az,
                                        int16_t *gx, int16_t *gy, int16_t *gz,
                                        int16_t *temp)
{
    // Wait for both accel and gyro data ready (STATUS bit0=XLDA, bit1=GDA)
    uint8_t status = 0;
    for (int i = 0; i < 20; i++) {
        esp_err_t r = i2c_read_regs(dev->i2c_addr, LSM6DSO_REG_STATUS, &status, 1);
        if (r != ESP_OK) return r;
        if ((status & 0x03) == 0x03) break;    // both XLDA + GDA set
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    // Burst read 14 bytes: TEMP(2) + GYRO(6) + ACCEL(6)
    // Starting from OUT_TEMP_L (0x20) through OUTZ_H_A (0x2D)
    uint8_t raw[14];
    esp_err_t ret = i2c_read_regs(dev->i2c_addr, LSM6DSO_REG_OUT_TEMP_L, raw, 14);
    if (ret != ESP_OK) return ret;

    *temp = (int16_t)(raw[1]  << 8 | raw[0]);
    *gx   = (int16_t)(raw[3]  << 8 | raw[2]);
    *gy   = (int16_t)(raw[5]  << 8 | raw[4]);
    *gz   = (int16_t)(raw[7]  << 8 | raw[6]);
    *ax   = (int16_t)(raw[9]  << 8 | raw[8]);
    *ay   = (int16_t)(raw[11] << 8 | raw[10]);
    *az   = (int16_t)(raw[13] << 8 | raw[12]);

    return ESP_OK;
}

// ═══════════════════════════════════════════════════════════════
//  CALIBRATION
//  Sensor must be FLAT and STILL during calibration.
//  Accel: X=0g, Y=0g, Z=+1g expected → bias = measured - expected
//  Gyro:  X=0, Y=0, Z=0 expected → bias = measured
// ═══════════════════════════════════════════════════════════════

esp_err_t lsm6dso_calibrate(lsm6dso_dev_t *dev)
{
    ESP_LOGI(TAG, "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    ESP_LOGI(TAG, "  CALIBRATION STARTED");
    ESP_LOGI(TAG, "  Place sensor FLAT & STILL on a level surface.");
    ESP_LOGI(TAG, "  Collecting %d samples...", LSM6DSO_CALIB_SAMPLES);
    ESP_LOGI(TAG, "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");

    vTaskDelay(pdMS_TO_TICKS(2000));    // 2s grace period for user to settle sensor

    double sum_ax = 0, sum_ay = 0, sum_az = 0;
    double sum_gx = 0, sum_gy = 0, sum_gz = 0;

    int16_t ax, ay, az, gx, gy, gz, temp;
    int valid = 0;

    for (int i = 0; i < LSM6DSO_CALIB_SAMPLES; i++) {
        esp_err_t ret = lsm6dso_read_raw_lsb(dev, &ax, &ay, &az,
                                               &gx, &gy, &gz, &temp);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Skipping sample %d (read error)", i);
            continue;
        }
        sum_ax += ax;
        sum_ay += ay;
        sum_az += az;
        sum_gx += gx;
        sum_gy += gy;
        sum_gz += gz;
        valid++;

        // Progress every 64 samples
        if ((i + 1) % 64 == 0) {
            ESP_LOGI(TAG, "  Progress: %d/%d", i + 1, LSM6DSO_CALIB_SAMPLES);
        }

        vTaskDelay(pdMS_TO_TICKS(5));   // ~104Hz, so 5ms is within ODR
    }

    if (valid == 0) {
        ESP_LOGE(TAG, "Calibration failed — no valid samples");
        return ESP_FAIL;
    }

    // Average raw LSB values
    double mean_ax = sum_ax / valid;
    double mean_ay = sum_ay / valid;
    double mean_az = sum_az / valid;
    double mean_gx = sum_gx / valid;
    double mean_gy = sum_gy / valid;
    double mean_gz = sum_gz / valid;

    // Convert to physical units (g, dps) then subtract expected gravity on Z
    // Expected: AX=0g, AY=0g, AZ=+1g (sensor flat, Z pointing up)
    float s = dev->accel_sens;
    dev->calib.accel_bias.x = (float)(mean_ax * s);           // measured - 0g
    dev->calib.accel_bias.y = (float)(mean_ay * s);           // measured - 0g
    dev->calib.accel_bias.z = (float)(mean_az * s) - 1.0f;   // measured - 1g

    float gs = dev->gyro_sens;
    dev->calib.gyro_bias.x  = (float)(mean_gx * gs);
    dev->calib.gyro_bias.y  = (float)(mean_gy * gs);
    dev->calib.gyro_bias.z  = (float)(mean_gz * gs);

    ESP_LOGI(TAG, "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    ESP_LOGI(TAG, "  CALIBRATION COMPLETE (%d samples)", valid);
    ESP_LOGI(TAG, "  Accel bias → X: %+.4f g  Y: %+.4f g  Z: %+.4f g",
             dev->calib.accel_bias.x,
             dev->calib.accel_bias.y,
             dev->calib.accel_bias.z);
    ESP_LOGI(TAG, "  Gyro  bias → X: %+.4f°/s Y: %+.4f°/s Z: %+.4f°/s",
             dev->calib.gyro_bias.x,
             dev->calib.gyro_bias.y,
             dev->calib.gyro_bias.z);
    ESP_LOGI(TAG, "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");

    return ESP_OK;
}

// ═══════════════════════════════════════════════════════════════
//  PUBLIC READ — returns calibrated physical values
// ═══════════════════════════════════════════════════════════════

esp_err_t lsm6dso_read_raw(lsm6dso_dev_t *dev,
                            lsm6dso_vec3_t *accel_g,
                            lsm6dso_vec3_t *gyro_dps,
                            float *temp_c)
{
    int16_t ax, ay, az, gx, gy, gz, temp;
    esp_err_t ret = lsm6dso_read_raw_lsb(dev, &ax, &ay, &az,
                                          &gx, &gy, &gz, &temp);
    if (ret != ESP_OK) return ret;

    // Convert to physical units and subtract calibration bias
    float s  = dev->accel_sens;
    float gs = dev->gyro_sens;

    accel_g->x   = (ax * s)  - dev->calib.accel_bias.x;
    accel_g->y   = (ay * s)  - dev->calib.accel_bias.y;
    accel_g->z   = (az * s)  - dev->calib.accel_bias.z;

    gyro_dps->x  = (gx * gs) - dev->calib.gyro_bias.x;
    gyro_dps->y  = (gy * gs) - dev->calib.gyro_bias.y;
    gyro_dps->z  = (gz * gs) - dev->calib.gyro_bias.z;

    // Temperature: 0 LSB = 25°C, sensitivity = 1/256 °C/LSB
    *temp_c = 25.0f + (float)temp / 256.0f;

    return ESP_OK;
}

// ═══════════════════════════════════════════════════════════════
//  TILT ANGLES from accelerometer (static tilt only)
//  Roll  = atan2(Y, Z)
//  Pitch = atan2(-X, sqrt(Y²+Z²))
// ═══════════════════════════════════════════════════════════════

static void compute_tilt(lsm6dso_vec3_t *a, float *roll, float *pitch)
{
    *roll  = atan2f(a->y, a->z) * (180.0f / (float)M_PI);
    *pitch = atan2f(-a->x, sqrtf(a->y * a->y + a->z * a->z))
             * (180.0f / (float)M_PI);
}

// ═══════════════════════════════════════════════════════════════
//  APP MAIN
// ═══════════════════════════════════════════════════════════════

void app_main(void)
{
    ESP_LOGI(TAG, "Initializing I2C: SDA=GPIO%d, SCL=GPIO%d",
             I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);

    ESP_ERROR_CHECK(lsm6dso_i2c_master_init());

    lsm6dso_dev_t imu;

    // Try primary address first, fall back to secondary
    esp_err_t ret = lsm6dso_init(&imu, LSM6DSO_ADDR_LOW);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "0x6A failed, trying 0x6B...");
        ESP_ERROR_CHECK(lsm6dso_init(&imu, LSM6DSO_ADDR_HIGH));
    }

    // ── Calibration ─────────────────────────────────────────
    ESP_ERROR_CHECK(lsm6dso_calibrate(&imu));

    // ── Main Loop ───────────────────────────────────────────
    lsm6dso_vec3_t accel, gyro;
    float temp_c, roll, pitch;

    while (1) {
        ret = lsm6dso_read_raw(&imu, &accel, &gyro, &temp_c);

        if (ret == ESP_OK) {
            compute_tilt(&accel, &roll, &pitch);

            ESP_LOGI(TAG, "── Accel (g)   X:%+7.4f  Y:%+7.4f  Z:%+7.4f",
                     accel.x, accel.y, accel.z);
            ESP_LOGI(TAG, "── Gyro  (°/s) X:%+8.3f  Y:%+8.3f  Z:%+8.3f",
                     gyro.x, gyro.y, gyro.z);
            ESP_LOGI(TAG, "── Tilt        Roll:%+7.2f°  Pitch:%+7.2f°",
                     roll, pitch);
            ESP_LOGI(TAG, "── Temp        %.2f °C", temp_c);
            ESP_LOGI(TAG, "─────────────────────────────────────────");

        } else {
            ESP_LOGE(TAG, "Read failed: %s", esp_err_to_name(ret));
        }

        vTaskDelay(pdMS_TO_TICKS(100));   // 10 Hz display rate
    }
}