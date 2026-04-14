#include "lsm6dso.h"
#include "i2c_manager.h"
#include "esp_log.h"
#include "math.h"
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "LSM6DSO_DRV";

// ─── Internal Raw Read ────────────────────────────────────────
static esp_err_t lsm6dso_read_raw_lsb(lsm6dso_dev_t *dev,
                                        int16_t *ax, int16_t *ay, int16_t *az,
                                        int16_t *gx, int16_t *gy, int16_t *gz,
                                        int16_t *temp)
{
    uint8_t status = 0;
    for (int i = 0; i < 20; i++) {
        esp_err_t r = i2c_manager_read_regs(dev->i2c_addr, LSM6DSO_REG_STATUS, &status, 1);
        if (r != ESP_OK) return r;
        if ((status & 0x03) == 0x03) break;
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    uint8_t raw[14];
    esp_err_t ret = i2c_manager_read_regs(dev->i2c_addr, LSM6DSO_REG_OUT_TEMP_L, raw, 14);
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

// ─── Public API ───────────────────────────────────────────────
esp_err_t lsm6dso_init(lsm6dso_dev_t *dev, uint8_t addr)
{
    dev->i2c_addr   = addr;
    dev->accel_sens = LSM6DSO_ACCEL_SENS_2G;
    dev->gyro_sens  = LSM6DSO_GYRO_SENS_250DPS;
    memset(&dev->calib, 0, sizeof(dev->calib));

    uint8_t who = 0;
    esp_err_t ret = i2c_manager_read_regs(addr, LSM6DSO_REG_WHO_AM_I, &who, 1);
    if (ret != ESP_OK || who != LSM6DSO_WHO_AM_I_VAL) {
        ESP_LOGE(TAG, "LSM6DSO not found (WHO_AM_I: 0x%02X)", who);
        return ESP_ERR_NOT_FOUND;
    }

    i2c_manager_write_reg(addr, LSM6DSO_REG_CTRL3_C, 0x01);
    vTaskDelay(pdMS_TO_TICKS(20));

    i2c_manager_write_reg(addr, LSM6DSO_REG_CTRL3_C, 0x44);
    i2c_manager_write_reg(addr, LSM6DSO_REG_CTRL1_XL, 0x40);
    i2c_manager_write_reg(addr, LSM6DSO_REG_CTRL2_G, 0x40);
    i2c_manager_write_reg(addr, LSM6DSO_REG_CTRL6_C, 0x00);
    i2c_manager_write_reg(addr, LSM6DSO_REG_CTRL7_G, 0x00);

    vTaskDelay(pdMS_TO_TICKS(50));
    ESP_LOGI(TAG, "LSM6DSO initialized successfully");
    return ESP_OK;
}

esp_err_t lsm6dso_calibrate(lsm6dso_dev_t *dev)
{
    ESP_LOGI(TAG, "Starting calibration... Keep sensor flat and still.");
    vTaskDelay(pdMS_TO_TICKS(2000));

    double sum_ax = 0, sum_ay = 0, sum_az = 0;
    double sum_gx = 0, sum_gy = 0, sum_gz = 0;
    int16_t ax, ay, az, gx, gy, gz, temp;
    int valid = 0;

    for (int i = 0; i < LSM6DSO_CALIB_SAMPLES; i++) {
        if (lsm6dso_read_raw_lsb(dev, &ax, &ay, &az, &gx, &gy, &gz, &temp) == ESP_OK) {
            sum_ax += ax; sum_ay += ay; sum_az += az;
            sum_gx += gx; sum_gy += gy; sum_gz += gz;
            valid++;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    if (valid == 0) return ESP_FAIL;

    float s = dev->accel_sens;
    float gs = dev->gyro_sens;
    dev->calib.accel_bias.x = (float)((sum_ax / valid) * s);
    dev->calib.accel_bias.y = (float)((sum_ay / valid) * s);
    dev->calib.accel_bias.z = (float)((sum_az / valid) * s) - 1.0f;
    dev->calib.gyro_bias.x  = (float)((sum_gx / valid) * gs);
    dev->calib.gyro_bias.y  = (float)((sum_gy / valid) * gs);
    dev->calib.gyro_bias.z  = (float)((sum_gz / valid) * gs);

    ESP_LOGI(TAG, "Calibration complete.");
    return ESP_OK;
}

esp_err_t lsm6dso_read_data(lsm6dso_dev_t *dev, lsm6dso_vec3_t *accel_g, lsm6dso_vec3_t *gyro_dps, float *temp_c)
{
    int16_t ax, ay, az, gx, gy, gz, temp;
    esp_err_t ret = lsm6dso_read_raw_lsb(dev, &ax, &ay, &az, &gx, &gy, &gz, &temp);
    if (ret != ESP_OK) return ret;

    float s = dev->accel_sens;
    float gs = dev->gyro_sens;
    accel_g->x = (ax * s) - dev->calib.accel_bias.x;
    accel_g->y = (ay * s) - dev->calib.accel_bias.y;
    accel_g->z = (az * s) - dev->calib.accel_bias.z;
    gyro_dps->x = (gx * gs) - dev->calib.gyro_bias.x;
    gyro_dps->y = (gy * gs) - dev->calib.gyro_bias.y;
    gyro_dps->z = (gz * gs) - dev->calib.gyro_bias.z;
    *temp_c = 25.0f + (float)temp / 256.0f;

    return ESP_OK;
}

void lsm6dso_compute_tilt(lsm6dso_vec3_t *a, float *roll, float *pitch)
{
    *roll  = atan2f(a->y, a->z) * (180.0f / (float)M_PI);
    *pitch = atan2f(-a->x, sqrtf(a->y * a->y + a->z * a->z)) * (180.0f / (float)M_PI);
}
