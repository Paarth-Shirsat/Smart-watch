#include "max_m10s.h"

#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "MAX-M10S";

/* ── Driver state ────────────────────────────────────────────────────────── */

static i2c_port_t s_port;
static uint8_t    s_addr;
static bool       s_init = false;

/* ── Timing ──────────────────────────────────────────────────────────────── */

#define I2C_TIMEOUT_MS      100
#define POLL_RETRIES        50
#define POLL_DELAY_MS       20

/* ── UBX Fletcher-8 checksum ─────────────────────────────────────────────── */

static void ubx_checksum(const uint8_t *data, size_t len,
                          uint8_t *ck_a, uint8_t *ck_b)
{
    uint8_t a = 0, b = 0;
    for (size_t i = 0; i < len; i++) {
        a += data[i];
        b += a;
    }
    *ck_a = a;
    *ck_b = b;
}

/* ── Build a UBX frame into buf; returns total byte count ────────────────── */

static size_t ubx_build_frame(uint8_t cls, uint8_t id,
                               const uint8_t *payload, uint16_t plen,
                               uint8_t *out)
{
    out[0] = UBX_SYNC1;
    out[1] = UBX_SYNC2;
    out[2] = cls;
    out[3] = id;
    out[4] = (uint8_t)(plen & 0xFF);
    out[5] = (uint8_t)(plen >> 8);
    if (payload && plen) memcpy(&out[6], payload, plen);
    ubx_checksum(&out[2], 4 + plen, &out[6 + plen], &out[6 + plen + 1]);
    return 6 + plen + 2;
}

/* ── I2C: write register address then read ───────────────────────────────── */

static esp_err_t i2c_read_reg(uint8_t reg, uint8_t *buf, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (!cmd) return ESP_ERR_NO_MEM;

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (s_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (s_addr << 1) | I2C_MASTER_READ, true);
    if (len > 1) i2c_master_read(cmd, buf, len - 1, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, &buf[len - 1], I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(s_port, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    return ret;
}

/* ── I2C: write raw bytes (no register prefix — used for UBX frames) ─────── */

static esp_err_t i2c_write_raw(const uint8_t *data, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (!cmd) return ESP_ERR_NO_MEM;

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (s_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, (uint8_t *)data, len, true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(s_port, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    return ret;
}

/* ── UBX-CFG-VALSET: enable UBX output + auto NAV-PVT on I2C ─────────────
 *
 *  Key IDs (little-endian u32):
 *    CFG-I2COUTPROT-UBX       0x10720001  L  (1 byte) — enable UBX on I2C out
 *    CFG-I2COUTPROT-NMEA      0x10720002  L  (1 byte) — disable NMEA on I2C out
 *    CFG-MSGOUT-UBX_NAV_PVT_I2C 0x20910007 U1 (1 byte) — output NAV-PVT every epoch
 */
static const uint8_t k_valset_payload[] = {
    0x00,                           /* version                                  */
    0x01,                           /* layers = RAM                             */
    0x00, 0x00,                     /* reserved                                 */
    0x01, 0x00, 0x72, 0x10, 0x01,  /* CFG-I2COUTPROT-UBX  = 1                 */
    0x02, 0x00, 0x72, 0x10, 0x00,  /* CFG-I2COUTPROT-NMEA = 0                 */
    0x07, 0x00, 0x91, 0x20, 0x01,  /* CFG-MSGOUT-UBX_NAV_PVT_I2C = 1          */
};

static esp_err_t configure_ubx_output(void)
{
    uint8_t frame[6 + sizeof(k_valset_payload) + 2];
    size_t  flen = ubx_build_frame(0x06, 0x8A,
                                    k_valset_payload, sizeof(k_valset_payload),
                                    frame);
    esp_err_t ret = i2c_write_raw(frame, flen);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "CFG-VALSET write failed: %s", esp_err_to_name(ret));
        return ret;
    }
    /* Give the module time to apply the config and ACK */
    vTaskDelay(pdMS_TO_TICKS(200));
    ESP_LOGI(TAG, "UBX output configured on I2C");
    return ESP_OK;
}

/* ── Public API ──────────────────────────────────────────────────────────── */

esp_err_t max_m10s_init(const max_m10s_config_t *cfg)
{
    if (!cfg) return ESP_ERR_INVALID_ARG;
    if (s_init) return ESP_OK;

    s_port = cfg->i2c_port;
    s_addr = cfg->i2c_addr ? cfg->i2c_addr : MAX_M10S_I2C_ADDR_DEFAULT;

    i2c_config_t ic = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = cfg->sda_pin,
        .scl_io_num       = cfg->scl_pin,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = cfg->i2c_freq_hz ? cfg->i2c_freq_hz
                                              : MAX_M10S_I2C_FREQ_HZ,
    };

    esp_err_t ret = i2c_param_config(s_port, &ic);
    if (ret != ESP_OK) return ret;

    ret = i2c_driver_install(s_port, I2C_MODE_MASTER, 0, 0, 0);
    if (ret != ESP_OK) return ret;

    /* Verify device is reachable */
    uint16_t avail = 0;
    ret = max_m10s_bytes_available(&avail);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Device not found at 0x%02X", s_addr);
        i2c_driver_delete(s_port);
        return ESP_ERR_NOT_FOUND;
    }

    /* Switch I2C output to UBX and enable auto NAV-PVT */
    ret = configure_ubx_output();
    if (ret != ESP_OK) {
        i2c_driver_delete(s_port);
        return ret;
    }

    s_init = true;
    ESP_LOGI(TAG, "Ready (port=%d addr=0x%02X)", s_port, s_addr);
    return ESP_OK;
}

esp_err_t max_m10s_deinit(void)
{
    if (!s_init) return ESP_OK;
    s_init = false;
    return i2c_driver_delete(s_port);
}

esp_err_t max_m10s_bytes_available(uint16_t *count)
{
    if (!count) return ESP_ERR_INVALID_ARG;
    uint8_t buf[2] = {0};
    esp_err_t ret = i2c_read_reg(MAX_M10S_REG_BYTES_AVAIL_H, buf, 2);
    if (ret != ESP_OK) return ret;
    uint16_t n = ((uint16_t)buf[0] << 8) | buf[1];
    *count = (n == 0xFFFF) ? 0 : n;   /* 0xFFFF = buffer empty / not ready    */
    return ESP_OK;
}

esp_err_t max_m10s_read_stream(uint8_t *buf, size_t len)
{
    if (!buf || !len) return ESP_ERR_INVALID_ARG;
    return i2c_read_reg(MAX_M10S_REG_DATA_STREAM, buf, len);
}

esp_err_t max_m10s_send_ubx(const uint8_t *buf, size_t len)
{
    if (!buf || !len) return ESP_ERR_INVALID_ARG;
    return i2c_write_raw(buf, len);
}

/* ── NAV-PVT ─────────────────────────────────────────────────────────────── */

static const uint8_t *find_nav_pvt(const uint8_t *stream, size_t slen)
{
    const size_t frame_size = 6 + UBX_NAV_PVT_PAYLOAD_LEN + 2;

    for (size_t i = 0; i + frame_size <= slen; i++) {
        if (stream[i]     != UBX_SYNC1)      continue;
        if (stream[i + 1] != UBX_SYNC2)      continue;
        if (stream[i + 2] != UBX_CLASS_NAV)  continue;
        if (stream[i + 3] != UBX_ID_NAV_PVT) continue;

        uint16_t plen = (uint16_t)stream[i + 4] | ((uint16_t)stream[i + 5] << 8);
        if (plen != UBX_NAV_PVT_PAYLOAD_LEN) continue;

        uint8_t ck_a, ck_b;
        ubx_checksum(&stream[i + 2], 4 + plen, &ck_a, &ck_b);
        if (stream[i + 6 + plen] != ck_a)     continue;
        if (stream[i + 6 + plen + 1] != ck_b) continue;

        return &stream[i];
    }
    return NULL;
}

esp_err_t max_m10s_read_nav_pvt(max_m10s_data_t *data)
{
    if (!data) return ESP_ERR_INVALID_ARG;

    /*
     * NAV-PVT is auto-output every navigation epoch (~1 Hz by default).
     * Wait until the module has at least one full frame ready.
     */
    const size_t expected = 6 + UBX_NAV_PVT_PAYLOAD_LEN + 2;   /* 100 bytes */
    uint16_t avail = 0;

    for (int i = 0; i < POLL_RETRIES; i++) {
        vTaskDelay(pdMS_TO_TICKS(POLL_DELAY_MS));
        max_m10s_bytes_available(&avail);
        if (avail >= expected) break;
    }
    if (avail < expected) {
        ESP_LOGW(TAG, "Timeout — bytes available: %u (need %u)",
                 avail, (unsigned)expected);
        return ESP_ERR_TIMEOUT;
    }

    /* Read stream */
    uint8_t *buf = malloc(avail);
    if (!buf) return ESP_ERR_NO_MEM;

    esp_err_t ret = max_m10s_read_stream(buf, avail);
    if (ret != ESP_OK) { free(buf); return ret; }

    /* Locate and validate the frame */
    const uint8_t *pvt_frame = find_nav_pvt(buf, avail);
    if (!pvt_frame) {
        ESP_LOGE(TAG, "NAV-PVT not found or checksum mismatch");
        free(buf);
        return ESP_ERR_INVALID_RESPONSE;
    }

    /* Parse */
    ubx_nav_pvt_payload_t pvt;
    memcpy(&pvt, pvt_frame + 6, sizeof(pvt));
    free(buf);

    data->fix_type    = pvt.fixType;
    data->satellites  = pvt.numSV;
    data->latitude    = pvt.lat  * 1e-7;
    data->longitude   = pvt.lon  * 1e-7;
    data->altitude_m  = pvt.hMSL * 1e-3f;
    data->speed_mps   = pvt.gSpeed  * 1e-3f;
    data->heading_deg = pvt.headMot * 1e-5f;
    data->h_acc_m     = pvt.hAcc * 1e-3f;
    data->v_acc_m     = pvt.vAcc * 1e-3f;
    data->year        = pvt.year;
    data->month       = pvt.month;
    data->day         = pvt.day;
    data->hour        = pvt.hour;
    data->minute      = pvt.min;
    data->second      = pvt.sec;
    data->date_valid  = (pvt.valid & 0x01) != 0;
    data->time_valid  = (pvt.valid & 0x02) != 0;

    return ESP_OK;
}
