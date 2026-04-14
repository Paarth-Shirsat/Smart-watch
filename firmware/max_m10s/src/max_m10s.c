#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "max_m10s.h"

static const char *TAG = "MAX-M10S";

// ─── Internal state ───────────────────────────────────────────
static max_m10s_dev_t s_dev;

// ─────────────────────────────────────────────────────────────
//  LOW-LEVEL I2C HELPERS
//
//  The MAX-M10S exposes a u-blox DDC (I2C) interface.
//  Register 0xFF = number of bytes available in the output buffer.
//  Registers 0xFD/0xFE = MSB/LSB of the same count (16-bit form).
//  Register 0xFF (write) / streaming read = UBX byte stream.
//
//  To READ: query 0xFD for byte count, then do a raw I2C read of
//  that many bytes — the module streams UBX frames continuously.
//  To WRITE a UBX message: plain I2C write with no register prefix.
// ─────────────────────────────────────────────────────────────

#define I2C_TIMEOUT_MS  1000

static esp_err_t i2c_read_reg(uint8_t reg, uint8_t *buf, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (s_dev.i2c_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (s_dev.i2c_addr << 1) | I2C_MASTER_READ, true);
    if (len > 1)
        i2c_master_read(cmd, buf, len - 1, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, buf + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(s_dev.i2c_port, cmd,
                        pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t i2c_stream_read(uint8_t *buf, size_t len)
{
    // Bare read — no register address — streams from the DDC output buffer
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (s_dev.i2c_addr << 1) | I2C_MASTER_READ, true);
    if (len > 1)
        i2c_master_read(cmd, buf, len - 1, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, buf + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(s_dev.i2c_port, cmd,
                        pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t i2c_write_bytes(const uint8_t *buf, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (s_dev.i2c_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, buf, len, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(s_dev.i2c_port, cmd,
                        pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    return ret;
}

// ─────────────────────────────────────────────────────────────
//  UBX HELPERS
// ─────────────────────────────────────────────────────────────

// Fletcher-8 checksum over class+id+length+payload
static void ubx_checksum(const uint8_t *buf, size_t len,
                          uint8_t *ck_a, uint8_t *ck_b)
{
    *ck_a = 0;
    *ck_b = 0;
    for (size_t i = 0; i < len; i++) {
        *ck_a += buf[i];
        *ck_b += *ck_a;
    }
}

// Poll a UBX message class/id from the module
static esp_err_t ubx_poll(uint8_t cls, uint8_t id)
{
    uint8_t frame[8];
    frame[0] = UBX_SYNC1;
    frame[1] = UBX_SYNC2;
    frame[2] = cls;
    frame[3] = id;
    frame[4] = 0x00;    // length LSB
    frame[5] = 0x00;    // length MSB
    ubx_checksum(&frame[2], 4, &frame[6], &frame[7]);
    return i2c_write_bytes(frame, 8);
}

// How many bytes are waiting in the DDC output buffer?
static esp_err_t ubx_bytes_available(uint16_t *count)
{
    uint8_t buf[2] = {0};
    // Registers 0xFD (MSB) and 0xFE (LSB)
    esp_err_t ret = i2c_read_reg(0xFD, buf, 2);
    if (ret != ESP_OK) return ret;
    *count = ((uint16_t)buf[0] << 8) | buf[1];
    return ESP_OK;
}

// ─────────────────────────────────────────────────────────────
//  UBX NAV-PVT PARSER
//  Payload layout (92 bytes) — u-blox 9/10 interface description
// ─────────────────────────────────────────────────────────────

static void parse_nav_pvt(const uint8_t *p, max_m10s_data_t *out)
{
    memset(out, 0, sizeof(*out));

    // Bytes 0-3:  iTOW (ms) — ignored
    out->year   = (uint16_t)p[4] | ((uint16_t)p[5] << 8);
    out->month  = p[6];
    out->day    = p[7];
    out->hour   = p[8];
    out->minute = p[9];
    out->second = p[10];

    uint8_t valid = p[11];
    out->date_valid = (valid & 0x01) != 0;  // validDate
    out->time_valid = (valid & 0x02) != 0;  // validTime

    // Bytes 12-15: tAcc — ignored
    // Bytes 16-19: nano — ignored

    out->fix_type  = p[20];
    // flags byte 21: bit0 = gnssFixOK, bit5:4 = psmState — ignored here
    out->satellites = p[23];

    // lon/lat in 1e-7 degrees
    int32_t lon_raw = (int32_t)((uint32_t)p[24] | ((uint32_t)p[25] << 8) |
                                ((uint32_t)p[26] << 16) | ((uint32_t)p[27] << 24));
    int32_t lat_raw = (int32_t)((uint32_t)p[28] | ((uint32_t)p[29] << 8) |
                                ((uint32_t)p[30] << 16) | ((uint32_t)p[31] << 24));
    out->longitude = lon_raw * 1e-7;
    out->latitude  = lat_raw * 1e-7;

    // height above ellipsoid in mm → metres
    int32_t height_mm = (int32_t)((uint32_t)p[32] | ((uint32_t)p[33] << 8) |
                                  ((uint32_t)p[34] << 16) | ((uint32_t)p[35] << 24));
    out->altitude_m = height_mm * 0.001f;

    // hAcc / vAcc in mm
    uint32_t h_acc_mm = (uint32_t)p[40] | ((uint32_t)p[41] << 8) |
                        ((uint32_t)p[42] << 16) | ((uint32_t)p[43] << 24);
    uint32_t v_acc_mm = (uint32_t)p[44] | ((uint32_t)p[45] << 8) |
                        ((uint32_t)p[46] << 16) | ((uint32_t)p[47] << 24);
    out->h_acc_m = h_acc_mm * 0.001f;
    out->v_acc_m = v_acc_mm * 0.001f;

    // gSpeed (ground speed) in mm/s → m/s
    int32_t gspeed_mms = (int32_t)((uint32_t)p[60] | ((uint32_t)p[61] << 8) |
                                   ((uint32_t)p[62] << 16) | ((uint32_t)p[63] << 24));
    out->speed_mps = gspeed_mms * 0.001f;

    // headMot (vehicle heading) in 1e-5 degrees
    int32_t head_raw = (int32_t)((uint32_t)p[64] | ((uint32_t)p[65] << 8) |
                                 ((uint32_t)p[66] << 16) | ((uint32_t)p[67] << 24));
    out->heading_deg = head_raw * 1e-5f;
}

// ─────────────────────────────────────────────────────────────
//  PUBLIC API
// ─────────────────────────────────────────────────────────────

esp_err_t max_m10s_init(const max_m10s_config_t *cfg)
{
    if (!cfg) return ESP_ERR_INVALID_ARG;

    s_dev.i2c_port = cfg->i2c_port;
    s_dev.i2c_addr = cfg->i2c_addr;

    // Install I2C driver
    i2c_config_t conf = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = cfg->sda_pin,
        .scl_io_num       = cfg->scl_pin,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = cfg->i2c_freq_hz,
    };
    esp_err_t ret = i2c_param_config(cfg->i2c_port, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2c_param_config failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ret = i2c_driver_install(cfg->i2c_port, I2C_MODE_MASTER, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2c_driver_install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Give the module time to boot
    vTaskDelay(pdMS_TO_TICKS(500));

    // Verify we can reach the module by reading the byte-count register
    uint16_t avail = 0;
    ret = ubx_bytes_available(&avail);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MAX-M10S not responding on I2C addr 0x%02X", cfg->i2c_addr);
        return ret;
    }
    ESP_LOGI(TAG, "MAX-M10S found at 0x%02X (%u bytes ready)", cfg->i2c_addr, avail);

    // Disable NMEA on DDC so we only receive UBX frames — cleaner parsing
    // UBX-CFG-PRT: disable NMEA output on DDC port (port ID 0)
    // This is optional but reduces noise; send UBX-CFG-PRT with outProtoMask=0x01 (UBX only)
    static const uint8_t cfg_prt[] = {
        UBX_SYNC1, UBX_SYNC2,
        0x06, 0x00,             // CFG-PRT
        0x14, 0x00,             // length = 20
        0x00,                   // portID = DDC (I2C)
        0x00,                   // reserved
        0x00, 0x00,             // txReady
        0x84, 0x00, 0x00, 0x00, // mode (7-bit addr 0x42 << 1 = 0x84)
        0x00, 0x00, 0x00, 0x00, // reserved
        0x07, 0x00,             // inProtoMask  = UBX+NMEA+RTCM
        0x01, 0x00,             // outProtoMask = UBX only
        0x00, 0x00,             // flags
        0x00, 0x00,             // reserved
        0x00, 0x00              // checksum — computed below
    };
    uint8_t prt_frame[28];
    memcpy(prt_frame, cfg_prt, 26);
    ubx_checksum(&prt_frame[2], 24, &prt_frame[26], &prt_frame[27]);
    i2c_write_bytes(prt_frame, 28);   // best-effort, ignore error

    vTaskDelay(pdMS_TO_TICKS(200));

    // Set navigation rate to 1 Hz (default, explicit)
    // UBX-CFG-RATE: measRate=1000ms, navRate=1, timeRef=1(GPS)
    static const uint8_t cfg_rate_body[] = {0x06, 0x08, 0x06, 0x00,
                                             0xE8, 0x03,  // measRate 1000 ms
                                             0x01, 0x00,  // navRate  1
                                             0x01, 0x00}; // timeRef  GPS
    uint8_t rate_frame[14];
    rate_frame[0] = UBX_SYNC1;
    rate_frame[1] = UBX_SYNC2;
    memcpy(&rate_frame[2], cfg_rate_body, 10);
    ubx_checksum(&rate_frame[2], 10, &rate_frame[12], &rate_frame[13]);
    i2c_write_bytes(rate_frame, 14);  // best-effort

    vTaskDelay(pdMS_TO_TICKS(200));
    ESP_LOGI(TAG, "MAX-M10S ready");
    return ESP_OK;
}

esp_err_t max_m10s_read_nav_pvt(max_m10s_data_t *out)
{
    if (!out) return ESP_ERR_INVALID_ARG;

    // Poll NAV-PVT so the module queues a fresh response
    esp_err_t ret = ubx_poll(UBX_CLASS_NAV, UBX_ID_NAV_PVT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Poll NAV-PVT failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Wait for the module to generate the response (~100 ms worst-case at 1 Hz)
    uint16_t avail = 0;
    int waited = 0;
    while (waited < 300) {
        vTaskDelay(pdMS_TO_TICKS(10));
        waited += 10;
        ret = ubx_bytes_available(&avail);
        if (ret != ESP_OK) return ret;
        // NAV-PVT frame = 6 (header+len) + 92 (payload) + 2 (ck) = 100 bytes
        if (avail >= 100) break;
    }

    if (avail == 0) {
        ESP_LOGW(TAG, "No data from MAX-M10S (timeout)");
        return ESP_ERR_TIMEOUT;
    }

    // Read all available bytes (cap at 256 to be safe)
    if (avail > 256) avail = 256;
    uint8_t buf[256];
    ret = i2c_stream_read(buf, avail);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Stream read failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Scan for a valid NAV-PVT frame in the buffer
    for (int i = 0; i <= (int)avail - 100; i++) {
        if (buf[i]   != UBX_SYNC1)      continue;
        if (buf[i+1] != UBX_SYNC2)      continue;
        if (buf[i+2] != UBX_CLASS_NAV)  continue;
        if (buf[i+3] != UBX_ID_NAV_PVT) continue;

        uint16_t payload_len = (uint16_t)buf[i+4] | ((uint16_t)buf[i+5] << 8);
        if (payload_len != UBX_NAV_PVT_LEN) continue;

        // Verify checksum
        uint8_t ck_a, ck_b;
        ubx_checksum(&buf[i+2], 4 + payload_len, &ck_a, &ck_b);
        if (ck_a != buf[i + 6 + payload_len] ||
            ck_b != buf[i + 7 + payload_len]) {
            ESP_LOGW(TAG, "NAV-PVT checksum mismatch");
            continue;
        }

        parse_nav_pvt(&buf[i+6], out);
        return ESP_OK;
    }

    ESP_LOGW(TAG, "NAV-PVT frame not found in %u bytes", avail);
    return ESP_ERR_NOT_FOUND;
}