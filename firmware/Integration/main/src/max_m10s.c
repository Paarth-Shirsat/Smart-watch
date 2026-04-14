#include "max_m10s.h"
#include "i2c_manager.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "MAX_M10S_DRV";

// ─── UBX helpers ──────────────────────────────────────────────

static void ubx_checksum(const uint8_t *buf, size_t len,
                          uint8_t *ck_a, uint8_t *ck_b)
{
    *ck_a = 0; *ck_b = 0;
    for (size_t i = 0; i < len; i++) {
        *ck_a += buf[i];
        *ck_b += *ck_a;
    }
}

// Send a poll request (zero-length payload) for a given UBX message
static esp_err_t ubx_poll(max_m10s_dev_t *dev, uint8_t cls, uint8_t id)
{
    uint8_t frame[8];
    frame[0] = UBX_SYNC1;
    frame[1] = UBX_SYNC2;
    frame[2] = cls;
    frame[3] = id;
    frame[4] = 0x00;
    frame[5] = 0x00;
    ubx_checksum(&frame[2], 4, &frame[6], &frame[7]);
    return i2c_manager_write_bytes(dev->i2c_addr, frame, 8);
}

// How many bytes are in the DDC output buffer (registers 0xFD / 0xFE)?
static esp_err_t ubx_bytes_available(max_m10s_dev_t *dev, uint16_t *count)
{
    uint8_t buf[2] = {0};
    esp_err_t ret = i2c_manager_read_regs(dev->i2c_addr, 0xFD, buf, 2);
    if (ret != ESP_OK) return ret;
    *count = ((uint16_t)buf[0] << 8) | buf[1];
    return ESP_OK;
}

// ─── NAV-PVT parser ───────────────────────────────────────────

static void parse_nav_pvt(const uint8_t *p, max_m10s_data_t *out)
{
    memset(out, 0, sizeof(*out));

    out->year   = (uint16_t)p[4] | ((uint16_t)p[5] << 8);
    out->month  = p[6];
    out->day    = p[7];
    out->hour   = p[8];
    out->minute = p[9];
    out->second = p[10];
    uint8_t valid   = p[11];
    out->date_valid = (valid & 0x01) != 0;
    out->time_valid = (valid & 0x02) != 0;

    out->fix_type   = p[20];
    out->satellites = p[23];

    int32_t lon_raw = (int32_t)((uint32_t)p[24] | ((uint32_t)p[25] << 8) |
                                ((uint32_t)p[26] << 16) | ((uint32_t)p[27] << 24));
    int32_t lat_raw = (int32_t)((uint32_t)p[28] | ((uint32_t)p[29] << 8) |
                                ((uint32_t)p[30] << 16) | ((uint32_t)p[31] << 24));
    out->longitude  = lon_raw * 1e-7;
    out->latitude   = lat_raw * 1e-7;

    int32_t height_mm = (int32_t)((uint32_t)p[32] | ((uint32_t)p[33] << 8) |
                                  ((uint32_t)p[34] << 16) | ((uint32_t)p[35] << 24));
    out->altitude_m = height_mm * 0.001f;

    uint32_t h_acc_mm = (uint32_t)p[40] | ((uint32_t)p[41] << 8) |
                        ((uint32_t)p[42] << 16) | ((uint32_t)p[43] << 24);
    uint32_t v_acc_mm = (uint32_t)p[44] | ((uint32_t)p[45] << 8) |
                        ((uint32_t)p[46] << 16) | ((uint32_t)p[47] << 24);
    out->h_acc_m = h_acc_mm * 0.001f;
    out->v_acc_m = v_acc_mm * 0.001f;

    int32_t gspeed_mms = (int32_t)((uint32_t)p[60] | ((uint32_t)p[61] << 8) |
                                   ((uint32_t)p[62] << 16) | ((uint32_t)p[63] << 24));
    out->speed_mps = gspeed_mms * 0.001f;

    int32_t head_raw = (int32_t)((uint32_t)p[64] | ((uint32_t)p[65] << 8) |
                                 ((uint32_t)p[66] << 16) | ((uint32_t)p[67] << 24));
    out->heading_deg = head_raw * 1e-5f;
}

// ─── Public API ───────────────────────────────────────────────

esp_err_t max_m10s_init(max_m10s_dev_t *dev, uint8_t addr)
{
    dev->i2c_addr = addr;

    // Give module time to boot on first power-up
    vTaskDelay(pdMS_TO_TICKS(500));

    // Verify reachability
    uint16_t avail = 0;
    esp_err_t ret = ubx_bytes_available(dev, &avail);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MAX-M10S not found at 0x%02X: %s", addr, esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "MAX-M10S found at 0x%02X (%u bytes ready)", addr, avail);

    // Configure UBX-only output on the DDC port (disable NMEA)
    // UBX-CFG-PRT for DDC: outProtoMask = 0x0001 (UBX only)
    uint8_t cfg_prt[26] = {
        UBX_SYNC1, UBX_SYNC2,
        0x06, 0x00,             // CFG-PRT
        0x14, 0x00,             // length = 20
        0x00,                   // portID = DDC
        0x00,                   // reserved
        0x00, 0x00,             // txReady
        0x84, 0x00, 0x00, 0x00, // mode (7-bit addr 0x42 << 1)
        0x00, 0x00, 0x00, 0x00, // reserved
        0x07, 0x00,             // inProtoMask = UBX+NMEA+RTCM
        0x01, 0x00,             // outProtoMask = UBX only
        0x00, 0x00,             // flags
        0x00, 0x00              // reserved
    };
    uint8_t prt_frame[28];
    memcpy(prt_frame, cfg_prt, 26);
    ubx_checksum(&prt_frame[2], 24, &prt_frame[26], &prt_frame[27]);
    i2c_manager_write_bytes(dev->i2c_addr, prt_frame, 28); // best-effort

    vTaskDelay(pdMS_TO_TICKS(200));
    ESP_LOGI(TAG, "MAX-M10S initialized (UBX output mode)");
    return ESP_OK;
}

esp_err_t max_m10s_read_nav_pvt(max_m10s_dev_t *dev, max_m10s_data_t *out)
{
    if (!out) return ESP_ERR_INVALID_ARG;

    // Poll a fresh NAV-PVT response
    esp_err_t ret = ubx_poll(dev, UBX_CLASS_NAV, UBX_ID_NAV_PVT);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "NAV-PVT poll failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Wait up to 400 ms for 100 bytes (6+92+2) to appear
    uint16_t avail = 0;
    for (int waited = 0; waited < 400; waited += 10) {
        vTaskDelay(pdMS_TO_TICKS(10));
        ret = ubx_bytes_available(dev, &avail);
        if (ret != ESP_OK) return ret;
        if (avail >= 100) break;
    }

    if (avail == 0) {
        ESP_LOGW(TAG, "GPS: no data (no fix yet?)");
        return ESP_ERR_TIMEOUT;
    }

    // Cap read at 256 bytes
    if (avail > 256) avail = 256;
    uint8_t buf[256];
    ret = i2c_manager_stream_read(dev->i2c_addr, buf, avail);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Stream read failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Scan for valid NAV-PVT frame
    for (int i = 0; i <= (int)avail - 100; i++) {
        if (buf[i]   != UBX_SYNC1)      continue;
        if (buf[i+1] != UBX_SYNC2)      continue;
        if (buf[i+2] != UBX_CLASS_NAV)  continue;
        if (buf[i+3] != UBX_ID_NAV_PVT) continue;

        uint16_t plen = (uint16_t)buf[i+4] | ((uint16_t)buf[i+5] << 8);
        if (plen != UBX_NAV_PVT_LEN) continue;

        uint8_t ck_a, ck_b;
        ubx_checksum(&buf[i+2], 4 + plen, &ck_a, &ck_b);
        if (ck_a != buf[i + 6 + plen] || ck_b != buf[i + 7 + plen]) {
            ESP_LOGW(TAG, "NAV-PVT checksum mismatch");
            continue;
        }

        parse_nav_pvt(&buf[i+6], out);
        return ESP_OK;
    }

    ESP_LOGW(TAG, "NAV-PVT frame not found in %u bytes (acquiring fix...)", avail);
    return ESP_ERR_NOT_FOUND;
}
