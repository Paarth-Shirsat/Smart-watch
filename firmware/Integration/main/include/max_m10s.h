#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

// ─── Default I2C address ──────────────────────────────────────
#define MAX_M10S_I2C_ADDR_DEFAULT   0x42

// ─── UBX protocol constants ───────────────────────────────────
#define UBX_SYNC1           0xB5
#define UBX_SYNC2           0x62
#define UBX_CLASS_NAV       0x01
#define UBX_ID_NAV_PVT      0x07
#define UBX_NAV_PVT_LEN     92      // payload bytes

// ─── Fix type enum ────────────────────────────────────────────
typedef enum {
    GPS_FIX_NONE   = 0,
    GPS_FIX_DR     = 1,
    GPS_FIX_2D     = 2,
    GPS_FIX_3D     = 3,
    GPS_FIX_GNSSDR = 4,
    GPS_FIX_TIME   = 5,
} gps_fix_type_t;

// ─── Parsed NAV-PVT output ────────────────────────────────────
typedef struct {
    uint8_t  fix_type;
    uint8_t  satellites;
    double   latitude;
    double   longitude;
    float    altitude_m;
    float    speed_mps;
    float    heading_deg;
    float    h_acc_m;
    float    v_acc_m;
    bool     date_valid;
    bool     time_valid;
    uint16_t year;
    uint8_t  month, day, hour, minute, second;
} max_m10s_data_t;

// ─── Internal device handle ───────────────────────────────────
typedef struct {
    uint8_t i2c_addr;
} max_m10s_dev_t;

// ─── Public API ───────────────────────────────────────────────
esp_err_t max_m10s_init(max_m10s_dev_t *dev, uint8_t addr);
esp_err_t max_m10s_read_nav_pvt(max_m10s_dev_t *dev, max_m10s_data_t *out);
