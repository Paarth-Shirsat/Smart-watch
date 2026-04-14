#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "driver/i2c.h"
#include "esp_err.h"

// ─── Default config ───────────────────────────────────────────
#define MAX_M10S_I2C_ADDR_DEFAULT   0x42        // u-blox default DDC address
#define MAX_M10S_I2C_FREQ_HZ        400000      // 400 kHz fast-mode

// ─── UBX protocol constants ───────────────────────────────────
#define UBX_SYNC1                   0xB5
#define UBX_SYNC2                   0x62
#define UBX_CLASS_NAV               0x01
#define UBX_ID_NAV_PVT              0x07        // Position/Velocity/Time
#define UBX_NAV_PVT_LEN            92           // payload bytes

// ─── Fix type values (gnssFixOK field) ───────────────────────
typedef enum {
    GPS_FIX_NONE   = 0,
    GPS_FIX_DR     = 1,   // dead-reckoning only
    GPS_FIX_2D     = 2,
    GPS_FIX_3D     = 3,
    GPS_FIX_GNSSDR = 4,   // GNSS + dead-reckoning
    GPS_FIX_TIME   = 5,   // time-only fix
} gps_fix_type_t;

// ─── Config struct ────────────────────────────────────────────
typedef struct {
    i2c_port_t  i2c_port;
    int         sda_pin;
    int         scl_pin;
    uint32_t    i2c_freq_hz;
    uint8_t     i2c_addr;
} max_m10s_config_t;

// ─── Parsed NAV-PVT output ────────────────────────────────────
typedef struct {
    // Fix quality
    uint8_t  fix_type;          // gps_fix_type_t
    uint8_t  satellites;        // number of SVs used

    // Position
    double   latitude;          // degrees
    double   longitude;         // degrees
    float    altitude_m;        // metres above ellipsoid

    // Velocity / heading
    float    speed_mps;         // ground speed m/s
    float    heading_deg;       // vehicle heading degrees

    // Accuracy estimates
    float    h_acc_m;           // horizontal accuracy m
    float    v_acc_m;           // vertical accuracy m

    // UTC time
    bool     date_valid;
    bool     time_valid;
    uint16_t year;
    uint8_t  month;
    uint8_t  day;
    uint8_t  hour;
    uint8_t  minute;
    uint8_t  second;
} max_m10s_data_t;

// ─── Internal device handle ───────────────────────────────────
typedef struct {
    i2c_port_t  i2c_port;
    uint8_t     i2c_addr;
} max_m10s_dev_t;

// ─── Public API ───────────────────────────────────────────────
esp_err_t max_m10s_init(const max_m10s_config_t *cfg);
esp_err_t max_m10s_read_nav_pvt(max_m10s_data_t *out);