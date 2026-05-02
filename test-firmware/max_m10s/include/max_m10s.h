#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ── I2C defaults ─────────────────────────────────────────────────────────── */

#define MAX_M10S_I2C_ADDR_DEFAULT   0x42
#define MAX_M10S_I2C_FREQ_HZ        400000

/* ── MAX-M10S internal register map ──────────────────────────────────────── */

#define MAX_M10S_REG_BYTES_AVAIL_H  0xFD    /* Available bytes MSB             */
#define MAX_M10S_REG_BYTES_AVAIL_L  0xFE    /* Available bytes LSB             */
#define MAX_M10S_REG_DATA_STREAM    0xFF    /* UBX data stream                 */

/* ── UBX protocol ────────────────────────────────────────────────────────── */

#define UBX_SYNC1                   0xB5
#define UBX_SYNC2                   0x62
#define UBX_CLASS_NAV               0x01
#define UBX_ID_NAV_PVT              0x07
#define UBX_NAV_PVT_PAYLOAD_LEN     92

/* ── Fix type codes (NAV-PVT fixType field) ──────────────────────────────── */

#define GPS_FIX_NONE                0
#define GPS_FIX_DEAD_RECKONING      1
#define GPS_FIX_2D                  2
#define GPS_FIX_3D                  3
#define GPS_FIX_GNSS_DEAD_RECKONING 4
#define GPS_FIX_TIME_ONLY           5

/* ── Configuration ───────────────────────────────────────────────────────── */

typedef struct {
    i2c_port_t  i2c_port;
    int         sda_pin;
    int         scl_pin;
    uint32_t    i2c_freq_hz;
    uint8_t     i2c_addr;
} max_m10s_config_t;

/* ── Parsed GPS data ─────────────────────────────────────────────────────── */

typedef struct {
    uint16_t year;
    uint8_t  month;
    uint8_t  day;
    uint8_t  hour;
    uint8_t  minute;
    uint8_t  second;
    bool     date_valid;
    bool     time_valid;

    double   latitude;      /* degrees, +N                                      */
    double   longitude;     /* degrees, +E                                      */
    float    altitude_m;    /* height above MSL                                 */

    float    speed_mps;
    float    heading_deg;

    uint8_t  fix_type;      /* GPS_FIX_* constants                              */
    uint8_t  satellites;
    float    h_acc_m;
    float    v_acc_m;
} max_m10s_data_t;

/* ── NAV-PVT wire layout (packed, 92 bytes) ──────────────────────────────── */

typedef struct __attribute__((packed)) {
    uint32_t iTOW;
    uint16_t year;
    uint8_t  month;
    uint8_t  day;
    uint8_t  hour;
    uint8_t  min;
    uint8_t  sec;
    uint8_t  valid;         /* bit0=validDate, bit1=validTime                   */
    uint32_t tAcc;
    int32_t  nano;
    uint8_t  fixType;
    uint8_t  flags;
    uint8_t  flags2;
    uint8_t  numSV;
    int32_t  lon;           /* 1e-7 deg                                         */
    int32_t  lat;           /* 1e-7 deg                                         */
    int32_t  height;        /* mm above ellipsoid                               */
    int32_t  hMSL;          /* mm above MSL                                     */
    uint32_t hAcc;          /* mm                                               */
    uint32_t vAcc;          /* mm                                               */
    int32_t  velN;          /* mm/s                                             */
    int32_t  velE;
    int32_t  velD;
    int32_t  gSpeed;        /* mm/s ground speed                                */
    int32_t  headMot;       /* 1e-5 deg                                         */
    uint32_t sAcc;
    uint32_t headAcc;
    uint16_t pDOP;
    uint8_t  flags3;
    uint8_t  reserved0[5];
    int32_t  headVeh;
    int16_t  magDec;
    uint16_t magAcc;
} ubx_nav_pvt_payload_t;

/* ── API ─────────────────────────────────────────────────────────────────── */

esp_err_t max_m10s_init(const max_m10s_config_t *cfg);
esp_err_t max_m10s_deinit(void);
esp_err_t max_m10s_bytes_available(uint16_t *count);
esp_err_t max_m10s_read_stream(uint8_t *buf, size_t len);
esp_err_t max_m10s_send_ubx(const uint8_t *buf, size_t len);
esp_err_t max_m10s_read_nav_pvt(max_m10s_data_t *data);

#ifdef __cplusplus
}
#endif
