#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include <math.h>

static const char *TAG = "ROUND_LCD";

/* ── Pin definitions ──────────────────────────────────────────────────────── */
#define LCD_SPI_HOST    SPI2_HOST
#define LCD_PIN_SCLK    18
#define LCD_PIN_MOSI    23
#define LCD_PIN_CS       5
#define LCD_PIN_DC       2
#define LCD_PIN_RST      4
#define LCD_PIN_BL      15

#define TOUCH_I2C_PORT  I2C_NUM_0
#define TOUCH_PIN_SDA   21
#define TOUCH_PIN_SCL   22
#define TOUCH_PIN_INT   36   /* set to -1 to disable */
#define TOUCH_PIN_RST   27   /* set to -1 to share with LCD RST */
#define TOUCH_I2C_ADDR  0x15 /* CST816S default address */
#define TOUCH_I2C_HZ    400000

/* ── LCD geometry ─────────────────────────────────────────────────────────── */
#define LCD_WIDTH   240
#define LCD_HEIGHT  240

/* ── SPI speed ────────────────────────────────────────────────────────────── */
#define LCD_SPI_HZ  (40 * 1000 * 1000)  /* 40 MHz */

/* ── Colour helpers (RGB565) ──────────────────────────────────────────────── */
#define RGB565(r, g, b) \
    ((uint16_t)(((r) & 0xF8) << 8) | (((g) & 0xFC) << 3) | ((b) >> 3))

#define COL_BLACK   0x0000
#define COL_WHITE   0xFFFF
#define COL_RED     RGB565(255,   0,   0)
#define COL_GREEN   RGB565(  0, 255,   0)
#define COL_BLUE    RGB565(  0,   0, 255)
#define COL_YELLOW  RGB565(255, 255,   0)
#define COL_CYAN    RGB565(  0, 255, 255)

//Display Driver

static spi_device_handle_t s_spi;

/* Send a single command byte */
static void lcd_cmd(uint8_t cmd)
{
    gpio_set_level(LCD_PIN_DC, 0);
    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &cmd,
        .flags = 0,
    };
    spi_device_polling_transmit(s_spi, &t);
}

/* Send a single data byte */
static void lcd_data(uint8_t data)
{
    gpio_set_level(LCD_PIN_DC, 1);
    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &data,
        .flags = 0,
    };
    spi_device_polling_transmit(s_spi, &t);
}

/* Send a buffer (data mode) */
static void lcd_data_buf(const uint8_t *buf, size_t len)
{
    if (!len) return;
    gpio_set_level(LCD_PIN_DC, 1);
    spi_transaction_t t = {
        .length = len * 8,
        .tx_buffer = buf,
        .flags = 0,
    };
    spi_device_polling_transmit(s_spi, &t);
}

/* Hardware reset */
static void lcd_reset(void)
{
    gpio_set_level(LCD_PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(LCD_PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(120));
}

/* GC9A01 full initialisation sequence */
static void gc9a01_init(void)
{
    lcd_reset();

    lcd_cmd(0xEF);
    lcd_cmd(0xEB); lcd_data(0x14);
    lcd_cmd(0xFE);
    lcd_cmd(0xEF);

    lcd_cmd(0xEB); lcd_data(0x14);
    lcd_cmd(0x84); lcd_data(0x40);
    lcd_cmd(0x85); lcd_data(0xFF);
    lcd_cmd(0x86); lcd_data(0xFF);
    lcd_cmd(0x87); lcd_data(0xFF);
    lcd_cmd(0x88); lcd_data(0x0A);
    lcd_cmd(0x89); lcd_data(0x21);
    lcd_cmd(0x8A); lcd_data(0x00);
    lcd_cmd(0x8B); lcd_data(0x80);
    lcd_cmd(0x8C); lcd_data(0x01);
    lcd_cmd(0x8D); lcd_data(0x01);
    lcd_cmd(0x8E); lcd_data(0xFF);
    lcd_cmd(0x8F); lcd_data(0xFF);

    lcd_cmd(0xB6);
    lcd_data(0x00);
    lcd_data(0x20);          /* display direction */

    lcd_cmd(0x3A); lcd_data(0x05); /* 16-bit (RGB565) */

    lcd_cmd(0x90);
    lcd_data(0x08); lcd_data(0x08);
    lcd_data(0x08); lcd_data(0x08);

    lcd_cmd(0xBD); lcd_data(0x06);
    lcd_cmd(0xBC); lcd_data(0x00);

    lcd_cmd(0xFF);
    lcd_data(0x60); lcd_data(0x01); lcd_data(0x04);

    lcd_cmd(0xC3); lcd_data(0x13);
    lcd_cmd(0xC4); lcd_data(0x13);
    lcd_cmd(0xC9); lcd_data(0x22);
    lcd_cmd(0xBE); lcd_data(0x11);

    lcd_cmd(0xE1);
    lcd_data(0x10); lcd_data(0x0E);

    lcd_cmd(0xDF);
    lcd_data(0x21); lcd_data(0x0C); lcd_data(0x02);

    /* Gamma */
    lcd_cmd(0xF0);
    lcd_data(0x45); lcd_data(0x09); lcd_data(0x08);
    lcd_data(0x08); lcd_data(0x26); lcd_data(0x2A);

    lcd_cmd(0xF1);
    lcd_data(0x43); lcd_data(0x70); lcd_data(0x72);
    lcd_data(0x36); lcd_data(0x37); lcd_data(0x6F);

    lcd_cmd(0xF2);
    lcd_data(0x45); lcd_data(0x09); lcd_data(0x08);
    lcd_data(0x08); lcd_data(0x26); lcd_data(0x2A);

    lcd_cmd(0xF3);
    lcd_data(0x43); lcd_data(0x70); lcd_data(0x72);
    lcd_data(0x36); lcd_data(0x37); lcd_data(0x6F);

    lcd_cmd(0xED);
    lcd_data(0x1B); lcd_data(0x0B);

    lcd_cmd(0xAE); lcd_data(0x77);
    lcd_cmd(0xCD); lcd_data(0x63);

    lcd_cmd(0x70);
    lcd_data(0x07); lcd_data(0x07); lcd_data(0x04);
    lcd_data(0x0E); lcd_data(0x0F); lcd_data(0x09);
    lcd_data(0x07); lcd_data(0x08); lcd_data(0x03);

    lcd_cmd(0xE8); lcd_data(0x34);

    lcd_cmd(0x62);
    lcd_data(0x18); lcd_data(0x0D); lcd_data(0x71);
    lcd_data(0xED); lcd_data(0x70); lcd_data(0x70);
    lcd_data(0x18); lcd_data(0x0F); lcd_data(0x71);
    lcd_data(0xEF); lcd_data(0x70); lcd_data(0x70);

    lcd_cmd(0x63);
    lcd_data(0x18); lcd_data(0x11); lcd_data(0x71);
    lcd_data(0xF1); lcd_data(0x70); lcd_data(0x70);
    lcd_data(0x18); lcd_data(0x13); lcd_data(0x71);
    lcd_data(0xF3); lcd_data(0x70); lcd_data(0x70);

    lcd_cmd(0x64);
    lcd_data(0x28); lcd_data(0x29); lcd_data(0xF1);
    lcd_data(0x01); lcd_data(0xF1); lcd_data(0x00);
    lcd_data(0x07);

    lcd_cmd(0x66);
    lcd_data(0x3C); lcd_data(0x00); lcd_data(0xCD);
    lcd_data(0x67); lcd_data(0x45); lcd_data(0x45);
    lcd_data(0x10); lcd_data(0x00); lcd_data(0x00);
    lcd_data(0x00);

    lcd_cmd(0x67);
    lcd_data(0x00); lcd_data(0x3C); lcd_data(0x00);
    lcd_data(0x00); lcd_data(0x00); lcd_data(0x01);
    lcd_data(0x54); lcd_data(0x10); lcd_data(0x32);
    lcd_data(0x98);

    lcd_cmd(0x74);
    lcd_data(0x10); lcd_data(0x85); lcd_data(0x80);
    lcd_data(0x00); lcd_data(0x00); lcd_data(0x4E);
    lcd_data(0x00);

    lcd_cmd(0x98);
    lcd_data(0x3E); lcd_data(0x07);

    lcd_cmd(0x35); /* Tearing effect ON */
    lcd_cmd(0x21); /* Inversion ON (needed for correct colours) */

    lcd_cmd(0x11); /* Sleep OUT */
    vTaskDelay(pdMS_TO_TICKS(120));
    lcd_cmd(0x29); /* Display ON */
    vTaskDelay(pdMS_TO_TICKS(20));
}

/* Set address window for subsequent pixel writes */
static void lcd_set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    lcd_cmd(0x2A);  /* Column address */
    lcd_data(x0 >> 8); lcd_data(x0 & 0xFF);
    lcd_data(x1 >> 8); lcd_data(x1 & 0xFF);

    lcd_cmd(0x2B);  /* Row address */
    lcd_data(y0 >> 8); lcd_data(y0 & 0xFF);
    lcd_data(y1 >> 8); lcd_data(y1 & 0xFF);

    lcd_cmd(0x2C);  /* Memory write */
}

/* Fill a rectangle with a solid colour */
static void lcd_fill_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t colour)
{
    if (w == 0 || h == 0) return;
    lcd_set_window(x, y, x + w - 1, y + h - 1);

    /* Build a small stripe buffer in DMA-capable RAM */
    const size_t stripe = 128;
    uint8_t *buf = heap_caps_malloc(stripe * 2, MALLOC_CAP_DMA);
    assert(buf);

    /* Pre-fill buffer with colour (big-endian) */
    for (size_t i = 0; i < stripe; i++) {
        buf[i * 2]     = colour >> 8;
        buf[i * 2 + 1] = colour & 0xFF;
    }

    size_t pixels = (size_t)w * h;
    while (pixels) {
        size_t chunk = pixels > stripe ? stripe : pixels;
        lcd_data_buf(buf, chunk * 2);
        pixels -= chunk;
    }
    free(buf);
}

/* Fill the whole screen */
static inline void lcd_clear(uint16_t colour)
{
    lcd_fill_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, colour);
}

/* Draw a single pixel */
static void lcd_draw_pixel(uint16_t x, uint16_t y, uint16_t colour)
{
    lcd_set_window(x, y, x, y);
    uint8_t buf[2] = { colour >> 8, colour & 0xFF };
    lcd_data_buf(buf, 2);
}

/* Draw a filled circle using midpoint algorithm */
static void lcd_fill_circle(int cx, int cy, int r, uint16_t colour)
{
    int x = 0, y = r, d = 1 - r;
    while (x <= y) {
        /* Draw horizontal spans for all octants */
        lcd_fill_rect(cx - y, cy + x, 2 * y + 1, 1, colour);
        lcd_fill_rect(cx - y, cy - x, 2 * y + 1, 1, colour);
        lcd_fill_rect(cx - x, cy + y, 2 * x + 1, 1, colour);
        lcd_fill_rect(cx - x, cy - y, 2 * x + 1, 1, colour);
        if (d < 0) d += 2 * x + 3;
        else { d += 2 * (x - y) + 5; y--; }
        x++;
    }
}

/* Draw a simple 5×7 digit character */
static const uint8_t FONT5x7[10][7] = {
    {0x3E,0x51,0x49,0x45,0x3E,0,0}, /* 0 */
    {0x00,0x42,0x7F,0x40,0x00,0,0}, /* 1 */
    {0x42,0x61,0x51,0x49,0x46,0,0}, /* 2 */
    {0x21,0x41,0x45,0x4B,0x31,0,0}, /* 3 */
    {0x18,0x14,0x12,0x7F,0x10,0,0}, /* 4 */
    {0x27,0x45,0x45,0x45,0x39,0,0}, /* 5 */
    {0x3C,0x4A,0x49,0x49,0x30,0,0}, /* 6 */
    {0x01,0x71,0x09,0x05,0x03,0,0}, /* 7 */
    {0x36,0x49,0x49,0x49,0x36,0,0}, /* 8 */
    {0x06,0x49,0x49,0x29,0x1E,0,0}, /* 9 */
};

static void lcd_draw_char(int x, int y, char c, uint16_t fg, uint16_t bg, int scale)
{
    if (c < '0' || c > '9') return;
    const uint8_t *glyph = FONT5x7[c - '0'];
    for (int col = 0; col < 5; col++) {
        uint8_t line = glyph[col];
        for (int row = 0; row < 7; row++) {
            uint16_t col_px = (line & (1 << row)) ? fg : bg;
            if (scale <= 1)
                lcd_draw_pixel(x + col, y + row, col_px);
            else
                lcd_fill_rect(x + col * scale, y + row * scale,
                              scale, scale, col_px);
        }
    }
}

static void lcd_draw_string(int x, int y, const char *s,
                             uint16_t fg, uint16_t bg, int scale)
{
    while (*s) {
        lcd_draw_char(x, y, *s++, fg, bg, scale);
        x += (5 + 1) * scale;
    }
}

// Touch Driver

typedef struct {
    uint16_t x;
    uint16_t y;
    uint8_t  gesture; /* 0=none,1=swipe-up,2=swipe-down,3=left,4=right,5=click */
    bool     pressed;
} touch_point_t;

/* Gesture IDs reported by CST816S */
#define GEST_NONE        0x00
#define GEST_SWIPE_UP    0x01
#define GEST_SWIPE_DOWN  0x02
#define GEST_SWIPE_LEFT  0x03
#define GEST_SWIPE_RIGHT 0x04
#define GEST_SINGLE_CLICK 0x05
#define GEST_DOUBLE_CLICK 0x0B
#define GEST_LONG_PRESS  0x0C

static void touch_reset(void)
{
#if TOUCH_PIN_RST >= 0
    gpio_set_level(TOUCH_PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(TOUCH_PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(50));
#endif
}

static esp_err_t touch_read_regs(uint8_t reg, uint8_t *buf, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (TOUCH_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (TOUCH_I2C_ADDR << 1) | I2C_MASTER_READ, true);
    if (len > 1)
        i2c_master_read(cmd, buf, len - 1, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, buf + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(TOUCH_I2C_PORT, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);
    return ret;
}

static bool touch_read(touch_point_t *tp)
{
    uint8_t buf[6];
    /* Registers: 0x01=gesture, 0x02=finger-count, 0x03-0x06=XH,XL,YH,YL */
    if (touch_read_regs(0x01, buf, 6) != ESP_OK) {
        tp->pressed = false;
        return false;
    }
    tp->gesture = buf[0];
    uint8_t fingers = buf[1] & 0x0F;
    tp->pressed = (fingers > 0);
    if (tp->pressed) {
        tp->x = ((buf[2] & 0x0F) << 8) | buf[3];
        tp->y = ((buf[4] & 0x0F) << 8) | buf[5];
    }
    return tp->pressed;
}

/* Chip ID readback – useful for verifying I2C communication */
static uint8_t touch_chip_id(void)
{
    uint8_t id = 0;
    touch_read_regs(0xA7, &id, 1);
    return id;
}

//Initialisation

static void hw_init(void)
{
    /* ── GPIO ── */
    gpio_config_t out_cfg = {
        .pin_bit_mask = (1ULL << LCD_PIN_DC) |
                        (1ULL << LCD_PIN_RST) |
                        (1ULL << LCD_PIN_BL),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&out_cfg);

#if TOUCH_PIN_RST >= 0
    gpio_config_t trst_cfg = {
        .pin_bit_mask = (1ULL << TOUCH_PIN_RST),
        .mode         = GPIO_MODE_OUTPUT,
    };
    gpio_config(&trst_cfg);
#endif

#if TOUCH_PIN_INT >= 0
    gpio_config_t int_cfg = {
        .pin_bit_mask = (1ULL << TOUCH_PIN_INT),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
    };
    gpio_config(&int_cfg);
#endif

    gpio_set_level(LCD_PIN_BL, 1);  /* backlight ON */

    /* ── SPI (LCD) ── */
    spi_bus_config_t buscfg = {
        .mosi_io_num   = LCD_PIN_MOSI,
        .miso_io_num   = -1,
        .sclk_io_num   = LCD_PIN_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_WIDTH * 16 * 2, /* one stripe */
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = LCD_SPI_HZ,
        .mode           = 0,
        .spics_io_num   = LCD_PIN_CS,
        .queue_size     = 7,
        .pre_cb         = NULL,
        .flags          = SPI_DEVICE_NO_DUMMY | SPI_DEVICE_HALFDUPLEX,
    };
    ESP_ERROR_CHECK(spi_bus_add_device(LCD_SPI_HOST, &devcfg, &s_spi));

    /* ── I2C (Touch) ── */
    i2c_config_t i2c_cfg = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = TOUCH_PIN_SDA,
        .scl_io_num       = TOUCH_PIN_SCL,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = TOUCH_I2C_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(TOUCH_I2C_PORT, &i2c_cfg));
    ESP_ERROR_CHECK(i2c_driver_install(TOUCH_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0));

    /* ── IC init ── */
    gc9a01_init();
    touch_reset();

    uint8_t chip_id = touch_chip_id();
    ESP_LOGI(TAG, "CST816S chip ID: 0x%02X (expect 0xB5)", chip_id);
}

//Task

/* Draw a simple watch-face style demo */
static void draw_face(void)
{
    lcd_clear(COL_BLACK);

    /* Outer ring */
    for (int r = 119; r >= 115; r--)
        for (int a = 0; a < 360; a++) {
            float rad = a * 3.14159f / 180.0f;
            int x = 120 + (int)(r * cosf(rad));
            int y = 120 + (int)(r * sinf(rad));
            lcd_draw_pixel(x, y, COL_CYAN);
        }

    /* Tick marks at every 30° */
    for (int i = 0; i < 12; i++) {
        float rad = i * 30 * 3.14159f / 180.0f;
        for (int r = 105; r <= 112; r++) {
            int x = 120 + (int)(r * cosf(rad));
            int y = 120 + (int)(r * sinf(rad));
            lcd_draw_pixel(x, y, COL_WHITE);
        }
    }

    /* Centre dot */
    lcd_fill_circle(120, 120, 4, COL_WHITE);

    /* Label */
    lcd_draw_string(79, 108, "240x240", COL_YELLOW, COL_BLACK, 2);
}

/* Draw a touch indicator at (x,y) */
static void draw_touch_indicator(int x, int y)
{
    lcd_fill_circle(x, y, 12, COL_RED);
    lcd_fill_circle(x, y,  4, COL_WHITE);
}

/* Map gesture enum to a short string */
static const char *gesture_str(uint8_t g)
{
    switch (g) {
        case GEST_SWIPE_UP:    return "UP   ";
        case GEST_SWIPE_DOWN:  return "DOWN ";
        case GEST_SWIPE_LEFT:  return "LEFT ";
        case GEST_SWIPE_RIGHT: return "RIGHT";
        case GEST_SINGLE_CLICK:return "CLICK";
        case GEST_DOUBLE_CLICK:return "DBLCK";
        case GEST_LONG_PRESS:  return "LONG ";
        default:               return "     ";
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Waveshare 1.28\" Round LCD + Touch demo starting…");

    hw_init();
    draw_face();

    static touch_point_t tp;
    static bool was_pressed = false;

    for (;;) {
        bool pressed = touch_read(&tp);

        if (pressed) {
            ESP_LOGI(TAG, "Touch x=%3d y=%3d gesture=%s",
                     tp.x, tp.y, gesture_str(tp.gesture));
            draw_touch_indicator(tp.x, tp.y);
            was_pressed = true;
        } else if (was_pressed) {
            /* Finger lifted – redraw face to erase indicator */
            draw_face();
            was_pressed = false;
        }

        vTaskDelay(pdMS_TO_TICKS(20)); /* ~50 Hz poll */
    }
}