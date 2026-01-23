/*
 * ESP32-S3-4848S040 Board Support Package - Implementation
 *
 * SPDX-FileCopyrightText: 2026 Victor Arzolla
 *
 * SPDX-License-Identifier: MIT
 *
 * Hardware initialization and abstraction for LCD display, touch panel,
 * and LVGL graphics support on the ESP32-S3-4848S040 development board.
 */

/* =========================================================
 *                  INCLUDES
 * ========================================================= */
#include <driver/gpio.h>
#include <driver/i2c_master.h>
#include <esp_lcd_panel_io_additions.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_st7701.h>
#include <esp_lcd_touch_gt911.h>
#include <esp_log.h>
#include <esp_lvgl_port.h>

#include "bsp/esp-bsp.h"

/* =========================================================
 *                  DEFINES
 * ========================================================= */
#define TAG "BSP"

/* =========================================================
 *                  CONSTANTS
 * ========================================================= */

/**
 * @brief ST7701 initialization command sequence
 *
 * This sequence configures the ST7701 LCD controller for 480x480
 * RGB panel operation with proper voltage levels and gamma correction.
 */
static const st7701_lcd_init_cmd_t lcd_init_cmds[] = {
    {0xFF, (uint8_t []){0x77, 0x01, 0x00, 0x00, 0x13}, 5, 0},
    {0xEF, (uint8_t []){0x08}, 1, 0},

    {0xFF, (uint8_t []){0x77, 0x01, 0x00, 0x00, 0x10}, 5, 0},
    {0xC0, (uint8_t []){0x3B, 0x00}, 2, 0},
    {0xC1, (uint8_t []){0x0D, 0x02}, 2, 0},
    {0xC2, (uint8_t []){0x21, 0x08}, 2, 0},
    {0xCD, (uint8_t []){0x00}, 1, 0},
    {
        0xB0,
        (uint8_t []){
            0x00, 0x11, 0x18, 0x0E, 0x11, 0x06, 0x07, 0x08, 0x07, 0x22, 0x04, 0x12, 0x0F, 0xAA, 0x31, 0x18
        },
        16, 0
    },
    {
        0xB1,
        (uint8_t []){
            0x00, 0x11, 0x19, 0x0E, 0x12, 0x07, 0x08, 0x08, 0x08, 0x22, 0x04, 0x11, 0x11, 0xA9, 0x32, 0x18
        },
        16, 0
    },

    {0xFF, (uint8_t []){0x77, 0x01, 0x00, 0x00, 0x11}, 5, 0},
    {0xB0, (uint8_t []){0x60}, 1, 0},
    {0xB1, (uint8_t []){0x30}, 1, 0},
    {0xB2, (uint8_t []){0x87}, 1, 0},
    {0xB3, (uint8_t []){0x80}, 1, 0},
    {0xB5, (uint8_t []){0x49}, 1, 0},
    {0xB7, (uint8_t []){0x85}, 1, 0},
    {0xB8, (uint8_t []){0x21}, 1, 0},
    {0xC1, (uint8_t []){0x78}, 1, 0},
    {0xC2, (uint8_t []){0x78}, 1, 20},
    {0xE0, (uint8_t []){0x00, 0x1B, 0x02}, 3, 0},
    {0xE1, (uint8_t []){0x08, 0xA0, 0x00, 0x00, 0x07, 0xA0, 0x00, 0x00, 0x00, 0x44, 0x44}, 11, 0},
    {0xE2, (uint8_t []){0x11, 0x11, 0x44, 0x44, 0xED, 0xA0, 0x00, 0x00, 0xEC, 0xA0, 0x00, 0x00}, 12, 0},
    {0xE3, (uint8_t []){0x00, 0x00, 0x11, 0x11}, 4, 0},
    {0xE4, (uint8_t []){0x44, 0x44}, 2, 0},
    {
        0xE5,
        (uint8_t []){
            0x0A, 0xE9, 0xD8, 0xA0, 0x0C, 0xEB, 0xD8, 0xA0, 0x0E, 0xED, 0xD8, 0xA0, 0x10, 0xEF, 0xD8, 0xA0
        },
        16, 0
    },
    {0xE6, (uint8_t []){0x00, 0x00, 0x11, 0x11}, 4, 0},
    {0xE7, (uint8_t []){0x44, 0x44}, 2, 0},
    {
        0xE8,
        (uint8_t []){
            0x09, 0xE8, 0xD8, 0xA0, 0x0B, 0xEA, 0xD8, 0xA0, 0x0D, 0xEC, 0xD8, 0xA0, 0x0F, 0xEE, 0xD8, 0xA0
        },
        16, 0
    },
    {0xEB, (uint8_t []){0x02, 0x00, 0xE4, 0xE4, 0x88, 0x00, 0x40}, 7, 0},
    {0xEC, (uint8_t []){0x3C, 0x00}, 2, 0},
    {
        0xED,
        (uint8_t []){
            0xAB, 0x89, 0x76, 0x54, 0x02, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x20, 0x45, 0x67, 0x98, 0xBA
        },
        16, 0
    },

    {0xFF, (uint8_t []){0x77, 0x01, 0x00, 0x00, 0x00}, 5, 0},

    {0x11, NULL, 0, 120},
    {0x29, NULL, 0, 0},
};

/* =========================================================
 *                  STATIC VARIABLES
 * ========================================================= */

/**
 * LVGL display handle (module-private)
 * Initialized by bsp_init() and bsp_display_start()
 */
static lv_display_t *disp = NULL;

/**
 * LVGL touch input device handle (module-private)
 * Initialized by bsp_init() and bsp_display_start()
 */
static lv_indev_t *disp_indev = NULL;

/* =========================================================
 *                  LVGL LOCK/UNLOCK FUNCTIONS
 * ========================================================= */

bool bsp_display_lock(uint32_t timeout_ms)
{
    return lvgl_port_lock(timeout_ms);
}

void bsp_display_unlock(void)
{
    lvgl_port_unlock();
}

/* =========================================================
 *                  INITIALIZATION FUNCTIONS
 * ========================================================= */

esp_err_t bsp_init(void)
{
    ESP_LOGI(TAG, "Initializing BSP");

    esp_err_t ret = ESP_OK;

    /* ================================================
     *  Initialize LCD backlight GPIO
     * ================================================ */
    ESP_LOGI(TAG, "Initializing LCD backlight");
    const gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << BSP_LCD_BACKLIGHT_GPIO
    };
    ret = gpio_config(&bk_gpio_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure backlight GPIO: %s", esp_err_to_name(ret));
        return ret;
    }
    /* Set backlight to disabled initially */
    ret = gpio_set_level(BSP_LCD_BACKLIGHT_GPIO, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set backlight level: %s", esp_err_to_name(ret));
        return ret;
    }

    /* ================================================
     *  Initialize LCD panel
     * ================================================ */
    ESP_LOGI(TAG, "Initializing 3-wire SPI");
    /* Configure SPI line: CS, SCL (clock), SDA (data) */
    spi_line_config_t line_config = {
        .cs_io_type = IO_TYPE_GPIO,
        .cs_gpio_num = BSP_LCD_SPI_CS_GPIO,
        .scl_io_type = IO_TYPE_GPIO,
        .scl_gpio_num = BSP_LCD_SPI_SCK_GPIO,
        .sda_io_type = IO_TYPE_GPIO,
        .sda_gpio_num = BSP_LCD_SPI_MOSI_GPIO,
        .io_expander = NULL,
    };

    esp_lcd_panel_io_3wire_spi_config_t io_config = ST7701_PANEL_IO_3WIRE_SPI_CONFIG(line_config, 0);
    esp_lcd_panel_io_handle_t io_handle = NULL;
    ret = esp_lcd_new_panel_io_3wire_spi(&io_config, &io_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create panel IO: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Initializing ST7701 driver");
    esp_lcd_rgb_panel_config_t rgb_config = {
        .clk_src = LCD_CLK_SRC_PLL160M,
        .timings = ST7701_480_480_PANEL_60HZ_RGB_TIMING(),
        .data_width = 16,
        .bits_per_pixel = 16,
        .dma_burst_size = 64,
        .hsync_gpio_num = BSP_LCD_HSYNC_GPIO,
        .vsync_gpio_num = BSP_LCD_VSYNC_GPIO,
        .de_gpio_num = BSP_LCD_DE_GPIO,
        .pclk_gpio_num = BSP_LCD_PCLK_GPIO,
        .disp_gpio_num = BSP_LCD_DISP_GPIO,
        .data_gpio_nums = {
            BSP_LCD_R0_GPIO, BSP_LCD_R1_GPIO, BSP_LCD_R2_GPIO, BSP_LCD_R3_GPIO, BSP_LCD_R4_GPIO,
            BSP_LCD_G0_GPIO, BSP_LCD_G1_GPIO, BSP_LCD_G2_GPIO, BSP_LCD_G3_GPIO, BSP_LCD_G4_GPIO, BSP_LCD_G5_GPIO,
            BSP_LCD_B0_GPIO, BSP_LCD_B1_GPIO, BSP_LCD_B2_GPIO, BSP_LCD_B3_GPIO, BSP_LCD_B4_GPIO,
        },
        .flags = {
            .disp_active_low = 0,
            .refresh_on_demand = 0,
            .fb_in_psram = 1,
            .double_fb = 0,
            .no_fb = 0,
            .bb_invalidate_cache = 0
        },
        .num_fbs = 2,
        .bounce_buffer_size_px = BSP_LCD_H_RES * 10
    };

    st7701_vendor_config_t vendor_config = {
        .rgb_config = &rgb_config,
        .init_cmds = lcd_init_cmds,
        .init_cmds_size = sizeof(lcd_init_cmds) / sizeof(st7701_lcd_init_cmd_t),
        .flags = {
            .mirror_by_cmd = 1,
            .enable_io_multiplex = 0,
        },
    };

    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = BSP_LCD_RST_GPIO,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .bits_per_pixel = BSP_LCD_BITS_PER_PIXEL,
        .vendor_config = &vendor_config,
    };

    esp_lcd_panel_handle_t panel_handle = NULL;
    ret = esp_lcd_new_panel_st7701(io_handle, &panel_config, &panel_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create panel: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_lcd_panel_reset(panel_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to reset panel: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_lcd_panel_init(panel_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize panel: %s", esp_err_to_name(ret));
        return ret;
    }

    /* ================================================
     *  Initialize LVGL graphics library
     * ================================================ */
    ESP_LOGI(TAG, "Initializing LVGL");
    /* Initialize LVGL port with default configuration */
    const lvgl_port_cfg_t lvgl_port_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    ret = lvgl_port_init(&lvgl_port_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize LVGL port: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Configure LVGL display with RGB panel and PSRAM buffer */
    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = io_handle,
        .panel_handle = panel_handle,
        .double_buffer = true,
        .buffer_size = BSP_LVGL_BUFFER_SIZE,
        .hres = BSP_LCD_H_RES,
        .vres = BSP_LCD_V_RES,
        .monochrome = false,
        .color_format = LV_COLOR_FORMAT_RGB565,
        .rotation = {
            .swap_xy = false,
            .mirror_x = false,
            .mirror_y = false,
        },
        .flags = {
            .buff_dma = true,
            .swap_bytes = false,
            .buff_spiram = true,
            .full_refresh = true,
            .direct_mode = true
        }
    };

    const lvgl_port_display_rgb_cfg_t rgb_cfg = {
        .flags = {
            .bb_mode = true,
            .avoid_tearing = true,
        }
    };

    /* Add RGB display to LVGL */
    disp = lvgl_port_add_disp_rgb(&disp_cfg, &rgb_cfg);
    if (disp == NULL) {
        ESP_LOGE(TAG, "Failed to setup LVGL display");
        return ESP_FAIL;
    }


    /* ================================================
     *  Initialize touch panel
     * ================================================ */
    ESP_LOGI(TAG, "Initializing touch panel");
    /* Create I2C master bus for touch controller */
    i2c_master_bus_handle_t tp_bus_handle = NULL;
    /* Configure I2C: 400 kHz, internal pull-ups enabled */
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = -1,
        .scl_io_num = BSP_TOUCH_I2C_SCL_GPIO,
        .sda_io_num = BSP_TOUCH_I2C_SDA_GPIO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    ret = i2c_new_master_bus(&i2c_mst_config, &tp_bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C master bus: %s", esp_err_to_name(ret));
        return ret;
    }

    esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_GT911_CONFIG();
    tp_io_config.scl_speed_hz = BSP_TOUCH_I2C_CLK_HZ;

    esp_lcd_panel_io_handle_t tp_io_handle = NULL;
    ret = esp_lcd_new_panel_io_i2c(tp_bus_handle, &tp_io_config, &tp_io_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create touch IO: %s", esp_err_to_name(ret));
        return ret;
    }

    esp_lcd_touch_io_gt911_config_t tp_gt911_config = {
        .dev_addr = tp_io_config.dev_addr,
    };

    esp_lcd_touch_config_t tp_cfg = {
        .x_max = BSP_LCD_V_RES,
        .y_max = BSP_LCD_H_RES,
        .rst_gpio_num = BSP_TOUCH_RST_GPIO,
        .int_gpio_num = BSP_TOUCH_INT_GPIO,
        .levels = {
            .reset = 0,
            .interrupt = 0,
        },
        .flags = {
            .swap_xy = 0,
            .mirror_x = 0,
            .mirror_y = 0,
        },
        .driver_data = &tp_gt911_config
    };

    esp_lcd_touch_handle_t tp = NULL;
    ret = esp_lcd_touch_new_i2c_gt911(tp_io_handle, &tp_cfg, &tp);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create touch controller: %s", esp_err_to_name(ret));
        return ret;
    }

    const lvgl_port_touch_cfg_t touch_cfg = {
        .disp = disp,
        .handle = tp,
    };

    disp_indev = lvgl_port_add_touch(&touch_cfg);
    if (disp_indev == NULL) {
        ESP_LOGE(TAG, "Failed to setup touch input device");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "BSP initialized successfully");
    return ESP_OK;
}

/* =========================================================
 *                  BACKLIGHT CONTROL API
 * ========================================================= */

esp_err_t bsp_display_backlight_on(void)
{
    esp_err_t ret = gpio_set_level(BSP_LCD_BACKLIGHT_GPIO, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable backlight: %s", esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t bsp_display_backlight_off(void)
{
    esp_err_t ret = gpio_set_level(BSP_LCD_BACKLIGHT_GPIO, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to disable backlight: %s", esp_err_to_name(ret));
    }
    return ret;
}

/* =========================================================
 *                  INITIALIZATION API
 * ========================================================= */
lv_display_t *bsp_display_start(void)
{
    if (bsp_init() != ESP_OK) {
        return NULL;
    }
    return disp;
}