/*
 * ESP32-S3-4848S040 Board Support Package - Header
 *
 * SPDX-FileCopyrightText: 2025 Hugo Trippaers
 * SPDX-FileCopyrightText: 2026 Victor Arzolla
 *
 * SPDX-License-Identifier: MIT
 *
 * This file is derived from work originally licensed under Apache-2.0.
 * Original source: https://github.com/spark404/esp32-4848S040-base
 *
 * Public API for hardware initialization and abstraction.
 */

#ifndef BSP_H
#define BSP_H

#include <esp_err.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <lvgl.h>



/**
 * @brief Initialize LCD backlight GPIO
 * @return ESP_OK on success
 */
esp_err_t bsp_init_lcd_backlight(void);

/**
 * @brief Enable LCD backlight
 * @return ESP_OK on success
 */
esp_err_t bsp_enable_lcd_backlight(void);

/**
 * @brief Initialize LCD panel with ST7701 driver
 * @param[out] io_handle Panel IO handle
 * @param[out] panel_handle Panel handle
 * @return ESP_OK on success
 */
esp_err_t bsp_init_lcd_panel(esp_lcd_panel_io_handle_t *io_handle, 
                              esp_lcd_panel_handle_t *panel_handle);

/**
 * @brief Initialize LVGL
 * @param[out] disp_handle Display handle
 * @param[in] io_handle Panel IO handle
 * @param[in] panel_handle Panel handle
 * @return ESP_OK on success
 */
esp_err_t bsp_init_lvgl(lv_disp_t **disp_handle, esp_lcd_panel_io_handle_t io_handle,
                         esp_lcd_panel_handle_t panel_handle);

/**
 * @brief Initialize touch panel
 * @param[out] touch_handle Touch input device handle
 * @param[in] disp_handle Display handle
 * @return ESP_OK on success
 */
esp_err_t bsp_init_touch(lv_indev_t **touch_handle, lv_disp_t *disp_handle);

/**
 * @brief Initialize all hardware components
 * Encapsulates all BSP initialization functions (LCD, LVGL, touch, backlight)
 * @return ESP_OK on success
 */
esp_err_t bsp_init(void);

#endif //BSP_H