/*
 * ESP32-S3-4848S040 Board Support Package - Header
 *
 * SPDX-FileCopyrightText: 2026 Victor Arzolla
 *
 * SPDX-License-Identifier: MIT
 *
 * Public API for hardware initialization and abstraction.
 */

#pragma once

#include <esp_err.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <lvgl.h>

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================
 *                  BOARD INFORMATION
 * ========================================================= */

/**
 * @defgroup BSP_BOARD Board Information
 * @brief Board identification and capabilities
 * @{
 */

/** @brief Board name */
#define BSP_BOARD_ESP32_S3_4848S040

/**
 * @defgroup BSP_CAPS Capabilities
 * @brief Supported features on this board
 * @{
 */
#define BSP_CAPS_DISPLAY                1
#define BSP_CAPS_TOUCH                  1
#define BSP_CAPS_BACKLIGHT              1
/** @} */

/** @} */ // end of BSP_BOARD

/* =========================================================
 *                  LCD BACKLIGHT PINS
 * ========================================================= */

/**
 * @defgroup BSP_LCD_BACKLIGHT LCD Backlight
 * @brief LCD backlight GPIO
 * @{
 */

#define BSP_LCD_BACKLIGHT_GPIO          38

/** @} */ // end of BSP_LCD_BACKLIGHT

/* =========================================================
 *                  LCD SPI PINS
 * ========================================================= */

/**
 * @defgroup BSP_LCD_SPI LCD SPI Configuration
 * @brief 3-wire SPI interface for LCD
 * @{
 */

#define BSP_LCD_SPI_CS_GPIO             39
#define BSP_LCD_SPI_SCK_GPIO            48
#define BSP_LCD_SPI_MOSI_GPIO           47

/** @} */ // end of BSP_LCD_SPI

/* =========================================================
 *                  LCD RGB PINS
 * ========================================================= */

/**
 * @defgroup BSP_LCD_RGB LCD RGB Interface
 * @brief RGB panel data and control lines
 * @{
 */

#define BSP_LCD_DE_GPIO                 18
#define BSP_LCD_HSYNC_GPIO              16
#define BSP_LCD_VSYNC_GPIO              17
#define BSP_LCD_PCLK_GPIO               21
#define BSP_LCD_DISP_GPIO               GPIO_NUM_NC
#define BSP_LCD_RST_GPIO                GPIO_NUM_NC

/** @defgroup BSP_LCD_RGB_RED Red Channel
 *  @{
 */
#define BSP_LCD_R0_GPIO                 11
#define BSP_LCD_R1_GPIO                 12
#define BSP_LCD_R2_GPIO                 13
#define BSP_LCD_R3_GPIO                 14
#define BSP_LCD_R4_GPIO                 0
/** @} */

/** @defgroup BSP_LCD_RGB_GREEN Green Channel
 *  @{
 */
#define BSP_LCD_G0_GPIO                 8
#define BSP_LCD_G1_GPIO                 20
#define BSP_LCD_G2_GPIO                 3
#define BSP_LCD_G3_GPIO                 46
#define BSP_LCD_G4_GPIO                 9
#define BSP_LCD_G5_GPIO                 10
/** @} */

/** @defgroup BSP_LCD_RGB_BLUE Blue Channel
 *  @{
 */
#define BSP_LCD_B0_GPIO                 4
#define BSP_LCD_B1_GPIO                 5
#define BSP_LCD_B2_GPIO                 6
#define BSP_LCD_B3_GPIO                 7
#define BSP_LCD_B4_GPIO                 15
/** @} */

/** @} */ // end of BSP_LCD_RGB

/* =========================================================
 *                  LCD PANEL CONFIGURATION
 * ========================================================= */

/**
 * @defgroup BSP_LCD_PANEL LCD Panel Configuration
 * @brief Display resolution and bit depth
 * @{
 */

#define BSP_LCD_H_RES                   480
#define BSP_LCD_V_RES                   480
#define BSP_LCD_BITS_PER_PIXEL           18

/** @} */ // end of BSP_LCD_PANEL

/* =========================================================
 *                  TOUCH PANEL PINS
 * ========================================================= */

/**
 * @defgroup BSP_TOUCH Touch Panel
 * @brief GT911 touch controller interface
 * @{
 */

#define BSP_TOUCH_I2C_SCL_GPIO          45
#define BSP_TOUCH_I2C_SDA_GPIO          19
#define BSP_TOUCH_INT_GPIO              GPIO_NUM_NC
#define BSP_TOUCH_RST_GPIO              GPIO_NUM_NC
#define BSP_TOUCH_I2C_CLK_HZ            400000

/** @} */ // end of BSP_TOUCH

/* =========================================================
 *                  LVGL CONFIGURATION
 * ========================================================= */

/**
 * @defgroup BSP_LVGL LVGL Configuration
 * @brief LVGL graphics library settings
 * @{
 */

#define BSP_LVGL_BUFFER_SIZE            (BSP_LCD_H_RES * BSP_LCD_V_RES)
#define BSP_LVGL_DOUBLE_BUFFER          true

/** @} */ // end of BSP_LVGL

/* =========================================================
 *                  INITIALIZATION API
 * ========================================================= */

/** \addtogroup BSP_INIT
 *  @brief Complete BSP initialization
 *  @{
 */

/**
 * @brief Initialize all BSP components
 *
 * Performs complete hardware initialization in the following order:
 *  1. LCD backlight GPIO configuration (disabled)
 *  2. LCD panel with ST7701 driver initialization
 *  3. LVGL graphics library setup
 *  4. LCD backlight enable
 *  5. Touch panel (GT911) initialization
 *
 * @return
 *      - ESP_OK:   All components initialized successfully
 *      - ESP_FAIL: One or more components failed. Check logs for details.
 *
 * @note This is the main initialization function for typical usage
 * @note Call bsp_display_lock()/bsp_display_unlock() before using LVGL API
 *
 * @see bsp_display_start()
 * @see bsp_display_lock()
 * @see bsp_display_unlock()
 */
esp_err_t bsp_init(void);

/**
 * @brief Initialize display and return LVGL display handle
 *
 * This is a convenience function that calls bsp_init() and returns
 * the LVGL display handle. Equivalent to calling bsp_init() followed
 * by bsp_display_get_handle().
 *
 * @return
 *      - Pointer to LVGL display on success
 *      - NULL on failure
 *
 * @note This is the recommended function for typical LVGL applications
 *
 * @see bsp_init()
 * @see bsp_display_get_handle()
 */
lv_display_t *bsp_display_start(void);

/** @} */ // end of BSP_INIT

/* =========================================================
 *                  BACKLIGHT CONTROL API
 * ========================================================= */

/** \addtogroup BSP_DISPLAY_BACKLIGHT
 *  @brief LCD backlight control
 *  @{
 */

/**
 * @brief Turn on LCD backlight
 *
 * @return
 *      - ESP_OK:   Backlight enabled
 *      - ESP_FAIL: GPIO operation failed
 *
 * @see bsp_display_backlight_off()
 */
esp_err_t bsp_display_backlight_on(void);

/**
 * @brief Turn off LCD backlight
 *
 * @return
 *      - ESP_OK:   Backlight disabled
 *      - ESP_FAIL: GPIO operation failed
 *
 * @see bsp_display_backlight_on()
 */
esp_err_t bsp_display_backlight_off(void);

/** @} */ // end of BSP_DISPLAY_BACKLIGHT

/* =========================================================
 *                  LVGL SYNCHRONIZATION API
 * ========================================================= */

/** \addtogroup BSP_DISPLAY_LOCK
 *  @brief LVGL mutex lock/unlock operations
 *  @{
 */

/**
 * @brief Take LVGL mutex
 *
 * Acquires the LVGL mutex lock before calling any LVGL API functions.
 * LVGL is not thread-safe, so proper locking is essential.
 *
 * @param[in] timeout_ms Timeout in milliseconds. 0 will block indefinitely.
 *
 * @return
 *      - true:  Mutex was taken
 *      - false: Mutex was NOT taken (timeout)
 *
 * @note Display must be already initialized by calling bsp_init() or bsp_display_start()
 * @note Always pair with bsp_display_unlock()
 *
 * @see bsp_display_unlock()
 */
bool bsp_display_lock(uint32_t timeout_ms);

/**
 * @brief Give LVGL mutex
 *
 * Releases the LVGL mutex lock after LVGL API operations are complete.
 *
 * @note Display must be already initialized by calling bsp_init() or bsp_display_start()
 * @note Must be called after bsp_display_lock()
 *
 * @see bsp_display_lock()
 */
void bsp_display_unlock(void);

/** @} */ // end of BSP_DISPLAY_LOCK

#ifdef __cplusplus
}
#endif