file(GLOB SRCS "src/*.c")

idf_component_register(
        SRCS ${SRCS}
        INCLUDE_DIRS "include"
        REQUIRES        esp_lvgl_port
        PRIV_REQUIRES   esp_lcd 
                        esp_driver_gpio
                        esp_driver_i2c
                        esp_lcd_st7701
                        esp_lcd_touch_gt911
)       