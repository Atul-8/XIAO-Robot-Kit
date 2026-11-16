#pragma once

#include <stdint.h>

#include "esp_err.h"

/*
 * 10-pixel addressable RGB strip (WS2812/WS2812B class) on D13/GPIO40, J7.
 * The MCU drives the data line at 3.3 V; a MOSFET level-shifts it to the strip's
 * 5 V domain and is non-inverting, so standard WS2812 timing applies.
 */
#define RGB_LED_PIXEL_COUNT 10 /* one LED per strip segment; GPIO40 comes from XiaoRobotKitPins.h */

esp_err_t rgb_led_init(void);
esp_err_t rgb_led_set_pixel(uint8_t index, uint8_t red, uint8_t green, uint8_t blue);
esp_err_t rgb_led_fill(uint8_t red, uint8_t green, uint8_t blue);
esp_err_t rgb_led_show(void);
esp_err_t rgb_led_rainbow(void);
esp_err_t rgb_led_off(void);
