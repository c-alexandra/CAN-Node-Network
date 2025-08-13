/*******************************************************************************
 * @file lcd16x2.h
 * @brief 16x2 character LCD driver for esp32
 * 
 * This library provides a comprehensive interface for controlling a 16x2 char
 * display in either 4-bit or 8-bit parallel communication mode.
 * 
 * Features:
 * - Support for both 4-bit and 8-bit parallel data interace
 * - thread-safe operation using FreeRTOS mutexes
 * - Multiple instance support
 * - optional direct GPIO register access for performance
 * 
 * @author Camille Aitken
 * @version 1.0.0
 * 
 * @copyright MIT License
 ******************************************************************************/
#pragma once

// #include "common-defines.h"
#include <stdint.h>
#include <stdbool.h>
#include "driver/gpio.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h" 
#include "freertos/semphr.h" // semaphore_handle_t

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * PREPROCESSOR DIRECTIVES
 ******************************************************************************/
#define LCD16X2_DEFAULT_TIMING() { \
    .enable_pulse_us = 1, \
    .command_delay_us = 37, \
    .clear_delay_ms = 2, \
    .init_delay_ms = 50 \
}

// LCD Error Codes 
#define ESP_ERR_LCD_BASE                (0x8000)
#define ESP_ERR_LCD_INVALID_MODE        (ESP_ERR_LCD_BASE + 1)
#define ESP_ERR_LCD_INVALID_POSITION    (ESP_ERR_LCD_BASE + 2)
#define ESP_ERR_LCD_NOT_INITIALIZED     (ESP_ERR_LCD_BASE + 3)
#define ESP_ERR_LCD_GPIO_CONFIG         (ESP_ERR_LCD_BASE + 4)
#define ESP_ERR_LCD_TIMEOUT             (ESP_ERR_LCD_BASE + 5)

/*******************************************************************************
 * TYPE DEFINITIONS
 ******************************************************************************/
// lcd interface modes
typedef enum {
    LCD16X2_BITMODE_8 = 0,
    LCD16X2_BITMODE_4 = 1
} lcd16x2_mode_t;

// timing configuration structure (MAY NEED ADJUSTMENT PER DATASHEET)
typedef struct {
    uint32_t enable_pulse_us;   // enable pulse width (min 1µs as per datasheet)
    uint32_t command_delay_us;  // delay after commands (min 37µs)
    uint32_t clear_delay_ms;    // delay after clear command (min 1.52ms)
    uint32_t init_delay_ms;     // initial power-on delay (min 15ms
} lcd16x2_timing_t;

// lcd interface configuration structure
typedef struct {
    gpio_num_t rs_pin;
    gpio_num_t rw_pin;
    gpio_num_t enable_pin;
    gpio_num_t data_pins[8];   // up to 8 data pins, use last 4 for 4-bit mode
    gpio_num_t backlight_pin;  // Backlight control pin (if enabled/connected) 
    lcd16x2_timing_t timing;   // Timing configuration 
    bool backlight_enable;     // Enable backlight control 
    lcd16x2_mode_t bit_mode;   // Interface mode (4-bit or 8-bit) 

} lcd16x2_config_t;

// Opaque pointer handler in line with ESP-IDF patterns
typedef struct lcd16x2_s *lcd16x2_handle_t;

/*******************************************************************************
 * FUNCTION PROTOTYPES
 ******************************************************************************/
esp_err_t lcd16x2_init(const lcd16x2_config_t *config, lcd16x2_handle_t *handle);
esp_err_t lcd16x2_validate_pins(const gpio_num_t *pins, size_t pin_count);
// void lcd16x2_teardown(lcd16x2_handle_t handle);

// void lcd16x2_print(lcd16x2_handle_t handle, const char*);
// void lcd16x2_clear(lcd16x2_handle_t handle);
// void lcd16x2_home(lcd16x2_handle_t handle);

// void lcd16x2_example(void *pvParamater);

#ifdef __cplusplus
}
#endif
