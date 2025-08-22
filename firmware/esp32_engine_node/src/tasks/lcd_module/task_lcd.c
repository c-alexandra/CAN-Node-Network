#include "common-defines.h"
#include "lcd16x2.h"

// Display configuration
#define RS_PIN (GPIO_NUM_4)
#define RW_PIN (GPIO_NUM_NC)
#define E_PIN (GPIO_NUM_32)
#define DATA_PIN_D0 (GPIO_NUM_NC)
#define DATA_PIN_D1 (GPIO_NUM_NC)
#define DATA_PIN_D2 (GPIO_NUM_NC)
#define DATA_PIN_D3 (GPIO_NUM_NC)
#define DATA_PIN_D4 (GPIO_NUM_21)
#define DATA_PIN_D5 (GPIO_NUM_22)
#define DATA_PIN_D6 (GPIO_NUM_23)
#define DATA_PIN_D7 (GPIO_NUM_25)
#define BACKLIGHT_PIN (GPIO_NUM_19)
#define BACKLIGHT_ENABLE true
#define BIT_MODE LCD16X2_BITMODE_4

// TODO: refactor to proper test code
void example_basic_lcd_usage(void* pvParameter) {
    const char* TAG = "LCD";

    lcd16x2_handle_t lcd_handle;
    lcd16x2_config_t lcd_config = {
        .rs_pin = RS_PIN,
        .rw_pin = RW_PIN,
        .enable_pin = E_PIN,
        .backlight_enable = BACKLIGHT_ENABLE,
        .backlight_pin = BACKLIGHT_PIN,
        .data_pins = {DATA_PIN_D0, DATA_PIN_D1, DATA_PIN_D2, DATA_PIN_D3, 
                      DATA_PIN_D4, DATA_PIN_D5, DATA_PIN_D6, DATA_PIN_D7},
        .bit_mode = BIT_MODE,
        .timing = LCD16X2_DEFAULT_TIMING()
    };

    esp_err_t ret = lcd16x2_init(&lcd_config, &lcd_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize example lcd: %s", esp_err_to_name(ret));
        return;
    }

    while (1) {
        // ESP_LOGI(TAG, "Should be writing to LCD");
        lcd16x2_write_string_at(lcd_handle, 0, 0, "Hello, World!");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        lcd16x2_clear(lcd_handle); // Clear the display
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}