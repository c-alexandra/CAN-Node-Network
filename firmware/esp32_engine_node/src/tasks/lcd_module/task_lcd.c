#include "common-defines.h"
#include "lcd16x2.h"
#include "temp_sensor.h"

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

static const char* TAG = "lcd_task";

temp_sensor_handle_t temp_handle;
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

temp_sensor_config_t temp_config = {
    .v_out_pin = GPIO_NUM_39,
    .sensor_type = TEMP_SENSOR_TYPE_LM35,
    .sensor_config.linear.offset = 0.0f,
    .sensor_config.linear.scale = 0.010f,
    .adc_unit = ADC_UNIT_1,
    .adc_channel = ADC_CHANNEL_3,
    .adc_attenuation = ADC_ATTEN_DB_0, // no attenuation
    .adc_bitwidth = ADC_BITWIDTH_12,
    .filter_type = TEMP_FILTER_MOVING_AVG,
    .avg_samples = TEMP_SENSOR_DEFAULT_AVG_SAMPLES,
    .sample_rate_ms = TEMP_SENSOR_DEFAULT_SAMPLE_RATE_MS,
    .continuous_mode = false,
    .calibration_offset = 0.0f,
    .calibration_scale = 1.0f
};

// TODO: refactor to proper test code
void example_basic_lcd_usage(void* pvParameter) {
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

esp_err_t init_temp_display() {
    esp_err_t ret = lcd16x2_init(&lcd_config, &lcd_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize example lcd: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = temp_sensor_init(&temp_config, &temp_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize LM35 sensor");
        return ret;
    }

    return ESP_OK;
}

void display_temp_on_lcd(void *pvParameter) {
    init_temp_display();

    float temperature = 0.0f;

    while (1) {
        lcd16x2_set_cursor(lcd_handle, 0, 0);
        temp_sensor_read(temp_handle, &temperature);
        // TODO implement custom character display in lcd lib
        lcd16x2_printf(lcd_handle, "temp: %.2fC", temperature);
        temp_sensor_read_filtered(temp_handle, &temperature);
        lcd16x2_set_cursor(lcd_handle, 1, 0);
        lcd16x2_printf(lcd_handle, " avg: %.2fC", temperature);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        // lcd16x2_clear(lcd_handle);
    }
}