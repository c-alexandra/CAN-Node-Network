// External libraries
// #include <stdio.h> // printf
#include "freertos/FreeRTOS.h" // 
#include "freertos/task.h" // 
// #include "esp_system.h"
// #include "driver/gpio.h" // gpio functions
#include "esp_log.h" // ESP_LOGI
// #include "esp_err.h"
// #include "esp_timer.h" // esp_timer_get_time

// // private libraries & headers
// #include "common-defines.h"
// #include "display.h"
#include "lcd16x2.h"
#include "temp_sensor.h"

#define LED_BUILTIN (GPIO_NUM_2)

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
static void example_basic_lcd_usage(void* pvParameter) {
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

static void blink_task(void *pvParameter)
{
    gpio_reset_pin(LED_BUILTIN); // configure as gpio
    gpio_set_direction(LED_BUILTIN, GPIO_MODE_OUTPUT);

    while (1)
    {
        gpio_set_level(LED_BUILTIN, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        gpio_set_level(LED_BUILTIN, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

static void example_temp_sensor(void* pvParameter) {
    const char* TAG = "temp_sensor";

    temp_sensor_handle_t temp_handle;
    temp_sensor_config_t temp_config = {
        .v_out_pin = GPIO_NUM_39,
        .sensor_type = TEMP_SENSOR_TYPE_LM35,
        .sensor_config.linear.offset = 0.0f,
        .sensor_config.linear.scale = 0.010f,
        .adc_unit = ADC_UNIT_1,
        .adc_channel = ADC_CHANNEL_3,
        .adc_attenuation = ADC_ATTEN_DB_0, // no attenuation
        .adc_bitwidth = ADC_BITWIDTH_12,
        .filter_type = TEMP_FILTER_NONE,
        .avg_samples = TEMP_SENSOR_DEFAULT_AVG_SAMPLES,
        .sample_rate_ms = TEMP_SENSOR_DEFAULT_SAMPLE_RATE_MS,
        .continuous_mode = false,
        .calibration_offset = 0.0f,
        .calibration_scale = 1.0f
    };

    esp_err_t ret = temp_sensor_init(&temp_config, &temp_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize LM35 sensor");
        return;
    }

    float temperature = 0.0f;

    while (1) {
        temp_sensor_read(temp_handle, &temperature);
        ESP_LOGI(TAG,"Temperature reading: %f", temperature);
        // printf("Temperature reading: %f\n", temperature);
        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
}

void app_main()
{
    xTaskCreate(&example_basic_lcd_usage, "lcd_example", 4096, NULL, 5, NULL);
    xTaskCreate(&blink_task, "blink_task", 2048, NULL, 5, NULL);
    xTaskCreate(&example_temp_sensor, "temp_sensor", 4096, NULL, 5, NULL);
    // xTaskCreate(&log_task, "log", 2048, NULL, 5, NULL);
    // xTaskCreate(&hello_task, "hello", 1024, NULL, 5, NULL);
    // xTaskCreate(&display_example, "display", 2048, NULL, 5, NULL);
}