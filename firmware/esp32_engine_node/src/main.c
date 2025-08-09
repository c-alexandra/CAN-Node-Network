#include <stdio.h> // printf
#include "freertos/FreeRTOS.h" // 
// #include "freertos/task.h" // 
// #include "esp_system.h"
#include "driver/gpio.h" // gpio functions
#include "esp_log.h" // ESP_LOGI

#define LED_BUILTIN (GPIO_NUM_2)

static const char *TAG = "myModule"; // defined for log, per .c file

void hello_task(void *pvParameter)
{
    while (1)
    {
        printf("Hello, World!\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void log_task(void *pvParameter)
{
    while (1)
    {
        ESP_LOGI(TAG, "Log testing...");
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}

void blink_task(void *pvParameter)
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

void app_main()
{
    xTaskCreate(&blink_task, "blink_task", 2048, NULL, 5, NULL);
    xTaskCreate(&log_task, "log", 2048, NULL, 5, NULL);
    xTaskCreate(&hello_task, "hello", 1024, NULL, 5, NULL);
}