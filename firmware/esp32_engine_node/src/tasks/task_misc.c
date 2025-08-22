#include "common-defines.h"

#define LED_BUILTIN (GPIO_NUM_2)


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