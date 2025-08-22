/*******************************************************************************
 * @file main.c
 * @brief 
 * 
 * 
 * @author Camille Aitken
 * @version 1.0.0
 * 
 * @copyright MIT License
 ******************************************************************************/
#include "common-defines.h"
#include "tasks/lcd_module/task_lcd.h"
#include "tasks/temperature_sensor/task_temp_sensor.h"
#include "tasks/task_misc.h"

void app_main()
{
    xTaskCreate(&example_basic_lcd_usage, "lcd_example", 4096, NULL, 5, NULL);
    xTaskCreate(&blink_task, "blink_task", 2048, NULL, 5, NULL);
    xTaskCreate(&example_temp_sensor, "temp_sensor", 4096, NULL, 5, NULL);
}