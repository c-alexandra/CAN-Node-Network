/*******************************************************************************
 * @file temp_sensor.h
 * @brief Temperature sensor driver for ESP32 supporting ADC-based sensors
 * 
 * This library supports LM35 and NTC package type temperature sensors, and 
 * features various calibration settings, filter types, and thread-safe
 * interface functions.
 * 
 * 
 * @author Camille Aitken
 * @version 1.0.0
 * 
 * @copyright MIT License
 ******************************************************************************/
#pragma once

#include "driver/gpio.h"
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_cali.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_check.h"


#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * CONSTANTS & MACROS
 ******************************************************************************/

// Temperature sensor error codes
#define ESP_ERR_TEMP_BASE             (0x9000)
#define ESP_ERR_TEMP_INVALID_CONFIG   (ESP_ERR_TEMP_BASE + 1)
#define ESP_ERR_TEMP_NOT_INITIALIZED  (ESP_ERR_TEMP_BASE + 2)
#define ESP_ERR_TEMP_ADC_CONFIG       (ESP_ERR_TEMP_BASE + 3)
#define ESP_ERR_TEMP_OUT_OF_RANGE     (ESP_ERR_TEMP_BASE + 4)
#define ESP_ERR_TEMP_CAL_FAILED       (ESP_ERR_TEMP_BASE + 5)

// Default configuration values
#define TEMP_SENSOR_DEFAULT_AVG_SAMPLES     (10)
#define TEMP_SENSOR_DEFAULT_SAMPLE_RATE_MS  (100)
#define TEMP_SENSOR_MAX_FILTERS             (5)

// Temperature limits (Celsius)
// #define TEMP_SENSOR_MIN_VALID    (-40.0f) // if using +/- configuration
#define TEMP_SENSOR_MIN_VALID    (0.0f)
#define TEMP_SENSOR_MAX_VALID    (150.0f)

/*******************************************************************************
 * TYPE DEFINITIONS
 ******************************************************************************/
// NTC sensor configuration requirements
typedef struct {
    float beta_coefficient;     // beta coefficient (typically 3000-4000K)
    float nominal_resistance;   // resistance at reference temperature (ohms)
    float series_resistance;    // series resistor value in voltage divider (ohms)
    bool pullup_configuration;  // true if NTC is connected to ground, false if to Vcc
    
    // TODO: Research how to implement these
    // Optional Steinhart-Hart coefficients for higher accuracy
    // float steinhart_a;                  
    // float steinhart_b;                  
    // float steinhart_c;                  
    // bool use_steinhart;                 
} temp_sensor_ntc_config_t;

typedef struct {
    float offset; 
    float scale;  // eg. LM35 10mv/°C
} temp_sensor_linear_config_t;

typedef struct {
    float curr_temp;
    float min_temp;
    float max_temp;
    float avg_temp; 
    uint32_t sample_count;    // total number of counted samples
    uint32_t error_count;     // number of read errors
    int64_t last_update_time; // timestamp of last sensor update (µs)
} temp_sensor_stats_t;

typedef enum {
    TEMP_SENSOR_TYPE_NTC, // ntc thermistor-voltage divider
    TEMP_SENSOR_TYPE_LM35 // lm35 linear voltage sensor
} temp_sensor_type_t;

typedef enum {
    TEMP_FILTER_NONE,
    TEMP_FILTER_MOVING_AVG,
    TEMP_FILTER_MEDIAN,
    // TEMP_FILTER_KALMAN // TODO: another common filter type to research
} temp_filter_type_t;

typedef struct {
    // gpio pin configuration
    gpio_num_t v_out_pin;

    // sensor configuration
    temp_sensor_type_t sensor_type;
    union {
        temp_sensor_linear_config_t linear;
        temp_sensor_ntc_config_t ntc;
    } sensor_config;

    // adc configuration
    adc_unit_t adc_unit;
    adc_channel_t adc_channel;
    adc_atten_t adc_attenuation;
    adc_bitwidth_t adc_bitwidth;

    // filter configuration
    temp_filter_type_t filter_type;
    uint8_t avg_samples; // number of samples to use to calc avg

    // sample rate configuration
    uint32_t sample_rate_ms;
    bool continuous_mode; // continuous or one-shot mode flag

    // sensor calibration
    float calibration_offset; // (°C)
    float calibration_scale;  // determined by datasheet (eg. lm35 10mV/°C)
} temp_sensor_config_t;

typedef struct temp_sensor_s *temp_sensor_handle_t;

/*******************************************************************************
 * FUNCTION PROTOTYPES
 ******************************************************************************/
esp_err_t temp_sensor_init(const temp_sensor_config_t *config, 
    temp_sensor_handle_t *handle);

esp_err_t temp_sensor_read(temp_sensor_handle_t handle, float* temperature);

esp_err_t temp_sensor_read_filtered(temp_sensor_handle_t handle, float* temperature);

esp_err_t temp_sensor_get_stats(temp_sensor_handle_t handle, temp_sensor_stats_t* stats);



#ifdef __cplusplus
}
#endif