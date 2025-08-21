/*******************************************************************************
 * @file temp_sensor.c
 * @brief Temperature sensor driver implementation for ESP32
 * 
 * 
 * @author Camille Aitken
 * @version 1.0.0
 * 
 * @copyright MIT License
 ******************************************************************************/
#include "esp_check.h"
#include "temp_sensor.h"
#include "freertos/task.h"
#include "freertos/semphr.h" // semaphore_handle_t
#include "esp_timer.h"

/*******************************************************************************
 * PRIVATE CONSTANTS & MACROS
 ******************************************************************************/
#define KELVIN_OFFSET           (273.15f)
#define ADC_MAX_VOLTAGE_MV      (3300)     // max accepted ADC voltage in millivolts
#define LM35_MV_PER_DEGREE      (10.0f)    // LM35 10mV/°C

static const char *TAG = "temp_sensor"; 

// Validation macros
#define TEMP_SENS_CHECK(condition, err_code, format, ...) do { \
    if (!(condition)) { \
        ESP_LOGE(TAG, format, ##__VA_ARGS__); \
        return err_code; \
    } \
} while(0)

#define TEMP_SENS_CHECK_HANDLE(handle) \
    TEMP_SENS_CHECK(handle != NULL, ESP_ERR_INVALID_ARG, "Handle is NULL"); \
    TEMP_SENS_CHECK(handle->initialized, ESP_ERR_TEMP_NOT_INITIALIZED, "Temperature sensor not initialized")

/*******************************************************************************
 * PRIVATE TYPE DEFINITIONS
 ******************************************************************************/
// Opaque handle struct
typedef struct temp_sensor_s {
    // hardware configuration
    gpio_num_t v_out_pin;

    temp_sensor_config_t sensor_config;

    // adc resources
    adc_oneshot_unit_handle_t adc_handle; // unit handle
    adc_cali_handle_t adc_cali_handle;    // calibration handle
    bool adc_calibrated;                  // adc calibration status

    // filtering
    float* filter_buffer; // buffer to store filter samples
    uint8_t filter_index; // current position in filter buffer
    uint8_t filter_count; // number of valid samples in buffer
    float filtered_value; // last filtered temp value

    // statistics
    temp_sensor_stats_t stats;

    // continuous mode (if enabled)
    TaskHandle_t task_handle;
    bool continuous_running;
    
    // State management
    SemaphoreHandle_t mutex;    // Thread safety mutex */
    bool initialized;           // Initialization status */
} temp_sensor_t;

/*******************************************************************************
 * PRIVATE FUNCTION PROTOTYPES
 ******************************************************************************/
static esp_err_t adc_init(temp_sensor_handle_t handle);
static esp_err_t adc_deinit(temp_sensor_handle_t handle);
static void update_stats(temp_sensor_handle_t handle, float temp);
static esp_err_t read_adc_voltage(temp_sensor_handle_t handle, int* mv_voltage);
static void update_statistics(temp_sensor_handle_t handle, float temperature);
static float convert_voltage_to_temperature(temp_sensor_handle_t handle, int voltage_mv);
static esp_err_t calculate_ntc_temperature(int voltage_mv, const temp_sensor_ntc_config_t* config);

/*******************************************************************************
 * PUBLIC FUNCTION IMPLEMENTATIONS
 ******************************************************************************/

/**
 * @brief initialize temperature sensor module with given configuration
 * 
 * @param config configuration template for 
 * @param handle 
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t temp_sensor_init(const temp_sensor_config_t *config, 
    temp_sensor_handle_t *handle) {
    TEMP_SENS_CHECK(config != NULL, ESP_ERR_INVALID_ARG, "Config is NULL");
    TEMP_SENS_CHECK(handle != NULL, ESP_ERR_INVALID_ARG, "Handle pointer is NULL");

    // allocate space for handle 
    temp_sensor_t* sensor = calloc(1, sizeof(temp_sensor_t));
    TEMP_SENS_CHECK(sensor != NULL, ESP_ERR_NO_MEM, "Failed to allocate sensor memory");

    memcpy(&sensor->sensor_config, config, sizeof(temp_sensor_config_t));

    // initialize statistics
    sensor->stats.min_temp     = TEMP_SENSOR_MIN_VALID;
    sensor->stats.max_temp     = TEMP_SENSOR_MAX_VALID;
    sensor->stats.curr_temp    = 0.0f;
    sensor->stats.avg_temp     = 0.0f;
    sensor->stats.sample_count = 0;
    sensor->stats.error_count  = 0;

    // allocate filter buffer if enabled
    if (config->filter_type != TEMP_FILTER_NONE && config->avg_samples > 0) {
        sensor->filter_buffer = calloc(config->avg_samples, sizeof(float));
        if (sensor->filter_buffer == NULL) {
            free(sensor);
            return ESP_ERR_NO_MEM;
        }
        sensor->filter_index = 0;
        sensor->filter_count = 0;
    }

    // Create mutex for thread safety
    sensor->mutex = xSemaphoreCreateMutex();
    if (sensor->mutex == NULL) {
        if (sensor->filter_buffer) {
            free(sensor->filter_buffer);
        }
        free(sensor);
        return ESP_ERR_NO_MEM;
    }

    // initialize adc
    esp_err_t ret = adc_init(sensor);
    if (ret != ESP_OK) {
        vSemaphoreDelete(sensor->mutex);
        if (sensor->filter_buffer) {
            free(sensor->filter_buffer);
        }
        free(sensor);
        return ret;
    }

    sensor->initialized = true;
    *handle = sensor;

    // TODO: implement continuous mode
    // optionally start continuous mode
    if (config->continuous_mode) {

    }

    ESP_LOGI(TAG, "Temp sensor is initialized: type=%d, channel=%d, filter=%d", config->sensor_type, config->adc_channel, config->filter_type);

    return ESP_OK;
}

esp_err_t temp_sensor_deinit(temp_sensor_handle_t handle) {
    TEMP_SENS_CHECK_HANDLE(handle);

    // stop continuous mode if running 
    if (handle->continuous_running) {
        temp_sensor_stop_continuous(handle);
    }

    // take mutex
    if (xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    // deinit adc
    adc_deinit(handle);

    if (handle->filter_buffer) {
        free(handle->filter_buffer);
    }

    handle->initialized = false;
    xSemaphoreGive(handle->mutex);
    vSemaphoreDelete(handle->mutex);
    free(handle);

    ESP_LOGI(TAG, "Temperature sensor deinitialized");
    return ESP_OK;
}

/**
 * @brief performs a single temperature read
 * 
 * @param handle temperature sensor handle
 * @param temperature pointer to temperature float to be set by function
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t temp_sensor_read(temp_sensor_handle_t handle, float* temperature) {
    TEMP_SENS_CHECK_HANDLE(handle);
    TEMP_SENS_CHECK(temperature != NULL, ESP_ERR_INVALID_ARG, "Temperature pointer is NULL");

    if (xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    // read adc voltage
    int voltage_mv;
    esp_err_t ret = read_adc_voltage(handle, &voltage_mv);
    if (ret != ESP_OK) {
        handle->stats.error_count++;
        xSemaphoreGive(handle->mutex);
        return ret;
    }

    // convert to readable temperature
    float temp = convert_voltage_to_temperature(handle, voltage_mv);

    // apply calibration
    temp = (temp * handle->sensor_config.calibration_scale) + handle->sensor_config.calibration_offset;

    // Validate temperature range
    if (temp < TEMP_SENSOR_MIN_VALID || temp > TEMP_SENSOR_MAX_VALID) {
        handle->stats.error_count++;
        xSemaphoreGive(handle->mutex);
        ESP_LOGW(TAG, "Temperature out of range: %.2f°C", temp);
        return ESP_ERR_TEMP_OUT_OF_RANGE;
    }
    
    // Update statistics
    update_statistics(handle, temp);
    
    *temperature = temp;
    handle->stats.curr_temp = temp;
    handle->stats.last_update_time = esp_timer_get_time();

    xSemaphoreGive(handle->mutex);
    return ESP_OK;
}
/*******************************************************************************
 * PRIVATE FUNCTION IMPLEMENTATIONS
 ******************************************************************************/

/**
 * @brief initialize adc module for temperature sensor
 * 
 * @param handle temperature sensor handle
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t adc_init(temp_sensor_handle_t handle) {
    esp_err_t ret;

    // configure adc unit
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = handle->sensor_config.adc_unit,
        .ulp_mode = ADC_ULP_MODE_DISABLE // disable low power mode
    };

    ret = adc_oneshot_new_unit(&init_config, &handle->adc_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize ADC unit: %s", esp_err_to_name(ret));
        return ESP_ERR_TEMP_ADC_CONFIG;
    }

    // configure adc channel
    adc_oneshot_chan_cfg_t channel_config = {
        .bitwidth = handle->sensor_config.adc_bitwidth,
        .atten = handle->sensor_config.adc_attenuation
    };

    ret = adc_oneshot_config_channel(handle->adc_handle, handle->sensor_config.adc_channel, &channel_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure ADC channel: %s", esp_err_to_name(ret));
        adc_oneshot_del_unit(handle->adc_handle);
        return ESP_ERR_TEMP_ADC_CONFIG;
    }

    // init adc calibration
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = handle->sensor_config.adc_unit,
        .atten = handle->sensor_config.adc_attenuation,
        .bitwidth = handle->sensor_config.adc_bitwidth,
    };
    
    ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle->adc_cali_handle);
    if (ret == ESP_OK) {
        handle->adc_calibrated = true;
        ESP_LOGI(TAG, "ADC calibration initialized");
    } else {
        ESP_LOGW(TAG, "ADC calibration not available: %s", esp_err_to_name(ret));
        handle->adc_calibrated = false;
    }
    
    return ESP_OK;
}

static esp_err_t adc_deinit(temp_sensor_handle_t handle) {
    if (handle->adc_calibrated && handle->adc_cali_handle) {
        adc_cali_delete_scheme_line_fitting(handle->adc_cali_handle);
    }

    if (handle->adc_handle) {
        adc_oneshot_del_unit(handle->adc_handle);
    }

    return ESP_OK;
}

/**
 * @brief reads oneshot value from configured adc channel and updates voltage
 * 
 * @param handle temperature sensor handle
 * @param voltage_mv pointer to int representing voltage on adc
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
static esp_err_t read_adc_voltage(temp_sensor_handle_t handle, int* voltage_mv) {
    int raw_value;

    esp_err_t ret = adc_oneshot_read(handle->adc_handle, handle->sensor_config.adc_channel,
    &raw_value);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read ADC: %s", esp_err_to_name(ret));
        return ret;
    }

    // convert reading to voltage
    if (handle->adc_calibrated) {
        ret = adc_cali_raw_to_voltage(handle->adc_cali_handle, raw_value, voltage_mv);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to convert ADC to voltage: %s", esp_err_to_name(ret));
            // fallback to simple conversion
            *voltage_mv = (raw_value * ADC_MAX_VOLTAGE_MV) / ((1 << handle->sensor_config.adc_bitwidth) - 1);
        }
    } else {
        // simple conversion, no calibration
        *voltage_mv = (raw_value * ADC_MAX_VOLTAGE_MV) / ((1 << handle->sensor_config.adc_bitwidth) - 1);
    }

    ESP_LOGD(TAG, "ADC raw=%d, voltage=%dmV", raw_value, *voltage_mv);
    return ESP_OK;
}

static void update_statistics(temp_sensor_handle_t handle, float temperature) {
    handle->stats.sample_count++;

    // update min/max
    if (temperature < handle->stats.min_temp) {
        handle->stats.min_temp = temperature;
    }
    if (temperature > handle->stats.max_temp) {
        handle->stats.max_temp = temperature;
    }

    // update running average
    if (handle->stats.sample_count == 1) {
        handle->stats.avg_temp = temperature;
    } else {
        handle->stats.avg_temp = ((handle->stats.avg_temp * (handle->stats.sample_count - 1)) + temperature) / handle->stats.sample_count;
    }
}

static float convert_voltage_to_temperature(temp_sensor_handle_t handle, int voltage_mv) {
    float temperature = 0.0f;

    switch (handle->sensor_config.sensor_type) {
        case TEMP_SENSOR_TYPE_LM35:
            temperature = (float)voltage_mv / LM35_MV_PER_DEGREE;
            break;
        // case TEMP_SENSOR_TYPE_NTC:
        //     // temperature = calculate_ntc_temperature(voltage_mv, &handle->sensor_config.sensor_config.ntc);
        //     break;
        default:
            ESP_LOGE(TAG, "Unknown sensor type: %d", handle->sensor_config.sensor_type);
            break;
    }

    return temperature;
}