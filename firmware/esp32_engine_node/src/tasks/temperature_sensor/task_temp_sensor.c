#include "common-defines.h"
#include "temp_sensor.h"

static const char* TAG = "temp_sensor_task";

void example_temp_sensor(void* pvParameter) {
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
        .filter_type = TEMP_FILTER_MOVING_AVG,
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
        temp_sensor_read_filtered(temp_handle, &temperature);
        ESP_LOGI(TAG,"Temperature reading filtered:   %.2f°C", temperature);
        temp_sensor_read(temp_handle, &temperature);
        ESP_LOGI(TAG,"Temperature reading unfiltered: %.2f°C\n", temperature);

        // printf("Temperature reading: %f\n", temperature);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}