/*******************************************************************************
 * @file lcd16x2.c
 * @brief 16x2 character LCD driver for esp32
 * 
 * This implementation provides a high-performance, thread-safe LCD control with
 * optimized GPIO operations and comprehensive error handling.
 * 
 * @author Camille Aitken
 * @version 1.0.0
 * 
 * @copyright MIT License
 ******************************************************************************/
#include "lcd16x2.h"
// #include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include "esp_log.h"
#include "esp_check.h"
// #include "esp_timer.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h" // gpio functions
#include "freertos/task.h"
// #include "freertos/semphr.h" // semaphore_handle_t

// debug defines for code testing
// #define DEBUG_DELAY

// constants dictated by datasheet and borrowed from arduino liquidcrystal
// commands
#define LCD_CLEARDISPLAY        (0x01)
#define LCD_RETURNHOME          (0x02)
#define LCD_ENTRYMODESET        (0x04)
#define LCD_DISPLAYCONTROL      (0x08)
#define LCD_CURSORSHIFT         (0x10)
#define LCD_FUNCTIONSET         (0x20)
#define LCD_SETCGRAMADDR        (0x40)
#define LCD_SETDDRAMADDR        (0x80)

// flags for display entry mode
#define LCD_ENTRYRIGHT          (0x00)
#define LCD_ENTRYLEFT           (0x02)
#define LCD_ENTRYSHIFTINCREMENT (0x01)
#define LCD_ENTRYSHIFTDECREMENT (0x00)

// flags for display on/off control
#define LCD_DISPLAYON           (0x04)
#define LCD_DISPLAYOFF          (0x00)
#define LCD_CURSORON            (0x02)
#define LCD_CURSOROFF           (0x00)
#define LCD_BLINKON             (0x01)
#define LCD_BLINKOFF            (0x00)

// flags for display/cursor shift
#define LCD_DISPLAYMOVE         (0x08)
#define LCD_CURSORMOVE          (0x00)
#define LCD_MOVERIGHT           (0x04)
#define LCD_MOVELEFT            (0x00)

// flags for function set
#define LCD_8BITMODE            (0x10)
#define LCD_4BITMODE            (0x00)
#define LCD_2LINE               (0x08)
#define LCD_1LINE               (0x00)
#define LCD_5x10DOTS            (0x04)
#define LCD_5x8DOTS             (0x00)

// Display dimensions
#define LCD_ROWS                (2)
#define LCD_COLS                (16)
#define LCD_TOTAL_CHARS         (LCD_ROWS * LCD_COLS)

// Row addresses for DDRAM
#define LCD_ROW0_ADDR               0x00
#define LCD_ROW1_ADDR               0x40

static const char *TAG = "lcd16x2"; // defined for log, per .c file

// Validation macros
#define LCD16X2_CHECK(condition, err_code, format, ...) do { \
    if (!(condition)) { \
        ESP_LOGE(TAG, format, ##__VA_ARGS__); \
        return err_code; \
    } \
} while(0)

// TODO: Ensure working as expected,
#define LCD16X2_CHECK_HANDLE(handle) \
    LCD16X2_CHECK(handle != NULL, ESP_ERR_INVALID_ARG, "Handle is NULL"); \
    LCD16X2_CHECK(handle->initialized, ESP_ERR_LCD_NOT_INITIALIZED, "LCD not initialized")

typedef struct lcd16x2_s {
    // Hardware configuration
    lcd16x2_mode_t mode;        // Interface mode (4-bit or 8-bit) */
    gpio_num_t rs_pin;          // Register Select pin */
    gpio_num_t rw_pin;           // read/write pin
    gpio_num_t enable_pin;      // Enable pin */
    gpio_num_t data_pins[8];    // Data pins (4 or 8 used depending on mode) */
    gpio_num_t backlight_pin;   // Backlight control pin */
    bool backlight_enabled;     // Backlight control available */
    
    // Timing configuration
    lcd16x2_timing_t timing;    // Timing parameters */
    
    // State management
    SemaphoreHandle_t mutex;    // Thread safety mutex */
    bool initialized;           // Initialization status */
    bool display_on;            // Display on/off state */
    bool cursor_on;             // Cursor visibility state */
    bool blink_on;              // Cursor blink state */
    bool backlight_state;       // Backlight on/off state */
    
    // Current position tracking
    uint8_t current_row;        // Current cursor row */
    uint8_t current_col;        // Current cursor column */
    
    // Display buffer (for tracking content)
    char display_buffer[LCD_TOTAL_CHARS]; /**< Mirror of display content */
} lcd16x2_t;

/*******************************************************************************
 * PRIVATE FUNCTION PROTOTYPES
 ******************************************************************************/
static esp_err_t lcd_gpio_init(lcd16x2_handle_t handle);
static esp_err_t lcd_gpio_deinit(lcd16x2_handle_t handle);
static esp_err_t lcd_hardware_init(lcd16x2_handle_t handle);
static esp_err_t lcd_update_display_control(lcd16x2_handle_t handle);

static esp_err_t lcd_write_command(lcd16x2_handle_t handle, uint8_t command);
static esp_err_t lcd_write_data(lcd16x2_handle_t handle, uint8_t data);
static esp_err_t lcd_write_4bit(lcd16x2_handle_t handle, uint8_t data);
static esp_err_t lcd_write_8bit(lcd16x2_handle_t handle, uint8_t data);
static void lcd_pulse_enable(lcd16x2_handle_t handle);
static void lcd_gpio_set_level_fast(gpio_num_t pin, int level);

static void lcd_delay_us(uint32_t us);
static void lcd_delay_ms(uint32_t ms);
static esp_err_t lcd_validate_position(uint8_t row, uint8_t col);

/*******************************************************************************
 * PUBLIC FUNCTION IMPLEMENTATIONS
 ******************************************************************************/

/** 
 * @brief Initialize the LCD in either 4-bit or 8-bit mode based on configuration.
 * 
 * @param config Pointer to lcd16x2_config_t structure with pin and timing configuration.
 * @param handle Pointer to lcd16x2_handle_t variable to receive the LCD handle.
 * @return esp_err_t ESP_OK on success, error code otherwise.
 */
esp_err_t lcd16x2_init(const lcd16x2_config_t *config, lcd16x2_handle_t *handle) {
    // Check if config and handle pointers are valid
    LCD16X2_CHECK(config != NULL, ESP_ERR_INVALID_ARG, "Config is NULL");
    LCD16X2_CHECK(handle != NULL, ESP_ERR_INVALID_ARG, "Handle pointer is NULL");

    // Validate GPIO pins
    gpio_num_t pins[] = {
        config->rs_pin, 
        config->rw_pin, 
        config->enable_pin, 
        config->backlight_pin,
        config->data_pins[0],
        config->data_pins[1],
        config->data_pins[2],
        config->data_pins[3],
        config->data_pins[4],
        config->data_pins[5],
        config->data_pins[6],
        config->data_pins[7],
    };
    ESP_RETURN_ON_ERROR(lcd16x2_validate_pins(pins, sizeof(pins) / sizeof(gpio_num_t)), TAG, "Invalid GPIO configuration");

    // Allocate memory for the LCD handle (Handled by FreeRTOS)
    lcd16x2_t *lcd = calloc(1, sizeof(lcd16x2_t));
    LCD16X2_CHECK(lcd != NULL, ESP_ERR_NO_MEM, "Failed to allocate memory for LCD handle");

    // Initialize the handle structure
    lcd->mode = config->bit_mode;
    lcd->rs_pin = config->rs_pin;
    lcd->rw_pin = config->rw_pin;
    lcd->enable_pin = config->enable_pin;
    lcd->backlight_enabled = config->backlight_enable;
    lcd->backlight_pin = config->backlight_pin;
    lcd->timing = config->timing;
    for (int i = 0; i < 8; i++) {
        lcd->data_pins[i] = config->data_pins[i];
    }
    lcd->initialized = false;
    lcd->display_on = true;
    lcd->cursor_on = false;
    lcd->blink_on = false;
    lcd->backlight_state = true; // Default backlight on
    lcd->current_row = 0;
    lcd->current_col = 0;

    memset(lcd->display_buffer, ' ', LCD_TOTAL_CHARS); // Clear display buffer

    // Create mutex for thread safety
    lcd->mutex = xSemaphoreCreateMutex();
    if (lcd->mutex == NULL) {
        free(lcd);
        return ESP_ERR_NO_MEM;
    }

    // Initialize GPIO
    esp_err_t ret = lcd_gpio_init(lcd);
    if (ret != ESP_OK) {
        vSemaphoreDelete(lcd->mutex);
        free(lcd);
        return ret;
    }

    // Initialize LCD hardware
    ret = lcd_hardware_init(lcd);
    if (ret != ESP_OK) {
        lcd_gpio_deinit(lcd);
        vSemaphoreDelete(lcd->mutex);
        free(lcd);
        return ret;
    }

    lcd->initialized = true;
    *handle = lcd;

    ESP_LOGI(TAG, "LCD initialized in %s mode", (lcd->mode == LCD16X2_BITMODE_4) ? "4-bit" : "8-bit");
    return ESP_OK;
}

/** 
 * @brief Validate GPIO pins for LCD configuration.
 * 
 * Ensures pins are valid output GPIOs and checks for duplicates.
 * Valid GPIO are determined by GPIO_IS_VALID_OUTPUT_GPIO macro and includes
 * GPIO_NUM_NC for not connected pins.
 * 
 * @param pins Array of gpio_num_t pins to validate.
 * @param count Number of pins in the array.
 * @return esp_err_t ESP_OK if all pins are valid, error code otherwise.
 */
esp_err_t lcd16x2_validate_pins(const gpio_num_t *pins, size_t count) {
    LCD16X2_CHECK(pins != NULL, ESP_ERR_INVALID_ARG, "Pin array is NULL");
    
    // Validate each pin
    for (size_t i = 0; i < count; i++) {
        if (!GPIO_IS_VALID_OUTPUT_GPIO(pins[i]) && !(pins[i] == GPIO_NUM_NC)) {
            ESP_LOGE(TAG, "Invalid GPIO pin: %d", pins[i]);
            return ESP_ERR_INVALID_ARG;
        }
        
        // Check for duplicate pins
        for (size_t j = i + 1; j < count; j++) {
            if (!(pins[i] == GPIO_NUM_NC) && pins[i] == pins[j]) {
                ESP_LOGE(TAG, "Duplicate GPIO pin: %d", pins[i]);
                return ESP_ERR_INVALID_ARG;
            }
        }
    }
    
    return ESP_OK;
}

/**
 * @brief Write a single character to the LCD display.
 * 
 * This function writes a character to the current cursor position on the LCD.
 * It updates the internal display buffer and cursor position accordingly.
 * 
 * @param handle LCD handle.
 * @param character Character to write.
 * @return esp_err_t ESP_OK on success, error code otherwise.
 */
esp_err_t lcd16x2_write_char(lcd16x2_handle_t handle, char character) {
    LCD16X2_CHECK_HANDLE(handle); // verifyy handle is valid and initialized

    if (xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t ret = lcd_write_data(handle, (uint8_t)(character));
    if (ret == ESP_OK) {
        // Update display buffer and cursor position
        // TODO: Update with bit manipulation and modulo to handle wrap-around
        if (handle->current_col < LCD_COLS && handle->current_row < LCD_ROWS) {
            handle->display_buffer[handle->current_row * LCD_COLS + handle->current_col] = character;
            handle->current_col++;
            if (handle->current_col >= LCD_COLS) {
                handle->current_col = 0;
                handle->current_row = (handle->current_row + 1) % LCD_ROWS; // Wrap to next row
            }
        }
    }

    xSemaphoreGive(handle->mutex);
    return ret;
}

/**
 * @brief Write a string to the LCD display.
 * 
 * @param handle LCD handle.
 * @param str Null-terminated string to write.
 * @return esp_err_t ESP_OK on success, error code otherwise.
 */
esp_err_t lcd16x2_write_string(lcd16x2_handle_t handle, const char *str) {
    LCD16X2_CHECK_HANDLE(handle);
    LCD16X2_CHECK(str != NULL, ESP_ERR_INVALID_ARG, "String is NULL");

    // iterate until end of string, validating each character
    esp_err_t ret = ESP_OK;
    for (const char *p = str; *p != '\0' && ret == ESP_OK; p++) {
        ret = lcd16x2_write_char(handle, *p);
    }

    return ret;
}

/**
 * @brief Write a string to the LCD at a specific row and column.
 * 
 * This function sets the cursor to the specified position and writes the string.
 * It handles cursor positioning and ensures the string fits within the display bounds.
 * 
 * @param handle LCD handle.
 * @param row Row index (0 or 1).
 * @param col Column index (0 to 15).
 * @param str String to write.
 * @return esp_err_t ESP_OK on success, error code otherwise.
 */
esp_err_t lcd16x2_write_string_at(lcd16x2_handle_t handle, uint8_t row, 
                                  uint8_t col, const char *str) {
    esp_err_t ret = lcd16x2_set_cursor(handle, row, col);
    if (ret != ESP_OK) {
        return ret;
    }
    
    return lcd16x2_write_string(handle, str);
}

/** 
 * @brief Set the cursor position on the LCD.
 * 
 * @param handle LCD handle.
 * @param row Row position (0-1).
 * @param col Column position (0-15).
 * @return esp_err_t ESP_OK on success, error code otherwise.
 */
esp_err_t lcd16x2_set_cursor(lcd16x2_handle_t handle, uint8_t row, uint8_t col) {
    LCD16X2_CHECK_HANDLE(handle);
    ESP_RETURN_ON_ERROR(lcd_validate_position(row, col), TAG, "Invalid cursor position");

    if (xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    uint8_t cursor_address = (row == 0) ? LCD_ROW0_ADDR : LCD_ROW1_ADDR;
    cursor_address += col;

    esp_err_t ret = lcd_write_command(handle, LCD_SETDDRAMADDR | cursor_address);
    if (ret == ESP_OK) {
        handle->current_row = row;
        handle->current_col = col;
    }

    xSemaphoreGive(handle->mutex);
    return ret;
}

/**
 * @brief Clear the LCD display and reset cursor position.
 * 
 * This function clears the display, resets the cursor to the home position,
 * and updates the internal display buffer.
 * 
 * @param handle LCD handle.
 * @return esp_err_t ESP_OK on success, error code otherwise.
 */
esp_err_t lcd16x2_clear(lcd16x2_handle_t handle) {
    LCD16X2_CHECK_HANDLE(handle);

    if (xSemaphoreTake(handle->mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t ret = lcd_write_command(handle, LCD_CLEARDISPLAY);
    if (ret == ESP_OK) {
        lcd_delay_ms(handle->timing.clear_delay_ms); // clear takes a long time

        // Reset cursor position
        handle->current_row = 0;
        handle->current_col = 0;
        memset(handle->display_buffer, ' ', LCD_TOTAL_CHARS); // Clear display buffer
    }

    xSemaphoreGive(handle->mutex);
    return ret;
}

/*******************************************************************************
 * PRIVATE FUNCTION IMPLEMENTATIONS
 ******************************************************************************/

/**
 * @brief Initialize GPIO pins for LCD operation.
 * 
 * @param handle LCD handle.
 * @return esp_err_t ESP_OK on success, error code otherwise.
 */
esp_err_t lcd_gpio_init(lcd16x2_handle_t handle) {
    // Configure RS, RW, Enable pins
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << handle->rs_pin) | (1ULL << handle->enable_pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_RETURN_ON_ERROR(gpio_config(&io_conf), TAG, "Failed to configure control pins");

    // Configure RW pin if not NC
    if (handle->rw_pin != GPIO_NUM_NC) {
        io_conf.pin_bit_mask |= (1ULL << handle->rw_pin);
        ESP_RETURN_ON_ERROR(gpio_config(&io_conf), TAG, "Failed to configure RW pin");
    }

    // Configure data pins
    uint64_t data_pin_mask = 0;
    for (int i = 0; i < 8; i++) {
        if (handle->data_pins[i] != GPIO_NUM_NC) {
            data_pin_mask |= (1ULL << handle->data_pins[i]);
        }
    }
    io_conf.pin_bit_mask = data_pin_mask;
    ESP_RETURN_ON_ERROR(gpio_config(&io_conf), TAG, "Failed to configure data pins");

    // Configure backlight pin if enabled
    if (handle->backlight_enabled && handle->backlight_pin != GPIO_NUM_NC) {
        io_conf.pin_bit_mask = (1ULL << handle->backlight_pin);
        ESP_RETURN_ON_ERROR(gpio_config(&io_conf), TAG, "Failed to configure backlight pin");
        // Set initial backlight state
        gpio_set_level(handle->backlight_pin, handle->backlight_state ? 1 : 0);
    }

    // Initialize all pins LOW
    gpio_set_level(handle->rs_pin, 0);
    if (handle->rw_pin != GPIO_NUM_NC) {
        gpio_set_level(handle->rw_pin, 0);
    }
    gpio_set_level(handle->enable_pin, 0);

    for (int i = 0; i < 8; i++) {
        if (handle->data_pins[i] != GPIO_NUM_NC) {
            gpio_set_level(handle->data_pins[i], 0);
        }
    }

    return ESP_OK;
}

/**
 * @brief Reset GPIO pins used by the LCD to default known state.
 * 
 * @param handle LCD handle.
 * @return esp_err_t ESP_OK on success, error code otherwise.
 */
esp_err_t lcd_gpio_deinit(lcd16x2_handle_t handle) {
    // Reset RS, RW, Enable pins
    gpio_reset_pin(handle->rs_pin);
    if (handle->rw_pin != GPIO_NUM_NC) {
        gpio_reset_pin(handle->rw_pin);
    }
    gpio_reset_pin(handle->enable_pin);

    // Reset data pins
    for (int i = 0; i < 8; i++) {
        if (handle->data_pins[i] != GPIO_NUM_NC) {
            gpio_reset_pin(handle->data_pins[i]);
        }
    }

    // Reset backlight pin if enabled
    if (handle->backlight_enabled && handle->backlight_pin != GPIO_NUM_NC) {
        gpio_reset_pin(handle->backlight_pin);
    }

    return ESP_OK;
}

/**
 * @brief Initialize the LCD hardware following HD44780 initialization sequence.
 * 
 * @param handle LCD handle.
 * @return esp_err_t ESP_OK on success, error code otherwise.
 */
esp_err_t lcd_hardware_init(lcd16x2_handle_t handle) {
    // Power-on delay determined by datasheet. MCU will be ready before lcd 
    // without this.
    lcd_delay_ms(handle->timing.init_delay_ms);

    // if 4-bit mode chosen, send command to set 4-bit mode in hardware
    if (handle->mode == LCD16X2_BITMODE_4) {
        // 4-bit mode initialization sequence. Sent three times to ensure LCD
        // is in 4-bit mode. As suggested by Arduino LiquidCrystal library.
        ESP_RETURN_ON_ERROR(lcd_write_4bit(handle, 0x03), TAG, "Failed to send first 4-bit init command");
        lcd_delay_ms(5); // wait >4.1ms
        ESP_RETURN_ON_ERROR(lcd_write_4bit(handle, 0x03), TAG, "Failed to send second 4-bit init command");
        lcd_delay_us(150); // wait >100us
        ESP_RETURN_ON_ERROR(lcd_write_4bit(handle, 0x03), TAG, "Failed to send third 4-bit init command");
        lcd_delay_us(150);

        ESP_RETURN_ON_ERROR(lcd_write_4bit(handle, 0x02), TAG, "Failed to set 4-bit mode");

        // Function set: 4-bit, 2 line, 5x8 dots
        ESP_RETURN_ON_ERROR(lcd_write_command(handle, LCD_FUNCTIONSET | LCD_4BITMODE | LCD_2LINE | LCD_5x8DOTS), TAG, "Failed to set function");
    } else {
        // 8-bit mode initialization sequence
        lcd_delay_ms(15);

        // Function set: 8-bit, 2 line, 5x8 dots
        ESP_RETURN_ON_ERROR(lcd_write_command(handle, LCD_FUNCTIONSET | LCD_8BITMODE | LCD_2LINE | LCD_5x8DOTS), TAG, "Failed to set function");
        lcd_delay_ms(5); // wait >4.1ms
        ESP_RETURN_ON_ERROR(lcd_write_command(handle, LCD_FUNCTIONSET | LCD_8BITMODE | LCD_2LINE | LCD_5x8DOTS), TAG, "Failed to set function");
        lcd_delay_us(150); // wait >100us

        ESP_RETURN_ON_ERROR(lcd_write_command(handle, LCD_FUNCTIONSET | LCD_8BITMODE | LCD_2LINE | LCD_5x8DOTS), TAG, "Failed to set function");
        lcd_delay_us(150);
    }

    // Display OFF
    ESP_RETURN_ON_ERROR(lcd_write_command(handle, LCD_DISPLAYCONTROL | LCD_DISPLAYOFF), TAG, "Failed to turn off display");

    ESP_RETURN_ON_ERROR(lcd_write_command(handle, LCD_CLEARDISPLAY), TAG, "Failed to clear display"); // Clear display
    lcd_delay_ms(handle->timing.clear_delay_ms); // Wait for clear to complete

    // Entry mode set: increment cursor, no shift
    ESP_RETURN_ON_ERROR(lcd_write_command(handle, LCD_ENTRYMODESET | LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT), TAG, "Failed to set entry mode");

    // Display ON, update blink and cursor state
    ESP_RETURN_ON_ERROR(lcd_update_display_control(handle), TAG, "Failed to update display control");

    return ESP_OK;
}

/**
 * @brief Write a command to the LCD.
 * 
 * @param handle LCD handle.
 * @param command Command byte to send.
 * @return esp_err_t ESP_OK on success, error code otherwise.
 */
esp_err_t lcd_write_command(lcd16x2_handle_t handle, uint8_t command) {
    // Set RS and RW pins for command mode
    gpio_set_level(handle->rs_pin, 0); // RS = 0 for command
    if (handle->rw_pin != GPIO_NUM_NC){
        gpio_set_level(handle->rw_pin, 0); // RW = 0 for write
    }   

    // Write command based on mode
    if (handle->mode == LCD16X2_BITMODE_4) {
        ESP_RETURN_ON_ERROR(lcd_write_4bit(handle, command >> 4), TAG, "Failed to write high nibble");
        ESP_RETURN_ON_ERROR(lcd_write_4bit(handle, command & 0x0F), TAG, "Failed to write low nibble");
    } else {
        ESP_RETURN_ON_ERROR(lcd_write_8bit(handle, command), TAG, "Failed to write 8-bit command");
    }

    lcd_delay_us(handle->timing.command_delay_us); // Delay for command processing

    return ESP_OK;
}

/**
 * @brief Write data to the LCD.
 * 
 * @param handle LCD handle.
 * @param data Data byte to send.
 * @return esp_err_t ESP_OK on success, error code otherwise.
 */
static esp_err_t lcd_write_data(lcd16x2_handle_t handle, uint8_t data) {
    gpio_set_level(handle->rs_pin, 1); // RS = 1 for data
    if (handle->rw_pin != GPIO_NUM_NC) {
        gpio_set_level(handle->rw_pin, 0); // RW = 0 for write
    }

    // Write data based on mode
    if (handle->mode == LCD16X2_BITMODE_4) {
        ESP_RETURN_ON_ERROR(lcd_write_4bit(handle, data >> 4), TAG, "Failed to write high nibble");
        ESP_RETURN_ON_ERROR(lcd_write_4bit(handle, data & 0x0F), TAG, "Failed to write low nibble");
    } else {
        ESP_RETURN_ON_ERROR(lcd_write_8bit(handle, data), TAG, "Failed to write 8-bit data");
    }

    return ESP_OK;
}

// TODO: Test and validate this function works as expected, them implement in 
// place of gpio_set_level in other functions
/**
 * @brief Set GPIO pin level quickly using direct register manipulation.
 * 
 * This function sets the GPIO pin level using direct register access for
 * improved performance over the standard gpio_set_level function.
 * 
 * @param pin GPIO pin number to set.
 * @param level Level to set (0 or 1).
 */
static void lcd_gpio_set_level_fast(gpio_num_t pin, int level) {
    if (level) {
        if (pin < 32) {
            GPIO.out_w1ts = (1 << pin); // Set pin high
        } else {
            GPIO.out1_w1ts.val = (1U << (pin - 32)); // Set pin high for pins > 31
        }
        GPIO.out_w1ts = (1 << pin); // Set pin high
    } else {
        if (pin < 32) {
            GPIO.out_w1tc = (1 << pin); // Set pin low
        } else {
            GPIO.out1_w1tc.val = (1U << (pin - 32)); // Set pin low for pins > 31
        }
    }
}

/**
 * @brief Pulse the Enable pin with fast delay.
 * 
 * @param handle LCD handle.
 */
static void lcd_pulse_enable(lcd16x2_handle_t handle) {
#ifndef DEBUG_DELAY // TODO: Remove
    lcd_gpio_set_level_fast(handle->enable_pin, 1);
    lcd_delay_us(handle->timing.enable_pulse_us);
    lcd_gpio_set_level_fast(handle->enable_pin, 0);
#else
    // TODO: remove - having a lot of problems with enable pulse width 
    // vTaskDelay(pdMS_TO_TICKS(1)); 
    // gpio_set_level(handle->enable_pin, 0);
    // lcd_delay_us(handle->timing.enable_pulse_us);
    gpio_set_level(handle->enable_pin, 1); 
    lcd_delay_us(handle->timing.enable_pulse_us);
    gpio_set_level(handle->enable_pin, 0); 
    // vTaskDelay(pdMS_TO_TICKS(1));
#endif
}

/**
 * @brief Write a 4-bit command to the LCD.
 * 
 * @param handle LCD handle.
 * @param data 4-bit data to send.
 * @return esp_err_t ESP_OK on success, error code otherwise.
 */
static esp_err_t lcd_write_4bit(lcd16x2_handle_t handle, uint8_t data) {
    // set data pins
    for (int i = 0; i < 4; i++) {
    lcd_gpio_set_level_fast(handle->data_pins[4 + i], (data >> i) & 0x01);    }

    lcd_pulse_enable(handle); // Pulse the enable pin to latch data
    return ESP_OK;
}

/**
 * @brief Write an 8-bit command to the LCD.
 * 
 * @param handle LCD handle.
 * @param data 8-bit data to send.
 * @return esp_err_t ESP_OK on success, error code otherwise.
 */
static esp_err_t lcd_write_8bit(lcd16x2_handle_t handle, uint8_t data) {
    // set data pins
    for (int i = 0; i < 8; i++) {
        gpio_set_level(handle->data_pins[i], (data >> i) & 0x01);
    }   

    lcd_pulse_enable(handle); // Pulse the enable pin to latch data
    return ESP_OK;
}

/** 
 * @brief Updates the display control for cursor, display, and blink states.
 * 
 * @param handle LCD handle.
 * @return esp_err_t ESP_OK on success, error code otherwise.
 */
static esp_err_t lcd_update_display_control(lcd16x2_handle_t handle) {
    uint8_t control = LCD_DISPLAYCONTROL;
    
    if (handle->display_on) {
        control |= LCD_DISPLAYON;
    }
    if (handle->cursor_on) {
        control |= LCD_CURSORON;
    }
    if (handle->blink_on) {
        control |= LCD_BLINKON;
    }
    
    return lcd_write_command(handle, control);
}

/**
 * @brief Delay function for microseconds using ESP-IDF's delay function.
 * 
 * FreeRTOS vtaskdelay is not suitable for microsecond delays, as task delays
 * are ordered by tick precision (typically 1ms).
 * 
 * @param us Number of microseconds to delay.
 */
static void lcd_delay_us(uint32_t us) {
    if (us > 0) {
        esp_rom_delay_us(us); // Use ESP-IDF's delay function for microsecond precision
    }
}

/**
 * @brief Delay function for milliseconds using FreeRTOS's delay function.
 * 
 * @param ms Number of milliseconds to delay.
 */
static void lcd_delay_ms(uint32_t ms) {
    if (ms > 0) {
        vTaskDelay(pdMS_TO_TICKS(ms)); // Use FreeRTOS delay for millisecond precision
    }
}

/**
 * @brief Validate cursor position.
 * 
 * @param row Row index (0 or 1).
 * @param col Column index (0 to 15).
 * @return esp_err_t ESP_OK if position is valid, error code otherwise.
 */
static esp_err_t lcd_validate_position(uint8_t row, uint8_t col) {
    LCD16X2_CHECK(row < LCD_ROWS, ESP_ERR_LCD_INVALID_POSITION, 
                  "Row %d out of range (0-%d)", row, LCD_ROWS - 1);
    LCD16X2_CHECK(col < LCD_COLS, ESP_ERR_LCD_INVALID_POSITION, 
                  "Column %d out of range (0-%d)", col, LCD_COLS - 1);
    return ESP_OK;
}

