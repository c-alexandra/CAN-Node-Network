#include "freertos/FreeRTOS.h" // vTaskDelay
#include "driver/gpio.h" // gpio functions

#include "display.h"

#define NOT_CONNECTED (255)

// Display configuration with GPIO 
display_t display_config = {
    .display_rs = GPIO_NUM_4,
    .display_rw = NOT_CONNECTED,
    .display_e = GPIO_NUM_32,
    .display_data = {
        NOT_CONNECTED, NOT_CONNECTED, NOT_CONNECTED, NOT_CONNECTED, 
        GPIO_NUM_21, GPIO_NUM_22, GPIO_NUM_23, GPIO_NUM_25
    },
    .bit_mode = DISPLAY_BITMODE_4
};

/*******************************************************************************
 * @brief Pulses the enable pin to latch data into the display.
*******************************************************************************/
static void pulse_enable(void) {
    vTaskDelay(1 / portTICK_PERIOD_MS); // wait for a short time
    gpio_set_level(display_config.display_e, 0);
    vTaskDelay(1 / portTICK_PERIOD_MS);
    gpio_set_level(display_config.display_e, 1);
    vTaskDelay(1 / portTICK_PERIOD_MS);
    gpio_set_level(display_config.display_e, 0);
    vTaskDelay(1 / portTICK_PERIOD_MS);
}

/*******************************************************************************
 * @brief Writes 4 bits to the display.
 * @param data The 4 bits to write.
 * 
 * This function assumes the display is in 4-bit mode & is used to set 4-bits 
*******************************************************************************/
static void write_4_bits(uint8_t data) {
    for (int i = 0; i < 4; i++) {
        gpio_set_level(display_config.display_data[i + 4], (data >> i) & 0x01);
    }
    
    pulse_enable();
}

/*******************************************************************************
 * @brief Writes 8 bits to the display.
 * @param data The 8 bits to write.
*******************************************************************************/
static void write_8_bits(uint8_t data) {
    for (int i = 0; i < 8; i++) {
        gpio_set_level(display_config.display_data[i], (data >> i) & 0x01);
    }
    
    pulse_enable();
}

/*******************************************************************************
 * @brief sends data to the display, accounting for bitmode and register select.
 * @param data The data to send.
 * @param reg_select The register select value (0 for command, 1 for data).
*******************************************************************************/
static void send_data(uint8_t data, uint8_t reg_select) {
    gpio_set_level(display_config.display_rs, reg_select);

    if (display_config.display_rw != NOT_CONNECTED) {
        gpio_set_level(display_config.display_rw, 0);
    }
    
    if (display_config.bit_mode == DISPLAY_BITMODE_4) {
        write_4_bits(data >> 4); // send high nibble
        write_4_bits(data & 0x0F); // send low nibble
    } else if (display_config.bit_mode == DISPLAY_BITMODE_8) {
        write_8_bits(data); // send all 8 bits
    }
}

/*******************************************************************************
 * @brief initializes the display with the provided configuration.
 * @param config The display configuration.
*******************************************************************************/
void display_init(display_t config)
{
    // Initialize GPIO pins for display
    gpio_set_direction(config.display_rs, GPIO_MODE_OUTPUT);
    gpio_set_direction(config.display_e, GPIO_MODE_OUTPUT);

    if (config.display_rw != NOT_CONNECTED) {
        gpio_set_direction(config.display_rw, GPIO_MODE_OUTPUT);
    }
    
    if (config.bit_mode == DISPLAY_BITMODE_8) {
        // Set all data pins for 8-bit mode
        for (int i = 0; i < 8; i++) {
            gpio_set_direction(config.display_data[i], GPIO_MODE_OUTPUT);
        }
    } else if (config.bit_mode == DISPLAY_BITMODE_4) {
        // Set only the last 4 data pins for 4-bit mode
        for (int i = 0; i < 4; i++) {
            gpio_set_direction(config.display_data[i+4], GPIO_MODE_OUTPUT);
        }
    }

    // Put LCD into 4-bit or 8-bit mode
    if (config.bit_mode == DISPLAY_BITMODE_4) {
        // start in 8-bit mode, switch with following commands
        write_4_bits(0x03);
        vTaskDelay(1 / portTICK_PERIOD_MS); 
        // second attempt to switch to 4-bit mode
        write_4_bits(0x03);
        vTaskDelay(1 / portTICK_PERIOD_MS);
        // third attempt to switch to 4-bit mode
        write_4_bits(0x03);
        vTaskDelay(1 / portTICK_PERIOD_MS);
        // switch to 4-bit mode
        write_4_bits(0x02);
    } else {

    }

    // Finally, set # lines, font size, etc.
    uint8_t _displayfunction = LCD_2LINE | LCD_5x8DOTS;
    send_data(LCD_FUNCTIONSET | _displayfunction, 0);  

    _displayfunction = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
    send_data(LCD_DISPLAYCONTROL | _displayfunction, 0);

    // clear it off
    display_clear();

    // Initialize to default text direction/ entry mode
    _displayfunction = LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT;
    send_data(LCD_ENTRYMODESET | _displayfunction, 0);
}

/*******************************************************************************
 * @brief sends a string to the display.
 * @param str The string to send.
*******************************************************************************/
void display_print(const char *str) {
    while (*str) {
        send_data(*str++, RS_HIGH); // send each character with RS set high
    }
}

/*******************************************************************************
 * @brief clears the display
*******************************************************************************/
void display_clear(void) {
    send_data(LCD_CLEARDISPLAY, 0); // clear display, set cursor position to zero
    vTaskDelay(2 / portTICK_PERIOD_MS); // this command takes a long time!
}

/*******************************************************************************
 * @brief sets cursor position to home.
*******************************************************************************/
void display_home(void) {
    send_data(LCD_RETURNHOME, 0); // set cursor position to zero
    vTaskDelay(2 / portTICK_PERIOD_MS); // this command takes a long time!
}

/*******************************************************************************
 * @brief example function to demonstrate display usage.
 * @param pvParameter Unused parameter for freertos, can be NULL.
*******************************************************************************/
void display_example(void *pvParameter) {
    display_init(display_config);

    while (1) {
        display_print("Hello, World!");

        vTaskDelay(1000 / portTICK_PERIOD_MS); // wait before next write
        display_clear(); // clear the display
        // display_home(); // return cursor to home position
        vTaskDelay(1000 / portTICK_PERIOD_MS); // wait before next iteration
    }
}