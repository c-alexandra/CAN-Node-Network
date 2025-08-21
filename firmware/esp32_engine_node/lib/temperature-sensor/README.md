# Temperature Sensor Library for esp32

A comprehensive, thread-safe library for controlling LM35 and NTC thermistor type temperature sensors.

## Features

- [x] Dual Interface Support: Both 4-bit and 8-bit parallel communication
- [x] Thread-Safe Operation: Utilizes FreeRTOS mutex protection and task handoff where appropriate
- [x] Multiple Instance Support: Easily support multiple LCD instance with opaque handle per instance
- [x] Comprehensive Error Handling: Easily debug issues with labeled error points and ESP-IDF standard error codes
- [] Optional Backlight Support: Provides interface for controlling backlight state
- [] Professional Documentation: Doxygen-compatible comments

## Hardware Requirements

### Minimum Requirements

- ESP32 Development Board
- LM35 Package or NTC thermistor
- 1 GPIO pin (Vout of sensor)

## Configuration Options

The library uses configurable timing parameters, as each LCD panel may feature different timing requirements.

```c
typedef struct {
    uint32_t enable_pulse_us;       // Enable pulse width (min 1µs)
    uint32_t command_delay_us;      // Delay after commands (min 37µs)
    uint32_t clear_delay_ms;        // Delay after clear command (min 1.52ms)
    uint32_t init_delay_ms;         // Initial power-on delay (min 15ms)
} lcd16x2_timing_t;
```

Use `LCD16X2_DEFAULT_TIMING()` for standard HD44780 displays.


## Timing Optimization


## Troubleshooting

### Common Issues

