# LCD16x2 Library for esp32

A comprehensive, thread-safe library for controlling 16x2 character LCD displays using LCD ESP32 MCUs.
Supports both 4-bit and 8-bit parallel interface modes.

## Features

- [x] Dual Interface Support: Both 4-bit and 8-bit parallel communication
- [x] Thread-Safe Operation: Utilizes FreeRTOS mutex protection and task handoff where appropriate
- [x] Multiple Instance Support: Easily support multiple LCD instance with opaque handle per instance
- [x] Comprehensive Error Handling: Easily debug issues with labeled error points and ESP-IDF standard error codes
- [] Optional Backlight Support: Provides interface for controlling backlight state
- [] Professional Documentation: Doxygen-compatible comments

## Hardware Requirements

### Minimum Requirements (4-bit)

- ESP32 Development Board
- 16x2 HD4470-compatible LED display
- 6 GPIO pins (RS, Enable, D4 - D7)
- OPTIONAL: (RW, 1 GPIO for backlight)

### Full Requirements (8-bit)

- ESP32 Development Board
- 16x2 HD4470-compatible LED display
- 10 GPIO pins (RS, Enable, D0 - D7)
- OPTIONAL: (RW, 1 GPIO for backlight)

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

### Custom Timing Example

```c
lcd16x2_timing_config_t config = {
    // ... pin configuration ...
    .timing = {
        .enable_pulse_us = 2,       // Slower for noisy environments
        .command_delay_us = 50,     // Extra margin for slow displays
        .clear_delay_ms = 3,        // Conservative clear timing
        .init_delay_ms = 100        // Extended startup delay
    },
};
```

## Timing Optimization

For maximum performance:

- Use minimum timing values compatible with LCD hardware (consult datasheet timing requirements)
- Consider caching display content to minimize unnecessary writes to LCD
- Use 8-bit mode (faster, but requires more IO pins)

## Troubleshooting

### Common Issues

#### 1. Display shows garbage characters

- Check wiring and connections
- Verify power supply (3.3v or 5.0v min)
- Adjust timings. (Particularly found issue with `enable_pulse_us` timing)

#### 2. Display contrast too high / low

- Ensure potentiometer bridging Vss -> Vo -> Vdd

#### 3. No output

- Adjust timings far above minimum suggested, particularly `clear_delay_ms` and `command_delay_us`
