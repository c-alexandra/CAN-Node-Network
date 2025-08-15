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
