# TCS34725 Color Sensor Project

## Overview

This project is designed to interface with the TCS34725 color sensor using an ESP32 microcontroller. The system is capable of detecting colors and controlling NeoPixel LED rings based on the detected colors. It also utilizes ESP-NOW for wireless communication between devices.

## Features

- **Color Detection**: Utilizes the TCS34725 sensor to detect RGB values and map them to predefined color names.
- **LED Control**: Controls NeoPixel LED rings to display colors based on sensor readings.
- **Wireless Communication**: Uses ESP-NOW for sending color data between devices.
- **Deep Sleep Mode**: The system can enter a low-power deep sleep mode and wake up based on external signals or timers.

## Components

- **TCS34725 Color Sensor**: Detects RGB values and clear light intensity.
- **ESP32 Microcontroller**: Handles sensor data processing, LED control, and wireless communication.
- **NeoPixel LED Rings**: Displays colors based on sensor readings.
- **ESP-NOW**: Provides wireless communication capabilities.

## Code Structure

### Main Application

- **`main/main.c`**: Contains the main application logic, including initialization and wake-up handling.
  ```c:main/main.c
  startLine: 1
  endLine: 110
  ```

### Color Sensor

- **`components/TCS34725_colorsensor/TCS34725_colorsensor.c`**: Implements functions for interfacing with the TCS34725 sensor, including data acquisition and threshold management.
  ```c:components/TCS34725_colorsensor/TCS34725_colorsensor.c
  startLine: 1
  endLine: 405
  ```

- **`components/TCS34725_colorsensor/include/TCS34725_colorsensor_private.h`**: Contains private definitions and configurations for the TCS34725 sensor.
  ```c:components/TCS34725_colorsensor/include/TCS34725_colorsensor_private.h
  startLine: 1
  endLine: 136
  ```

- **`components/TCS34725_colorsensor/include/TCS34725_helper_functions.h`**: Declares helper functions for sensor operations.
  ```c:components/TCS34725_colorsensor/include/TCS34725_helper_functions.h
  startLine: 1
  endLine: 46
  ```

- **`components/TCS34725_colorsensor/include/TCS34725_colorsensor.h`**: Provides public API functions for the TCS34725 sensor.
  ```c:components/TCS34725_colorsensor/include/TCS34725_colorsensor.h
  startLine: 1
  endLine: 19
  ```

### NeoPixel LED Control

- **`managed_components/zorxx__neopixel/neopixel.c`**: Manages NeoPixel LED operations, including initialization and pixel setting.
  ```c:managed_components/zorxx__neopixel/neopixel.c
  startLine: 1
  endLine: 226
  ```

### ESP-NOW Communication

- **`components/esp_now_handler/include/esp_now_handler.h`**: Defines functions for initializing ESP-NOW and managing data queues.
  ```c:components/esp_now_handler/include/esp_now_handler.h
  startLine: 1
  endLine: 34
  ```

## Setup and Usage

1. **Hardware Connections**: Connect the TCS34725 sensor and NeoPixel rings to the ESP32 as per the pin definitions in `main/pin_map.h`.
   ```c:main/pin_map.h
   startLine: 1
   endLine: 17
   ```

2. **Build and Flash**: Use the ESP-IDF toolchain to build and flash the firmware onto the ESP32.

3. **Operation**: The device will initialize and start detecting colors. It will control the NeoPixel rings and communicate color data via ESP-NOW.

4. **Deep Sleep**: The system will enter deep sleep mode based on certain conditions and can be woken up by external signals or timers.

## License

This project is licensed under the MIT License. See the LICENSE file for details.

## Acknowledgments

- Zorxx Software for the NeoPixel driver.
- ESP-IDF for the development framework.