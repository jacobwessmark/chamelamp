# Chamelamp

## Overview

Chamelamp is a color-matching lamp system that consists of a sender and receiver unit. The sender unit features a high-precision TCS34725 color sensor surrounded by two NeoPixel LED rings, allowing it to both detect and reproduce colors from its environment. When the sensor detects a color from any object or surface, the LED rings instantly illuminate to match that color, creating a dynamic and interactive lighting experience.

The system uses ESP-NOW wireless communication to enable a receiver unit to mirror the same colors remotely. This creates possibilities for synchronized ambient lighting across different locations or rooms. The sender unit is battery-powered with 4 AA batteries, making it portable and easy to use anywhere.

Key applications include:
- Ambient lighting that adapts to your environment
- Color matching for design and artistic purposes
- Interactive lighting for educational settings
- Mood lighting that can be controlled remotely
- Color exploration and learning tool

## Features

- **Color Detection**: Utilizes the TCS34725 sensor to detect RGB values and map them to predefined color names.
- **LED Control**: Controls NeoPixel LED rings to display colors based on sensor readings.
- **Wireless Communication**: Uses ESP-NOW for sending color data between devices.
- **Deep Sleep Mode**: The system can enter a low-power deep sleep mode and wake up based on external signals or timers.

## Components

- **TCS34725 Color Sensor**: Detects RGB values and clear light intensity.
- **ESP32 Microcontroller**: Handles sensor data processing, LED control, and wireless communication.
- **NeoPixel LED Rings**: 
  - Upper Ring: 12-LED NeoPixel ring
  - Lower Ring: 24-LED NeoPixel ring
- **LCD Display** (Receiver mode): I2C LCD display for visual feedback
- **ESP-NOW**: Provides wireless communication capabilities.
- **Power Supply**: 4x AA batteries (1.5V each) providing 6V total
- **Power Components**: 100µF capacitor for power rail stabilization

## Hardware Connections

### Sender Mode

#### TCS34725 Color Sensor
| TCS34725 Pin | ESP32 Pin | Description   |
|--------------|-----------|---------------|
| VIN          | 3.3V      | Power supply  |
| GND          | GND       | Ground        |
| SCL          | GPIO38    | I2C Clock     |
| SDA          | GPIO47    | I2C Data      |
| INT          | GPIO21    | Interrupt pin |
| LED          | GND       | LED Control   |

#### NeoPixel LED Rings
| Component   | Pin | ESP32 Pin | Description          |
|-------------|-----|-----------|----------------------|
| Upper Ring  | VIN | Vcc       | Battery power supply |
| Upper Ring  | GND | GND       | Ground               |
| Upper Ring  | DIN | GPIO18    | Data Input (12 LEDs) |
| Lower Ring  | VIN | Vcc       | Battery power supply |
| Lower Ring  | GND | GND       | Ground               |
| Lower Ring  | DIN | GPIO17    | Data Input (24 LEDs) |

### Receiver Mode

#### LCD Display
| LCD Pin | ESP32 Pin | Description   |
|---------|-----------|---------------|
| VCC     | 3.3V      | Power supply  |
| GND     | GND       | Ground        |
| SCL     | GPIO38    | I2C Clock     |
| SDA     | GPIO47    | I2C Data      |

### Power System
- **Battery Pack**: 4x 1.5V AA batteries in series (6V output)
- **Filtering**: 100µF electrolytic capacitor across power rails for stability
- **Input Protection**: Reverse polarity protection diode
- **Voltage Range**: System operates from 4.5V to 6V (accounting for battery discharge)


## Upcoming Features

### Connection and Communication
- OTA firmware updates for both sender and receiver units
- Version control and rollback capabilities
- Real-time UDP logging for debugging

### DMX Integration
- DMX512 protocol support for receiver unit
- Conversion of color data to DMX signals
- Compatible with professional stage lighting equipment

### Power Management
- Battery level monitoring and display
- Low battery warnings on both LCD and LED indicators
- USB-C charging capability (planned hardware revision)
- Charging status indication


These features are currently in development and will be released in future updates. Priority and implementation timeline may vary based on user feedback and requirements.


## License

This project is licensed under the MIT License. See the LICENSE file for details.

## Acknowledgments

- Zorxx Software for the NeoPixel driver.
- ESP-IDF for the development framework.
