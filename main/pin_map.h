// Hardware Pin Configuration and Mapping
// This file defines all GPIO pin assignments for the Chamelamp project

#ifdef CONFIG_DEVICE_MODE_SENDER

// TCS34725 Color Sensor Interface
#define TCS_I2C_MASTER_SCL_IO GPIO_NUM_38 // I2C Serial Clock (SCL) signal
#define TCS_I2C_MASTER_SDA_IO GPIO_NUM_47 // I2C Serial Data (SDA) signal
#define TCS_INT_PIN GPIO_NUM_21          // Color sensor interrupt signal for event detection

// NeoPixel LED Ring Configuration
// Two LED rings are used for visual feedback and lighting effects
#define SMALL_RING_PIN GPIO_NUM_18 // Control signal for inner 12-LED ring
#define SMALL_RING_LED_COUNT 12     // LED count for inner ring
#define BIG_RING_PIN GPIO_NUM_17    // Control signal for outer 24-LED ring
#define BIG_RING_LED_COUNT 24   // LED count for outer ring
    

#else // CONFIG_DEVICE_MODE_RECEIVER

// LCD Display Communication Interface
#define LCD_I2C_MASTER_SCL_IO GPIO_NUM_38 // LCD display I2C clock signal
#define LCD_I2C_MASTER_SDA_IO GPIO_NUM_47 // LCD display I2C data signal

#endif 