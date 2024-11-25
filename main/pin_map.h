
// Pin definitions

// TCS34725
#define TCS_I2C_MASTER_SCL_IO GPIO_NUM_9
#define TCS_I2C_MASTER_SDA_IO GPIO_NUM_8
#define TCS_INT_PIN GPIO_NUM_18

// NeoPixel rings
#define SMALL_RING_PIN 10
#define SMALL_RING_LED_COUNT 12
#define BIG_RING_PIN 12
#define BIG_RING_LED_COUNT 24

// LCD
#define LCD_I2C_MASTER_SCL_IO GPIO_NUM_38 // Defines the GPIO pin for the I2C clock line (SCL)
#define LCD_I2C_MASTER_SDA_IO GPIO_NUM_47 // Defines the GPIO pin for the I2C data line (SDA)