#include "lcd1602.h"
#include "lcd1602_private.h"

static const char *TAG = "LCD"; // Tag used in logging to identify messages from this program

// Initializes the I2C master interface for communication with the LCD
void i2c_master_init(gpio_num_t scl_pin, gpio_num_t sda_pin)
{
    ESP_LOGI(TAG, "Initializing I2C master...");
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,             // Sets the I2C mode to master
        .sda_io_num = sda_pin,               // Configures the SDA pin for I2C
        .scl_io_num = scl_pin,               // Configures the SCL pin for I2C
        .sda_pullup_en = GPIO_PULLUP_ENABLE, // Enables pull-up resistor for SDA
        .scl_pullup_en = GPIO_PULLUP_ENABLE, // Enables pull-up resistor for SCL
        .master.clk_speed = I2C_LCD_FREQ_HZ, // Sets the clock frequency
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);                // Configures I2C with the above settings
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0); // Installs the I2C driver
    ESP_LOGI(TAG, "I2C master initialized.");


}

// Sends a command to the LCD to configure or control it
void lcd_send_command(uint8_t command)
{
    ESP_LOGI(TAG, "Sending LCD command: 0x%02X", command);

    i2c_cmd_handle_t cmd = i2c_cmd_link_create(); // Creates an I2C command link
    i2c_master_start(cmd);                        // Starts the I2C communication

    // Sends the I2C address with the write mode flag
    i2c_master_write_byte(cmd, (LCD_ADDR << 1) | I2C_MASTER_WRITE, true);

    // Sends the upper nibble (high 4 bits) with enable bit set
    i2c_master_write_byte(cmd, (command & 0xF0) | LCD_BACKLIGHT | 0x04, true); // RS=0, E=1, Backlight=1
    i2c_master_write_byte(cmd, (command & 0xF0) | LCD_BACKLIGHT, true);        // RS=0, E=0, Backlight=1 (latch)

    // Sends the lower nibble (low 4 bits) with enable bit set
    i2c_master_write_byte(cmd, ((command << 4) & 0xF0) | LCD_BACKLIGHT | 0x04, true); // RS=0, E=1, Backlight=1
    i2c_master_write_byte(cmd, ((command << 4) & 0xF0) | LCD_BACKLIGHT, true);        // RS=0, E=0, Backlight=1 (latch)

    i2c_master_stop(cmd); // Ends the I2C communication

    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000)); // Executes the I2C command
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error sending command 0x%02X: %s", command, esp_err_to_name(ret));
    }

    i2c_cmd_link_delete(cmd); // Deletes the command link after execution
    ESP_LOGI(TAG, "LCD command 0x%02X sent", command);
}

// Sends a data nibble to the LCD to display characters or move the cursor
void lcd_send_data(uint8_t nibble)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);

    // Sends the I2C address in write mode
    i2c_master_write_byte(cmd, (LCD_ADDR << 1) | I2C_MASTER_WRITE, true);

    // Sends the data nibble with RS=1 (data mode), E=1 (enable high), and backlight on
    i2c_master_write_byte(cmd, (nibble << 4) | LCD_BACKLIGHT | 0x05, true); // RS=1, E=1, Backlight=1
    i2c_master_write_byte(cmd, (nibble << 4) | LCD_BACKLIGHT | 0x01, true); // RS=1, E=0, Backlight=1 (latch)

    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000)); // Sends the queued I2C commands
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Error sending data nibble: %s", esp_err_to_name(ret));
    }
    i2c_cmd_link_delete(cmd); // Deletes the I2C command link
}

// Initializes the LCD with required settings (4-bit mode, display on, cursor off)
void lcd_init(gpio_num_t scl_pin, gpio_num_t sda_pin)
{
    ESP_LOGI(TAG, "Initializing LCD...");

    i2c_master_init(scl_pin, sda_pin);

    // Sends a dummy command to enable the backlight initially
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LCD_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, LCD_BACKLIGHT, true); // Enables backlight
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    vTaskDelay(50 / portTICK_PERIOD_MS); // Waits for the LCD to stabilize after powering on

    // Step 1: Sends 0x30 three times to initialize the LCD in 8-bit mode
    lcd_send_command(0x30);
    vTaskDelay(5 / portTICK_PERIOD_MS);
    lcd_send_command(0x30);
    vTaskDelay(1 / portTICK_PERIOD_MS);
    lcd_send_command(0x30);
    vTaskDelay(1 / portTICK_PERIOD_MS);

    // Step 2: Switches to 4-bit mode
    lcd_send_command(LCD_CMD_FUNCTION_SET | LCD_4BIT_MODE);

    // Step 3: Finalizes 4-bit mode setup
    lcd_send_command(LCD_CMD_FUNCTION_SET | LCD_4BIT_MODE | LCD_2LINE | LCD_5x8_DOTS);
    vTaskDelay(2 / portTICK_PERIOD_MS);
    lcd_send_command(LCD_CMD_DISPLAY_CONTROL | LCD_DISPLAY_ON | LCD_CURSOR_OFF | LCD_BLINK_OFF);
    vTaskDelay(2 / portTICK_PERIOD_MS);
    lcd_send_command(LCD_CMD_ENTRY_MODE_SET | LCD_ENTRY_LEFT | LCD_ENTRY_SHIFT_DECREMENT);
    vTaskDelay(2 / portTICK_PERIOD_MS);
    lcd_send_command(LCD_CMD_CLEAR_DISPLAY); // Clears the display
    vTaskDelay(2 / portTICK_PERIOD_MS);      // Allows time for display clearing

    ESP_LOGI(TAG, "LCD initialized.");

}

// Writes a single character to the LCD
void lcd_write_char(char ch)
{
    uint8_t ascii_value = (uint8_t)ch; // Converts character to its ASCII code

    // Logs the character to display its ASCII value in hexadecimal
    ESP_LOGI(TAG, "Writing character '%c' to LCD (ASCII hex: 0x%02X)", ch, ascii_value);

    // Sends upper nibble (4 most significant bits) of ASCII code
    lcd_send_data((ascii_value & 0xF0) >> 4); // Send upper nibble

    // Sends lower nibble (4 least significant bits) of ASCII code
    lcd_send_data(ascii_value & 0x0F); // Send lower nibble
}

// Displays a string of text on the LCD
void lcd_display_text(const char *text, uint8_t row)
{
    ESP_LOGI(TAG, "Displaying text on LCD: \"%s\" on row %d", text, row);

    if (row > 1) {
        ESP_LOGW(TAG, "Invalid row number %d, defaulting to row 0", row);
        row = 0;
    }

    // Set cursor to start of specified row
    uint8_t row_addr = (row == 0) ? 0x00 : 0x40;
    lcd_send_command(LCD_CMD_SET_DDRAM_ADDR | row_addr);

    // Write up to 16 characters on the specified row
    for (int i = 0; i < 16 && text[i] != '\0'; i++)
    {
        lcd_write_char(text[i]);
    }
}
