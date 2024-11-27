#include "lcd1602.h"
#include "lcd1602_private.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_task_wdt.h"

static const char *TAG = "LCD_Animation";

// ASCII animation frames for a simple loading animation
static const char *ascii_frames[] = {
    "     Ready     ",
    ">    Ready    <",
    ">>   Ready   <<",
    ">>>  Ready  <<<",
    ">>>> Ready <<<<",
};
#define NUM_FRAMES (sizeof(ascii_frames) / sizeof(ascii_frames[0]))

// Animation task handle
static TaskHandle_t animation_task_handle = NULL;
// Animation pause flag
static bool animation_paused = false;

// Animation task function
static void ascii_animation_task(void *pvParameters)
{
    // Subscribe to watchdog timer if not already subscribed
    if (esp_task_wdt_status(NULL) != ESP_OK) {
        ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
    }
    ESP_ERROR_CHECK(esp_task_wdt_reset());

    int frame = 0;
    while (animation_task_handle != NULL)
    {
        // Reset watchdog timer first thing in the loop
        ESP_ERROR_CHECK(esp_task_wdt_reset());

        // Only display animation if not paused
        if (!animation_paused) {
            // Display current frame
            lcd_display_text(ascii_frames[frame], 1);

            // Move to next frame
            frame = (frame + 1) % NUM_FRAMES;
        }

        // Add a proper delay that allows other tasks to run
        vTaskDelay(pdMS_TO_TICKS(80));
    }

    // Clean up watchdog timer before exit
    if (esp_task_wdt_status(NULL) == ESP_OK) {
        ESP_ERROR_CHECK(esp_task_wdt_delete(NULL));
    }
    vTaskDelete(NULL);
}

// Function to start animation
void lcd_start_animation(void)
{
    // Reset pause flag
    animation_paused = false;

    // If there's no existing task, create one
    if (animation_task_handle == NULL) {
        // Create new task
        xTaskCreate(ascii_animation_task,
                    "ascii_animation",
                    4096,
                    NULL,
                    2,    // Lower priority to prevent starving other tasks
                    &animation_task_handle);
    }
}

// Function to stop animation
void lcd_stop_animation(void)
{
    // Pause the animation
    animation_paused = true;
    
    // Clear the display
    lcd_send_command(LCD_CMD_CLEAR_DISPLAY);
    vTaskDelay(pdMS_TO_TICKS(10)); // Small delay after clear
} 