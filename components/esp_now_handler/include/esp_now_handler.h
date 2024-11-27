// esp_now_handler.h
#pragma once

#include <esp_now.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <esp_log.h>
#include <esp_mac.h>
#include <string.h>
#include "rgb_types.h"
#include "esp_task_wdt.h"

// ESP-NOW Protocol Configuration
#define ESPNOW_CHANNEL 1            // WiFi channel for ESP-NOW communication
#define PRINTSCANRESULTS 0          // Disable WiFi scan result printing
#define DELETEBEFOREPAIR 0          // Don't delete existing pairings

// Task Configuration
#define ESP_NOW_TASK_STACK_SIZE 4096  // Stack size for ESP-NOW tasks
#define ESP_NOW_TASK_PRIORITY 5       // Task priority (higher number = higher priority)
#define QUEUE_SIZE 10                 // Maximum pending messages in queue

// Task Management
extern TaskHandle_t esp_now_task_handle;  // Handle for ESP-NOW processing task

// ESP-NOW Initialization Functions
esp_err_t initialize_esp_now_sender(void);    // Initialize device as ESP-NOW sender
esp_err_t initialize_esp_now_receiver(void);  // Initialize device as ESP-NOW receiver

// Data Management Functions
void add_rgb_to_send_queue(const rgb_data_t *data);  // Queue RGB data for transmission
