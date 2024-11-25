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

// ESP-NOW configuration
#define ESPNOW_CHANNEL 1
#define PRINTSCANRESULTS 0
#define DELETEBEFOREPAIR 0

// Task configuration
#define ESP_NOW_TASK_STACK_SIZE 4096
#define ESP_NOW_TASK_PRIORITY 5
#define QUEUE_SIZE 10

// Task handle for the ESP-NOW tasks
extern TaskHandle_t esp_now_task_handle;

// Function to initialize ESP-NOW sender
esp_err_t initialize_esp_now_sender(void);

// Function to initialize ESP-NOW receiver
esp_err_t initialize_esp_now_receiver(void);

// Function to add RGB data to the sending queue
void add_rgb_to_send_queue(const rgb_data_t *data);
