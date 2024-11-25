#pragma once

#include <stdint.h>

typedef struct
{
    uint8_t red;
    uint8_t green;
    uint8_t blue;
    char color_name[32]; // Add this to store color name
} rgb_data_t;

// Data structure to hold color data
typedef struct
{
    uint16_t red;
    uint16_t green;
    uint16_t blue;
    uint16_t clear;
} ColorData;

typedef struct
{
    const char *name;
    uint8_t r, g, b;
} RGBColor;
