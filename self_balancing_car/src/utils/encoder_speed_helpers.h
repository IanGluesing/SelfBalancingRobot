#pragma once

#include <Arduino_FreeRTOS.h>
#include "semphr.h"

extern volatile int16_t encoder_speed_left;
extern volatile int16_t encoder_speed_right;
extern volatile uint16_t encoder_count_left_a;
extern volatile uint16_t encoder_count_right_a;
extern SemaphoreHandle_t encoder_speed_mutex;