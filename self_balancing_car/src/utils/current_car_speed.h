#pragma once

#include <Arduino_FreeRTOS.h>
#include "semphr.h"

extern volatile float current_car_speed;
extern SemaphoreHandle_t current_car_speed_mutex;