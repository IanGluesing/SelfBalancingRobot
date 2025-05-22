#include "current_car_speed.h"

volatile float current_car_speed = 0;
SemaphoreHandle_t current_car_speed_mutex = xSemaphoreCreateMutex();  