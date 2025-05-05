#pragma once

#include <Arduino_FreeRTOS.h>
#include "MPU6050.h"
#include "semphr.h"

extern MPU6050 mpu;

extern volatile float task_balance_control_output_value;
extern SemaphoreHandle_t balance_control_output_value_mutex;

extern volatile unsigned long encoder_count_left_a;
extern volatile unsigned long encoder_count_right_a;
extern SemaphoreHandle_t encoder_count_mutex;
