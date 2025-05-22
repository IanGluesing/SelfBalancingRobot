#pragma once

#include <Arduino_FreeRTOS.h>
#include "MPU6050.h"
#include "semphr.h"
#include "motor_helper.h"

extern MPU6050 mpu;
extern motor_helper motor;

extern volatile float angleAroundXAxis;
extern SemaphoreHandle_t balance_control_output_value_mutex;