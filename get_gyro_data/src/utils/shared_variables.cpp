#include "shared_variables.h"

MPU6050 mpu;

volatile float angleAroundXAxis = 0;
SemaphoreHandle_t balance_control_output_value_mutex = xSemaphoreCreateMutex();