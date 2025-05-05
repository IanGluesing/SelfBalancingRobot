#include "shared_variables.h"

MPU6050 mpu;

volatile float task_balance_control_output_value = 0;
SemaphoreHandle_t balance_control_output_value_mutex = xSemaphoreCreateMutex();

volatile unsigned long encoder_count_left_a = 0;
volatile unsigned long encoder_count_right_a = 0;
// Use this when accessing encoder_count_left_a or encoder_count_right_a
SemaphoreHandle_t encoder_count_mutex = xSemaphoreCreateMutex();