#include "encoder_speed_helpers.h"

volatile int16_t encoder_speed_left = 0;
volatile int16_t encoder_speed_right = 0;
volatile uint16_t encoder_count_left_a = 0;
volatile uint16_t encoder_count_right_a = 0;
// Use this when accessing encoder_count_left_a or encoder_count_right_a
SemaphoreHandle_t encoder_speed_mutex = xSemaphoreCreateMutex();  