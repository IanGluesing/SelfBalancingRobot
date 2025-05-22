#include "task_encoder_speed.h"
#include <Arduino.h>
#include "../utils/encoder_speed_helpers.h"
#include "../utils/current_car_speed.h"

// How often this task occurs
uint8_t task_encoder_speed_dt_milliseconds = 5;

void task_encoder_speed(void* pvParameters) {
    (void) pvParameters;

    while (true) {
        if (xSemaphoreTake(encoder_speed_mutex, portMAX_DELAY) == pdTRUE) {
            encoder_speed_left += (current_car_speed < 0) ? -encoder_count_left_a : encoder_count_left_a;
            encoder_speed_right += (current_car_speed < 0) ? -encoder_count_right_a : encoder_count_right_a;

            encoder_count_left_a = 0;
            encoder_count_right_a = 0;
            xSemaphoreGive(encoder_speed_mutex);
        }

        vTaskDelay(task_encoder_speed_dt_milliseconds / portTICK_PERIOD_MS);
    }
}