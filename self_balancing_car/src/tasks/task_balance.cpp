#include "task_balance.h"
#include <Arduino_FreeRTOS.h>
#include "../utils/shared_variables.h"

// How often this task occurs
uint8_t task_balance_dt_milliseconds = 100;

void task_balance(void* pvParameters) {
    (void) pvParameters;

    while (true) {

        vTaskDelay(task_balance_dt_milliseconds / portTICK_PERIOD_MS);
    }
}