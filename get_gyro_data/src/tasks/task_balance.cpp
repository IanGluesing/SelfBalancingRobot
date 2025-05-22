#include "task_balance.h"
#include <Arduino_FreeRTOS.h>
#include "../utils/shared_variables.h"

// How often this task occurs
float task_balance_dt_milliseconds = 5;

void task_balance(void* pvParameters) {
    (void) pvParameters;
    float integral = 0;
    float prevError = 0;

    while (true) {

        if (xSemaphoreTake(balance_control_output_value_mutex, portMAX_DELAY) == pdTRUE) {
            float error = -3 - angleAroundXAxis;
            integral += error * (task_balance_dt_milliseconds / 1000);
            float der = (error - prevError) / (task_balance_dt_milliseconds / 1000);
            prevError = error;
            integral = constrain(integral, -5000, 5000);

            float P = 40 * error;
            float I = 0 * integral;
            float D = 2 * der;

            // Serial.println("P: " + String(P) + " I: " + String(I) + " D: " + String(D));

            float motor_output = constrain(P + I + D, -255, 255);

            if (motor_output < 0) {
            motor.Forward(-motor_output);
            } else {
            motor.Back(motor_output);
            }
            
            xSemaphoreGive(balance_control_output_value_mutex);
        }

        vTaskDelay(task_balance_dt_milliseconds / portTICK_PERIOD_MS);
    }
}