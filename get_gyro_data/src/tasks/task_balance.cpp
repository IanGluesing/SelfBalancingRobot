#include "task_balance.h"
#include <Arduino_FreeRTOS.h>
#include "../utils/shared_variables.h"

// How often this task occurs
float task_balance_dt_milliseconds = 5;

void task_balance(void* pvParameters) {
    (void) pvParameters;

    float integral = 0;
    float previous_error = 0;
    float previous_error_derivative = 0;

    while (true) {

        if (xSemaphoreTake(balance_control_output_value_mutex, portMAX_DELAY) == pdTRUE) {
            // Get current error based on angle around the x axis
            float error = 0 - angle_around_x_axis;

            // Sum all error as time goes on
            integral += error * (task_balance_dt_milliseconds / 1000);

            // Determine derivative of the error at the current time
            float current_error_derivative = (error - previous_error) / (task_balance_dt_milliseconds / 1000);
            current_error_derivative = .7 * current_error_derivative + .3 * previous_error_derivative;
            previous_error_derivative = current_error_derivative;

            // Set previous error and constrain integral term
            previous_error = error;
            integral = constrain(integral, -5000, 5000);
            
            // Determine PID values based on Kp, Ki, Kd terms
            float P = 30 * error;
            float I = 0 * integral;
            float D = 2.75 * der;

            // Determine motor output
            float motor_output = constrain(P - I + D, -255, 255);

            // Set motor velocity based on motor output values
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