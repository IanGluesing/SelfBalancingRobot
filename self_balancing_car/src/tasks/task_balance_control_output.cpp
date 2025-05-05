#include "task_balance_control_output.h"
#include <Arduino_FreeRTOS.h>
#include "../utils/shared_variables.h"

// How often this task occurs
uint8_t task_balance_control_output_dt_milliseconds = 5;

// Acceleration of the MPU in direction of the given axis
int16_t accelerationAlongXAxis, accelerationAlongYAxis, accelerationAlongZAxis;

// Weight
float weight_factor = 0.05;

// Historical angle around the X axis
float angleAroundXAxis = 0;

uint8_t kp_balance = 55;
float kd_balance = .75;

void task_balance_control_output(void* pvParameters) {
    (void) pvParameters;

    while (true) {
        // Rotational velocity around X axis, divide by 131 to account for FS_SEL = 0 to convert from rotational units to degrees/sec
        float rotationalVelocityAroundXAxis = mpu.getRotationX() / 131.0; // Units: Degrees/Sec
      
        // Get acceleration along the X, Y, and Z axis. These are given in accelerometer units.
        mpu.getAcceleration(&accelerationAlongXAxis, &accelerationAlongYAxis, &accelerationAlongZAxis);
      
        // Determine 'angle of gravity' using acceleration values along Y and Z axis
        // Use this in tandem with the read rotation to determine best guess rotation value
        float angleOfGravityFromZAxis = atan2(accelerationAlongYAxis, accelerationAlongZAxis) * (180.0 / PI); // Angle of Gravity WRT Y anx Z axis
      
        // To determine the actual final angle, use simplified complimentary filter
        angleAroundXAxis = (1 - weight_factor) * (angleAroundXAxis + rotationalVelocityAroundXAxis * (task_balance_control_output_dt_milliseconds / 1000)) + (weight_factor * angleOfGravityFromZAxis);

        if (xSemaphoreTake(balance_control_output_value_mutex, portMAX_DELAY) == pdTRUE) {
            task_balance_control_output_value = kp_balance * angleAroundXAxis + kd_balance * rotationalVelocityAroundXAxis;
            // Serial.println("task_balance_control_output_value: " + String(task_balance_control_output_value));
            
            xSemaphoreGive(balance_control_output_value_mutex);
        }

        vTaskDelay(task_balance_control_output_dt_milliseconds / portTICK_PERIOD_MS);
    }
}