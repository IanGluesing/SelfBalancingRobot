#include "task_balance_control_output.h"
#include <Arduino_FreeRTOS.h>
#include "../utils/shared_variables.h"

// How often this task occurs
uint8_t task_balance_control_output_dt_milliseconds = 5;

// Acceleration of the MPU in direction of the given axis
int16_t accel_along_x_axis, accel_along_y_axis, accel_along_z_axis;

// Weight
float weight_factor = 0.05;

// Temp helpers
float rotationalVelocityAroundXAxis = 0;
float angleOfGravityFromZAxis = 0;

void task_balance_control_output(void* pvParameters) {
    (void) pvParameters;

    while (true) {
        // Rotational velocity around X axis, divide by 131 to account for FS_SEL = 0 to convert from rotational units to degrees/sec
        rotationalVelocityAroundXAxis = mpu.getRotationX() / 131.0; // Units: Degrees/Sec
      
        // Get acceleration along the X, Y, and Z axis. These are given in accelerometer units.
        mpu.getAcceleration(&accel_along_x_axis, &accel_along_y_axis, &accel_along_z_axis);
      
        // Determine 'angle of gravity' using acceleration values along Y and Z axis
        // Use this in tandem with the read rotation to determine best guess rotation value
        angleOfGravityFromZAxis = atan2(accel_along_y_axis, accel_along_z_axis) * (180.0 / PI); // Angle of Gravity WRT Y anx Z axis

        if (xSemaphoreTake(balance_control_output_value_mutex, portMAX_DELAY) == pdTRUE) {
            // To determine the actual final angle, use simplified complimentary filter
            angle_around_x_axis = (1 - weight_factor) * (angle_around_x_axis + rotationalVelocityAroundXAxis * (task_balance_control_output_dt_milliseconds / 1000)) + (weight_factor * angleOfGravityFromZAxis);
            // BTSerial.println("angle_around_x_axis" + String(angle_around_x_axis));
            xSemaphoreGive(balance_control_output_value_mutex);
        }

        vTaskDelay(task_balance_control_output_dt_milliseconds / portTICK_PERIOD_MS);
    }
}