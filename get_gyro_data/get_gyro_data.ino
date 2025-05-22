#include "Wire.h"
#include "MPU6050.h"
#include "Motor.h"
#include <Arduino_FreeRTOS.h>
#include <semphr.h>

MPU6050 mpu;
Motor motor;

int car_set_speed = 0;

// Estimated current angle around the X axis
float angleAroundXAxis;
float gyro_data_task_dt_milliseconds = 5;
void gyro_data_task(void *pvParameters);

SemaphoreHandle_t encoder_mutex;
int encoder_speed_left = 0;
int encoder_speed_right = 0;
float get_encoder_speed_task_dt_milliseconds = 5;
void get_encoder_speed(void *pvParameters);

void setup() {
  motor.Pin_init();
  motor.Encoder_init();
  Serial.begin(115200);
  while(!Serial) {}
  
  Wire.begin();
  mpu.initialize();

  encoder_mutex = xSemaphoreCreateMutex();

  // Spawn task to get gyro data
  xTaskCreate(gyro_data_task, "Gyro Data Task", 128, NULL, 1, NULL);
}

void gyro_data_task(void *pvParameters) {
  (void) pvParameters;

  // Acceleration of the MPU in direction of the given axis
  int16_t accelerationAlongXAxis, accelerationAlongYAxis, accelerationAlongZAxis;
  
  // Weight
  float weight_factor = 0.05;
  float integral = 0;
  float prevError = 0;
  
  while (true) {
    // Rotational velocity around X axis, divide by 131 to account for FS_SEL = 0 to convert from rotational units to degrees/sec
    float rotationalVelocityAroundXAxis = mpu.getRotationX() / 131.0; // Units: Degrees/Sec
  
    // Get acceleration along the X, Y, and Z axis. These are given in accelerometer units.
    mpu.getAcceleration(&accelerationAlongXAxis, &accelerationAlongYAxis, &accelerationAlongZAxis);
  
    // Determine 'angle of gravity' using acceleration values along Y and Z axis
    // Use this in tandem with the read rotation to determine best guess rotation value
    float angleOfGravityFromZAxis = atan2(accelerationAlongYAxis, accelerationAlongZAxis) * (180.0 / PI); // Angle of Gravity WRT Y anx Z axis
  
    // To determine the actual final angle, use simplified complimentary filter
    angleAroundXAxis = (1 - weight_factor) * (angleAroundXAxis + rotationalVelocityAroundXAxis * (gyro_data_task_dt_milliseconds / 1000)) + (weight_factor * angleOfGravityFromZAxis);

    float error = 0 - angleAroundXAxis;
    integral += error * gyro_data_task_dt_milliseconds;
    float der = (error - prevError) / gyro_data_task_dt_milliseconds;
    prevError = error;
    integral = constrain(integral, -5000, 5000);

    float P = 25.0 * error;
    float I = .1 * integral;
    float D = 3 * der;

    Serial.println("P: " + String(P) + " I: " + String(I) + " D: " + String(D));

    float motor_output = constrain(P + I + D, -255, 255);

    if (motor_output < 0) {
      motor.Forward(-motor_output);
    } else {
      motor.Back(motor_output);
    }

    vTaskDelay(gyro_data_task_dt_milliseconds / portTICK_PERIOD_MS);
  }
}

void loop() {
}
