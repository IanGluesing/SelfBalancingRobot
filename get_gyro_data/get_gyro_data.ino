#include "Wire.h"
#include <Arduino_FreeRTOS.h>

#include "src/tasks/task_balance_control_output.h"
#include "src/tasks/task_balance.h"
#include "src/utils/shared_variables.h"

void setup() {
  motor.Pin_init();
  Serial.begin(115200);
  while(!Serial) {}
  
  Wire.begin();
  mpu.initialize();

  // Spawn task to get gyro data
  xTaskCreate(task_balance_control_output, "Get angle around X axis", 128, NULL, 1, NULL);

  // Spawn task to get gyro data
  xTaskCreate(task_balance, "Balance", 128, NULL, 1, NULL);
}

void loop() {
} 
