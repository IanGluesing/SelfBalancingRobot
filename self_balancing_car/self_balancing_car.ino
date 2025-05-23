#include "Wire.h"
#include <Arduino_FreeRTOS.h>

#include "src/tasks/task_balance_control_output.h"
#include "src/tasks/task_encoder_count.h"
#include "src/tasks/task_speed_control_output.h"
#include "src/tasks/task_balance.h"
#include "src/tasks/task_encoder_speed.h"
#include "src/utils/shared_variables.h"

void setup() {
  motor.Pin_init();
  Serial.begin(115200);
  while(!Serial) {}
  
  Wire.begin();
  mpu.initialize();

  init_encoder_count_interrupts();
  
  // Spawn task to get gyro data
  xTaskCreate(task_balance_control_output, "Gyro Data Task", 128, NULL, 1, NULL); // ==100
  xTaskCreate(task_speed_control_output, "Speed control output", 256, NULL, 1, NULL); // ~<60
//  xTaskCreate(task_encoder_speed, "Encoder speed task", 128, NULL, 1, NULL); // ~<100
//  xTaskCreate(task_balance, "Balance Task", 64, NULL, 1, NULL); // ~<90
}



void loop() {
}
