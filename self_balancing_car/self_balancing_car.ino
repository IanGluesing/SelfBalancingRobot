#include "Wire.h"
#include <Arduino_FreeRTOS.h>

#include "src/tasks/task_balance_control_output.h"
#include "src/tasks/task_encoder_count.h"
#include "src/utils/shared_variables.h"

void setup() {
  Serial.begin(115200);
  while(!Serial) {}
  
  Wire.begin();
  mpu.initialize();

  init_encoder_count_interrupts();

  // Spawn task to get gyro data
  xTaskCreate(task_balance_control_output, "Gyro Data Task", 128, NULL, 1, NULL);
  xTaskCreate(test_task, "Gyro Data Task", 128, NULL, 1, NULL);
}

void test_task(void* pvParameters) {
  (void) pvParameters;

  while (true) {
    if (xSemaphoreTake(balance_control_output_value_mutex, portMAX_DELAY) == pdTRUE) {
      Serial.println("task_balance_control_output_value: " + String(task_balance_control_output_value));
      xSemaphoreGive(balance_control_output_value_mutex);
    }
    if (xSemaphoreTake(encoder_count_mutex, portMAX_DELAY) == pdTRUE) {
      Serial.println("encoder_count_left_a: " + String(encoder_count_left_a) + " encoder_count_right_a: " + String(encoder_count_right_a));
      xSemaphoreGive(encoder_count_mutex);
    }
  
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void loop() {
}
