#include "Wire.h"
#include <Arduino_FreeRTOS.h>
#include <semphr.h>
#include "PinChangeInt.h"

SemaphoreHandle_t encoderMutex;

void setup() {
  Serial.begin(115200);
  while(!Serial) {}
  Wire.begin();

  encoderMutex = xSemaphoreCreateMutex();
  attachPinChangeInterrupt(2, EncoderCountLeftA, CHANGE);
  attachPinChangeInterrupt(4, EncoderCountRightA, CHANGE);

  xTaskCreate(encoder_task, "Encoder task", 128, NULL, 1, NULL);
}

volatile unsigned long encoder_count_left_a;
volatile unsigned long encoder_count_right_a;

void EncoderCountLeftA()
{
  encoder_count_left_a++;
}

void EncoderCountRightA()
{
  encoder_count_right_a++;
}

void encoder_task(void* pvParameters) {
  (void) pvParameters;

  while (true) {

    if (xSemaphoreTake(encoderMutex, portMAX_DELAY) == pdTRUE) {
      Serial.println(String(encoder_count_left_a) + " " + String(encoder_count_right_a));
      xSemaphoreGive(encoderMutex);
    }
    
    vTaskDelay(5);
  }
}

void loop() {
}
