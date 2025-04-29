#include "Motor.h"

Motor Motor;

#define SPEED 100

void log_stats() {
  noInterrupts();
  Serial.println("Left count: " + String(Motor.encoder_count_left_a));
  Serial.println("Right count: " + String(Motor.encoder_count_right_a));
  Motor.encoder_count_left_a=0;
  Motor.encoder_count_right_a=0;
  interrupts();
}

void setup() 
{
  Motor.Pin_init();
  Motor.Encoder_init();
  Serial.begin(115200);
  delay(2000);
}

void loop() {
  Motor.Forward(SPEED);
  delay(2000);

  log_stats();

  Motor.Back(SPEED);
  delay(2000);
  
  log_stats();
}
