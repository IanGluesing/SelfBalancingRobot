#include <Arduino.h>
#include "Motor.h"
#include "PinChangeInt.h"

static void EncoderCountLeftA();
static void EncoderCountRightA();

void Motor::Pin_init()
{
  pinMode(AIN1, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(PWMA_LEFT, OUTPUT);
  pinMode(PWMB_RIGHT, OUTPUT);
  pinMode(STBY_PIN, OUTPUT);
  digitalWrite(STBY_PIN, HIGH);
}

void Motor::Stop()
{
  // Set speeds zero
  analogWrite(PWMA_LEFT, 0);
  analogWrite(PWMB_RIGHT, 0);
}

void Motor::Forward(int speed)
{
  // Set ain1/bin1 pins low for forward direction
  digitalWrite(AIN1, 0);
  digitalWrite(BIN1, 0);
  analogWrite(PWMA_LEFT, speed);
  analogWrite(PWMB_RIGHT, speed);
}

void Motor::Back(int speed)
{
  // Set ain1/bin1 pins high for backward direction
  digitalWrite(AIN1, 1);
  digitalWrite(BIN1, 1);
  analogWrite(PWMA_LEFT, speed);
  analogWrite(PWMB_RIGHT, speed);
}

unsigned long Motor::encoder_count_left_a;
unsigned long Motor::encoder_count_right_a;

void Motor::Encoder_init()
{
  // Add interrupts for encoder pulses
  attachPinChangeInterrupt(ENCODER_LEFT_A_PIN, EncoderCountLeftA, CHANGE);
  attachPinChangeInterrupt(ENCODER_RIGHT_A_PIN, EncoderCountRightA, CHANGE);
}

static void EncoderCountLeftA()
{
  Motor::encoder_count_left_a++;
}

static void EncoderCountRightA()
{
  Motor::encoder_count_right_a++;
}
