#pragma once

#include "PinChangeInt.h"
#include "../utils/shared_variables.h"

#define ENCODER_LEFT_A_PIN 2
#define ENCODER_RIGHT_A_PIN 4

void EncoderCountLeftA()
{
  encoder_count_left_a++;
}

void EncoderCountRightA()
{
  encoder_count_right_a++;
}

void init_encoder_count_interrupts() {
  attachPinChangeInterrupt(ENCODER_LEFT_A_PIN, EncoderCountLeftA, CHANGE);
  attachPinChangeInterrupt(ENCODER_RIGHT_A_PIN, EncoderCountRightA, CHANGE);
}