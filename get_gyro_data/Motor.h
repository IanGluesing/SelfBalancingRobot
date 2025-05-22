#pragma once

class Motor
{
  public:
    Motor() {}
    
    void Pin_init();
    void Encoder_init();
    
    void Stop();
    void Forward(int speed);
    void Back(int speed);

  public:
    static volatile unsigned long encoder_count_left_a;
    static volatile unsigned long encoder_count_right_a;
          
  private:
    // Pins as defined in IO Pin connection table coming from TB6612FNG
    #define AIN1 7
    #define PWMA_LEFT 5
    #define BIN1 12
    #define PWMB_RIGHT 6
    #define STBY_PIN 8
    
    // Pins as defined in IO Pin connection table coming from GA37-520 DC geared motor
    #define ENCODER_LEFT_A_PIN 2
    #define ENCODER_RIGHT_A_PIN 4
};
