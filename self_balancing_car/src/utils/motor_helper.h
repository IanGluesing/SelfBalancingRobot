#pragma once


class motor_helper
{
  public:
    motor_helper() {}
    
    void Pin_init();
    
    void Stop();
    void Forward(int speed);
    void Back(int speed);

  private:
    // Pins as defined in IO Pin connection table coming from TB6612FNG
    #define AIN1 7
    #define PWMA_LEFT 5
    #define BIN1 12
    #define PWMB_RIGHT 6
    #define STBY_PIN 8
};
