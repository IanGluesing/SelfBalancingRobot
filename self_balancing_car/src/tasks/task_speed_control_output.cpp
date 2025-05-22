#include "task_speed_control_output.h"
#include <Arduino_FreeRTOS.h>
#include "../utils/shared_variables.h"
#include "../utils/encoder_speed_helpers.h"
#include "../utils/current_car_speed.h"

// How often this task occurs
uint8_t task_speed_control_output_dt_milliseconds = 5;

uint8_t kp_speed = 10;
double ki_speed = .26;

double car_speed = 0;
double speed_filter = 0;
double car_speed_integral = 0;

double encoder_speed_multiplier = 0.5;
double car_speed_filter_multiplier = 0.7;
double car_speed_multiplier = 0.3;

int16_t car_speed_integral_lower_bounrd = -3000;
int16_t car_speed_integral_upper_bounrd = 3000;

int count = 0;

void task_speed_control_output(void* pvParameters) {
    (void) pvParameters;

    while (true) {
        count += 1;
        if (xSemaphoreTake(encoder_speed_mutex, portMAX_DELAY) == pdTRUE) {
            encoder_speed_left += (current_car_speed < 0) ? -encoder_count_left_a : encoder_count_left_a;
            encoder_speed_right += (current_car_speed < 0) ? -encoder_count_right_a : encoder_count_right_a;

            encoder_count_left_a = 0;
            encoder_count_right_a = 0;
            xSemaphoreGive(encoder_speed_mutex);
        }

        if (count > 10) {
            if (xSemaphoreTake(encoder_speed_mutex, portMAX_DELAY) == pdTRUE) {
                car_speed = (encoder_speed_left + encoder_speed_right) * encoder_speed_multiplier;

                encoder_speed_left = 0;
                encoder_speed_right = 0;
                xSemaphoreGive(encoder_speed_mutex);
            }
    
            speed_filter = speed_filter * car_speed_filter_multiplier + car_speed * car_speed_multiplier;
            car_speed_integral += speed_filter;
            car_speed_integral = constrain(car_speed_integral,  car_speed_integral_lower_bounrd, car_speed_integral_upper_bounrd);
    
            if (xSemaphoreTake(speed_control_output_mutex, portMAX_DELAY) == pdTRUE) {
                // Serial.println("b " + String(speed_filter) + " s:  " + String(car_speed_integral));
                task_speed_control_output_value = -kp_speed * speed_filter - ki_speed * car_speed_integral;
                xSemaphoreGive(speed_control_output_mutex);
            }
            count = 0;
        }

        if ((xSemaphoreTake(balance_control_output_value_mutex, portMAX_DELAY) == pdTRUE) &&
             xSemaphoreTake(speed_control_output_mutex, portMAX_DELAY) == pdTRUE) {
            
            current_car_speed = constrain(task_balance_control_output_value - task_speed_control_output_value, -255, 255);

            if (current_car_speed < 0) {
                motor.Back(-current_car_speed);
            } else {
                motor.Forward(current_car_speed);
            }

            Serial.println("b " + String(task_balance_control_output_value) + " s:  " + String(task_speed_control_output_value) + " cp: " + String(current_car_speed));
            
            xSemaphoreGive(balance_control_output_value_mutex);
            xSemaphoreGive(speed_control_output_mutex);
        }

        vTaskDelay(task_speed_control_output_dt_milliseconds / portTICK_PERIOD_MS);
    }
}