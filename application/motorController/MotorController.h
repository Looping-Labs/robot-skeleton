#pragma once

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "stdint.h"

namespace motorController {
class MotorController {
  enum STATE_PIN{
    HIGH = 1,
    LOW = 0
  };

  private:
    /*
    * @brief Pins for MOTOR object. 
    * @arg IN_1_PIN: Pin for motor input 1
    * @arg IN_2_PIN: Pin for motor input 2
    * @arg PWM_PIN: Pin for motor PWM
    * @arg pwm_channel: PWM channel for each motor
    * @note The pins are set in the constructor
    */
    gpio_num_t IN_1_PIN;
    gpio_num_t IN_2_PIN;
    gpio_num_t PWM_PIN;
    ledc_channel_t pwm_channel;

  protected:
    void configPwm(void);
    void setDuty(uint16_t duty);
    
  public:
    MotorController(gpio_num_t IN_1_PIN, gpio_num_t IN_2_PIN, gpio_num_t PWM_PIN, uint8_t channel = 0);
    
    esp_err_t init(void);
    /* The motor will go forward or backward depending on the speed [-1023 - 1023]*/
    void motorGo(int16_t speed);                 
    /* The motor will slowly stop because of inertia */
    void softStop(void);
    /* The motor will immediately stop */
    void hardStop(void);
  };
} // namespace motorController