#include "MotorController.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include <stdexcept>

// #define ESP_LOG_LEVEL_LOCAL ESP_LOG_WARN
#define TAG "MotorController"
namespace motorController {
  /*
  * @brief Pins for MOTOR object. 
  * @arg IN_1_PIN: Pin for motor input 1
  * @arg IN_2_PIN: Pin for motor input 2
  * @arg PWM_PIN: Pin for motor PWM
  * @arg pwm_channel: PWM channel for each motor
  * @note The pins are set in the constructor
  */
  MotorController::MotorController(gpio_num_t IN_1_PIN, gpio_num_t IN_2_PIN, gpio_num_t PWM_PIN, uint8_t channel) {
    this->IN_1_PIN = IN_1_PIN;
    this->IN_2_PIN = IN_2_PIN;
    this->PWM_PIN = PWM_PIN;
    
    // Validate and convert channel number to LEDC channel enum
    if (channel < 0 || channel > 7) {
      // If invalid channel, default to channel 0 and log warning
      this->pwm_channel = LEDC_CHANNEL_0;
      // ESP_LOGW(TAG, "Invalid channel %d, defaulting to channel 0", channel);
    } else {
      // Convert int to the corresponding LEDC_CHANNEL enum value
      // ESP32-S3 LEDC channels are sequential from 0-7
      this->pwm_channel = static_cast<ledc_channel_t>(channel);
    }
  }

  esp_err_t MotorController::init(void) {

    esp_err_t status = ESP_OK;
    gpio_config_t io_conf = {};

    try
    {
      // Reset pins to default state
      gpio_reset_pin(this->IN_1_PIN);
      gpio_reset_pin(this->IN_2_PIN);
      gpio_reset_pin(this->PWM_PIN);

      // Configure pins for motor control
      io_conf.intr_type = GPIO_INTR_DISABLE;
      io_conf.mode = GPIO_MODE_OUTPUT;
      io_conf.pin_bit_mask = (1ULL << this->IN_1_PIN) | (1ULL << this->IN_2_PIN) | (1ULL << this->PWM_PIN);
      io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
      io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
      gpio_config(&io_conf);

      // Set initial state - motor stopped
      gpio_set_level(this->IN_1_PIN, LOW);
      gpio_set_level(this->IN_2_PIN, LOW);
      gpio_set_level(this->PWM_PIN, 0);

      // Config PWM
      this->configPwm();
    }
    catch(const std::exception &exc)
    {
      status = ESP_FAIL;
    }

    return status;
  }

  void MotorController::configPwm(void) {
    // Static variable to track if timer has been configured
    static bool timer_configured = false;

    if (!timer_configured) {
      /*
      * Timer configuration - only needs to be done once for all motors
      * - 10 kHz frequency for good motor control
      * - 10-bit resolution (0-1023) for fine speed control
      */
      ledc_timer_config_t timer_config = {};
      timer_config.speed_mode = LEDC_LOW_SPEED_MODE;
      timer_config.duty_resolution = LEDC_TIMER_10_BIT;
      timer_config.timer_num = LEDC_TIMER_0;
      timer_config.freq_hz = 10000;                                         

      ESP_ERROR_CHECK(ledc_timer_config(&timer_config));
      timer_configured = true;
    }

    /*
    * PWM channel configuration for this specific motor
    */
    ledc_channel_config_t channel_config = {};
    channel_config.gpio_num = this->PWM_PIN;
    channel_config.channel = this->pwm_channel;
    channel_config.duty = 0;
    channel_config.intr_type = LEDC_INTR_DISABLE;
    channel_config.speed_mode = LEDC_LOW_SPEED_MODE;
    channel_config.timer_sel = LEDC_TIMER_0;
    channel_config.hpoint = 0;

    ESP_ERROR_CHECK(ledc_channel_config(&channel_config));

    // Initialize duty cycle to 0
    setDuty(0);
  }

  void MotorController::setDuty(uint16_t duty) {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, this->pwm_channel, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, this->pwm_channel);
  }

  void MotorController::motorGo(int16_t speed) {
    // Determine direction based on sign of speed
    if (speed > 0) {
      // Forward direction
      gpio_set_level(this->IN_1_PIN, HIGH);
      gpio_set_level(this->IN_2_PIN, LOW);
    } else if (speed < 0) {
      // Backward direction
      gpio_set_level(this->IN_1_PIN, LOW);
      gpio_set_level(this->IN_2_PIN, HIGH);
    } else {
      // Stop if speed is 0
      softStop();
      return;
    }

    // Convert speed to positive value and cap at 1023
    uint32_t absSpeed = abs(speed);
    if (absSpeed > 1023) {
      absSpeed = 1023;
    }
    
    // Set duty cycle for the motor using PWM
    setDuty(absSpeed);
  }

  void MotorController::softStop(void) {
    // Set both direction pins LOW to let motor coast to a stop
    gpio_set_level(this->IN_1_PIN, LOW);
    gpio_set_level(this->IN_2_PIN, LOW);
    setDuty(0);
  }

  void MotorController::hardStop(void) {
    // Set both direction pins HIGH to create electrical braking
    gpio_set_level(this->IN_1_PIN, HIGH);
    gpio_set_level(this->IN_2_PIN, HIGH);
    setDuty(0);
  }
} // namespace motorController
