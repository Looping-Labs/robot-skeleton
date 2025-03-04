#include "main.h"
#include "esp_log.h"

using namespace motorController;

#define LOG_LEVEL_LOCAL ESP_LOG_VERBOSE
#define TAG "MAIN"

const gpio_num_t IN_1_MOTOR_LEFT = GPIO_NUM_23;
const gpio_num_t IN_2_MOTOR_LEFT = GPIO_NUM_22;
const gpio_num_t PWM_MOTOR_LEFT = GPIO_NUM_1;
const gpio_num_t IN_1_MOTOR_RIGHT = GPIO_NUM_19;
const gpio_num_t IN_2_MOTOR_RIGHT = GPIO_NUM_18;
const gpio_num_t PWM_MOTOR_RIGHT = GPIO_NUM_5;

MotorController motorLeft(IN_1_MOTOR_LEFT, IN_2_MOTOR_LEFT, PWM_MOTOR_LEFT, 0);
MotorController motorRight(IN_1_MOTOR_RIGHT, IN_2_MOTOR_RIGHT, PWM_MOTOR_RIGHT, 1);

extern "C" void app_main() {
  ESP_LOGI(TAG, "Initial setup");
  setup();

  while (true) {
    motorRight.motorGo(150);
    motorLeft.motorGo(150);
    vTaskDelay(pdSECOND_motors);
    motorRight.motorGo(-150);
    motorLeft.motorGo(-150);
    vTaskDelay(pdSECOND_motors);
    motorRight.softStop();
    motorLeft.softStop();
    vTaskDelay(pdSECOND_motors);
  }
}

void setup(void) {
  gpio_reset_pin(LED_PIN);
  gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
  motorRight.init();
  motorLeft.init();

  for (int i = 0; i < 3; i++) {
    blinkLed();
  }
  
  ESP_LOGI(TAG, "Setup complete");
}

void blinkLed(void) {
  gpio_set_level(LED_PIN, HIGH);
  vTaskDelay(pdSECOND_LED);
  gpio_set_level(LED_PIN, LOW);
  vTaskDelay(pdSECOND_LED);
}