#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "../../application/motorController/MotorController.h"

#define pdSECOND_motors pdMS_TO_TICKS(4000) 
#define pdSECOND_LED pdMS_TO_TICKS(500) 
#define LED_PIN GPIO_NUM_2
#define HIGH    1
#define LOW     0

void setup(void);
void blinkLed(void);