#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "../../application/motorController/MotorController.h"
#include "../../application/PIDController/Inc/PIDController.h"
#include "../../application/Server/Inc/Server.h"

#define pdSECOND_motors pdMS_TO_TICKS(4000) 
#define pdSECOND_LED pdMS_TO_TICKS(500) 
#define LED_PIN GPIO_NUM_2
#define HIGH    1
#define LOW     0

// Constants for the server simulation
constexpr u_int32_t MINISUMO_UPDATE_INTERVAL = 50; // 20 Hz
constexpr u_int32_t FOLLOWER_UPDATE_INTERVAL = 30; // ~33 Hz

// Structure to hold simulation state
typedef struct {
  Server* server;
  float simulationTime;
  bool isRunning;
} simulationContext_t;

// Function to generate random float within range
float randomFloat(float min, float max);

// Function to generate random int within range
int randomInt(int min, int max);

// Update minisumo robot with sumalated data
void updateMinisumoData(MiniSumoEndpoints* minisumo, float simulationTime);

// Update line follower robot with simulated data
void updateFollowerData(FollowerEndpoints* lineFollower, float simulationTime);

// Robot simulation task
void robotSimulationTask(void* pvParameters);

// Minisumo robot internal data structure (for testing only)
struct MinisumoTestData {
  int lineSensorLeft;
  int lineSensorRight;
  int opponentLeft;
  int opponentRight;
  int opponentCenter;
};

// Line follower robot internal data structure (for testing only)
struct LineFollowerTestData {
  std::array<int, 16> lineSensors;
  float leftSpeed;
  float rightSpeed;
  int leftEncoder;
  int rightEncoder;
  int leftMotorSpeed;
  int rightMotorSpeed;
  float accelX;
  float accelY; 
  float accelZ;
  float gyroX;
  float gyroY;
  float gyroZ;
};

void setup(void);
void blinkLed(void);