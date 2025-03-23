#include "main.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include <cmath>

using namespace motorController;

#define LOG_LEVEL_LOCAL ESP_LOG_VERBOSE
#define TAG "MAIN"
#define TAG_API "API"

const gpio_num_t IN_1_MOTOR_LEFT = GPIO_NUM_23;
const gpio_num_t IN_2_MOTOR_LEFT = GPIO_NUM_22;
const gpio_num_t PWM_MOTOR_LEFT = GPIO_NUM_1;
const gpio_num_t IN_1_MOTOR_RIGHT = GPIO_NUM_19;
const gpio_num_t IN_2_MOTOR_RIGHT = GPIO_NUM_18;
const gpio_num_t PWM_MOTOR_RIGHT = GPIO_NUM_5;

MotorController motorLeft(IN_1_MOTOR_LEFT, IN_2_MOTOR_LEFT, PWM_MOTOR_LEFT, 0);
MotorController motorRight(IN_1_MOTOR_RIGHT, IN_2_MOTOR_RIGHT, PWM_MOTOR_RIGHT, 1);

/**
 * Example code for API Robotics
 */

// Global simulation context
static simulationContext_t simulationContext = {};

// Implementation of utility functions
float randomFloat(float min, float max) {
  float random = static_cast<float>((esp_random()) / static_cast<float>(UINT32_MAX));
  return min + random * (max - min);
}

int randomInt(int min, int max) {
  return min + (esp_random() % (max - min + 1));
}

extern "C" void app_main() {
  ESP_LOGI(TAG, "Initial setup");
  setup();
  ESP_LOGI(TAG_API, "Robot API Example starting...");

  // Initialize NVS flash (needed for WiFi)
  esp_err_t res = nvs_flash_init();
  if (res == ESP_ERR_NVS_NO_FREE_PAGES || res == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    res = nvs_flash_init();
  }
  ESP_ERROR_CHECK(res);

  // Create the server instance
  Server *server = new Server();
  if (!server) {
    ESP_LOGE(TAG_API, "Failed to create server instance");
    return;
  }

  // Initialize the server
  res = server->init();
  if (res != ESP_OK) {
    ESP_LOGE(TAG_API, "Failed to initialize server: %d", res);
    delete server;
    return;
  }

  // Start the server
  res = server->start();
  if (res != ESP_OK) {
    ESP_LOGE(TAG_API, "Failed to start server: %d", res);
    delete server;
    return;
  }

  ESP_LOGI(TAG_API, "WebSocket server started successfully");

  // Initialize the simulation context
  simulationContext.server = server;
  simulationContext.simulationTime = 0.0f;
  simulationContext.isRunning = true;

  // Create simulation task
  xTaskCreate(robotSimulationTask, "simulationTask", 4096, &simulationContext, 5, NULL);

  while (true) {
    // Update the server (Handles WebSocket messages)
    server->update();

    // Sleep to avoid hogging the CPU
    vTaskDelay(pdMS_TO_TICKS(20));
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

void robotSimulationTask(void *pvParameters) {
  simulationContext_t *context = (simulationContext_t *)pvParameters;
  Server *server = context->server;

  // Get endpoint pointers
  MiniSumoEndpoints *minisumo = server->getMiniSumoEndpoints();
  FollowerEndpoints *follower = server->getFollowerEndpoints();

  // Check if endpoints are available
  if (!minisumo || !follower) {
    ESP_LOGE(TAG_API, "Failed to get robot endpoints");
    // vTaskDelay(NULL);
    return;
  }

  // Timekeeping variables
  uint32_t lastMinisumoUpdate = 0;
  uint32_t lastFollowerUpdate = 0;
  uint32_t currenTime = 0;

  ESP_LOGI(TAG_API, "Simulation task started");

  // Main simulation loop
  while (context->isRunning) {
    // Get current time in milliseconds
    currenTime = esp_timer_get_time() / 1000;

    // Update simulation time (in seconds)
    context->simulationTime = static_cast<float>(currenTime / 1000.0f);

    // Update minisuomo robot data if interval elapsed
    if (currenTime - lastMinisumoUpdate >= MINISUMO_UPDATE_INTERVAL) {
      updateMinisumoData(minisumo, context->simulationTime);
      lastMinisumoUpdate = currenTime;
    }

    // Update follower robot data if interval elapsed
    if (currenTime - lastFollowerUpdate >= FOLLOWER_UPDATE_INTERVAL) {
      updateFollowerData(follower, context->simulationTime);
      lastFollowerUpdate = currenTime;
    }

    // Short delay to avoid hogging the CPU
    vTaskDelay(pdMS_TO_TICKS(5));
  }

  ESP_LOGI(TAG_API, "Robot Simulation task ended");
  vTaskDelete(NULL);
}

// Update the minisumo robot with simulated data
void updateMinisumoData(MiniSumoEndpoints *minisumo, float t) {
  // For testing, we're directly accessing private members
  // In production, use proper getters and setters

  const size_t DATA_OFFSET = sizeof(void *) * 8;

  MinisumoTestData *data = reinterpret_cast<MinisumoTestData *>(
      reinterpret_cast<uint8_t *>(minisumo) + DATA_OFFSET);

  // Simulate robot moving over a line - when one sensor detects the line,
  // the other usually does not, creating alternating values
  data->lineSensorLeft = 1000 + 2000 * fabs(sin(t * 0.5));
  data->lineSensorRight = 1000 + 2000 * fabs(cos(t * 0.5));

  // Simulate opponent robot detection - occasional spikes as opponent moves in/out of view
  data->opponentLeft = randomInt(500, 800);
  data->opponentCenter = randomInt(400, 4000);
  data->opponentRight = randomInt(500, 800);

  // Simulate opponent being directly in front every 5 seconds
  if (fmod(t, 5.0) < 0.5) {
    data->opponentCenter = randomInt(3000, 4000); // Strong detection
  }

  // Log some of the data
  ESP_LOGD(TAG, "MiniSumo: Line L:%d R:%d, Opp L:%d C:%d R:%d",
           data->lineSensorLeft, data->lineSensorRight,
           data->opponentLeft, data->opponentCenter, data->opponentRight);
}

// Update Line Follower robot with simulated data
void updateFollowerData(FollowerEndpoints *follower, float t) {
  // Similar to MiniSumo, we're directly accessing private data for testing

  // This is very brittle and should only be used for testing
  const size_t DATA_OFFSET = sizeof(void *) * 10; // Approximate offset of sensorData_ in class

  LineFollowerTestData *data = reinterpret_cast<LineFollowerTestData *>(
      reinterpret_cast<uint8_t *>(follower) + DATA_OFFSET);

  // Simulate line sensors detecting a line (Gaussian pattern moving across sensors)
  for (int i = 0; i < 16; i++) {
    // Position of the center of the line
    float lineCenter = fmod(t * 2, 18) - 1; // Line position moves across the array

    // Distance from this sensor to the line center
    float distance = fabsf(i - lineCenter);

    // Gaussian function to simulate line intensity
    float lineIntensity = expf(-distance * distance / 2.0f);

    // Scale to realistic sensor values (higher = more reflective)
    data->lineSensors[i] = 1000 + static_cast<int>(3000 * (1.0f - lineIntensity));
  }

  // Simulate encoder counts (continuously increasing)
  const float WHEEL_CIRCUMFERENCE = 0.2; // 20 cm wheel circumference
  const int TICKS_PER_REVOLUTION = 360;  // 360 ticks per revolution

  // Baseline speed is 0.5 m/s, varying slightly with time
  float baseSpeed = 0.5f + 0.1f * sinf(t * 0.25f);

  // Speed difference for turning
  float turnFactor = 0.15f * sinf(t * 0.5f);

  // Calculate speeds
  data->leftSpeed = baseSpeed + turnFactor;
  data->rightSpeed = baseSpeed - turnFactor;

  // Calculate encoder counts based on speed and time
  data->leftEncoder = static_cast<int>((data->leftSpeed * t / WHEEL_CIRCUMFERENCE) * TICKS_PER_REVOLUTION);
  data->rightEncoder = static_cast<int>((data->rightSpeed * t / WHEEL_CIRCUMFERENCE) * TICKS_PER_REVOLUTION);

  // Simulate IMU data
  // When robot turns, we see acceleration and gyro readings
  // Accelerometer: measures acceleration (including gravity)
  data->accelX = 0.1f * sinf(t * 2.0f);                        // Small side-to-side acceleration during turns
  data->accelY = 0.05f * (data->leftSpeed - data->rightSpeed); // Forward/backward acceleration during speed changes
  data->accelZ = 9.81f + 0.1f * sinf(t * 10.0f);               // Gravity plus small vibrations

  // Gyroscope: measures angular velocity
  data->gyroX = 0.05f * sinf(t * 5.0f);                      // Small roll oscillation (vibration)
  data->gyroY = 0.03f * sinf(t * 7.0f);                      // Small pitch oscillation (vibration)
  data->gyroZ = 2.0f * (data->leftSpeed - data->rightSpeed); // Yaw rate during turns

  // Log some of the data
  ESP_LOGI(TAG, "Follower: Speed L:%.2f R:%.2f, Gyro Z:%.2f", data->leftSpeed, data->rightSpeed, data->gyroZ);
}
