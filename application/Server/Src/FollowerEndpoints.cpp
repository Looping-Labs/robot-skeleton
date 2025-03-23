#include "FollowerEndpoints.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <algorithm>
#include <sstream>

static const char *TAG = "FollowerEndpoints";

FollowerEndpoints::FollowerEndpoints(WebSocketHandler *wsHandler)
    : RobotEndpoints(wsHandler, "/ws/follower"),
      isStreaming_(false),
      lastBroadcast_(0) {

  // Initialize sensor data with default values
  for (auto &sensor : sensorData_.lineSensors) {
    sensor = 0;
  }

  sensorData_.leftEncoder = 0;
  sensorData_.rightEncoder = 0;
  sensorData_.leftSpeed = 0.0f;
  sensorData_.rightSpeed = 0.0f;
  sensorData_.accelX = 0.0f;
  sensorData_.accelY = 0.0f;
  sensorData_.accelZ = 0.0f;
  sensorData_.gyroX = 0.0f;
  sensorData_.gyroY = 0.0f;
  sensorData_.gyroZ = 0.0f;

  // Initialize PID settings with default values
  pidSettings_.kp = 1.0f;
  pidSettings_.ki = 0.0f;
  pidSettings_.kd = 0.0f;
  pidSettings_.sampleTime = 10.0f; // 10ms
  pidSettings_.targetSpeed = 0.5f; // 0.5 m/s
}

esp_err_t FollowerEndpoints::init() {
  // Register command handlers
  commandHandlers_["getSensorData"] = [this](int fd, const std::map<std::string, std::string> &data) {
    return handleGetSensorData(fd, data);
  };

  commandHandlers_["startStreaming"] = [this](int fd, const std::map<std::string, std::string> &data) {
    return handleStartStreaming(fd, data);
  };

  commandHandlers_["stopStreaming"] = [this](int fd, const std::map<std::string, std::string> &data) {
    return handleStopStreaming(fd, data);
  };

  commandHandlers_["setPIDConstants"] = [this](int fd, const std::map<std::string, std::string> &data) {
    return handleSetPIDConstants(fd, data);
  };

  commandHandlers_["setSpeed"] = [this](int fd, const std::map<std::string, std::string> &data) {
    return handleSetSpeed(fd, data);
  };

  commandHandlers_["setSampleTime"] = [this](int fd, const std::map<std::string, std::string> &data) {
    return handleSetSampleTime(fd, data);
  };

  ESP_LOGI(TAG, "Follower endpoints initialized");
  return ESP_OK;
}

esp_err_t FollowerEndpoints::updateSensors(uint32_t broadcastThreshold) {
  // In a real implementation, you would read the actual sensor values here
  // For now, we'll just use dummy values for demonstration

  // Check if we need to broadcast
  if (isStreaming_) {
    uint32_t currentTime = esp_timer_get_time() / 1000; // Convert to milliseconds

    // Check if enough time has passed since the last broadcast
    if (currentTime - lastBroadcast_ >= broadcastThreshold) {
      // Create and broadcast the sensor data JSON
      std::string jsonData = createSensorDataJson();
      esp_err_t ret = broadcast(jsonData);

      // Update the last broadcast timestamp
      lastBroadcast_ = currentTime;

      return ret;
    }
  }

  return ESP_OK;
}

esp_err_t FollowerEndpoints::handleGetSensorData(int fd, const std::map<std::string, std::string> &data) {
  // Create JSON with current sensor data
  std::string jsonData = createSensorDataJson();

  // Send the response
  return sendResponse(fd, jsonData);
}

esp_err_t FollowerEndpoints::handleStartStreaming(int fd, const std::map<std::string, std::string> &data) {
  // Enable streaming mode
  isStreaming_ = true;

  // Reset the last broadcast timestamp
  lastBroadcast_ = 0;

  // Prepare response
  std::map<std::string, std::string> response;
  response["status"] = "ok";
  response["message"] = "Streaming started";

  // Send response
  return sendResponse(fd, jsonHelper_.createJson(response));
}

esp_err_t FollowerEndpoints::handleStopStreaming(int fd, const std::map<std::string, std::string> &data) {
  // Disable streaming mode
  isStreaming_ = false;

  // Prepare response
  std::map<std::string, std::string> response;
  response["status"] = "ok";
  response["message"] = "Streaming stopped";

  // Send response
  return sendResponse(fd, jsonHelper_.createJson(response));
}

esp_err_t FollowerEndpoints::handleSetPIDConstants(int fd, const std::map<std::string, std::string> &data) {
  // Parse the PID constants from the request
  std::string kpStr = jsonHelper_.getValue(data, "kp", "");
  std::string kiStr = jsonHelper_.getValue(data, "ki", "");
  std::string kdStr = jsonHelper_.getValue(data, "kd", "");

  // Check if we have all the required parameters
  if (kpStr.empty() || kiStr.empty() || kdStr.empty()) {
    // Prepare error response
    std::map<std::string, std::string> errorResponse;
    errorResponse["status"] = "error";
    errorResponse["message"] = "Missing PID constants";

    // Send error response
    return sendResponse(fd, jsonHelper_.createJson(errorResponse));
  }

  // Convert string values to floats
  try {
    float kp = std::stof(kpStr);
    float ki = std::stof(kiStr);
    float kd = std::stof(kdStr);

    // Update the PID settings
    pidSettings_.kp = kp;
    pidSettings_.ki = ki;
    pidSettings_.kd = kd;

    ESP_LOGI(TAG, "Updated PID constants: kp=%.2f, ki=%.2f, kd=%.2f", kp, ki, kd);

    // Prepare success response
    std::map<std::string, std::string> response;
    response["status"] = "ok";
    response["message"] = "PID constants updated";

    // Send response
    return sendResponse(fd, jsonHelper_.createJson(response));
  } catch (const std::exception &e) {
    // Prepare error response
    std::map<std::string, std::string> errorResponse;
    errorResponse["status"] = "error";
    errorResponse["message"] = "Invalid PID constants";

    // Send error response
    return sendResponse(fd, jsonHelper_.createJson(errorResponse));
  }
}

esp_err_t FollowerEndpoints::handleSetSpeed(int fd, const std::map<std::string, std::string> &data) {
  // Parse the speed from the request
  std::string speedStr = jsonHelper_.getValue(data, "speed", "");

  // Check if we have the required parameter
  if (speedStr.empty()) {
    // Prepare error response
    std::map<std::string, std::string> errorResponse;
    errorResponse["status"] = "error";
    errorResponse["message"] = "Missing speed parameter";

    // Send error response
    return sendResponse(fd, jsonHelper_.createJson(errorResponse));
  }

  // Convert string value to float
  try {
    float speed = std::stof(speedStr);

    // Update the target speed
    pidSettings_.targetSpeed = speed;

    ESP_LOGI(TAG, "Updated target speed: %.2f", speed);

    // Prepare success response
    std::map<std::string, std::string> response;
    response["status"] = "ok";
    response["message"] = "Target speed updated";

    // Send response
    return sendResponse(fd, jsonHelper_.createJson(response));
  } catch (const std::exception &e) {
    // Prepare error response
    std::map<std::string, std::string> errorResponse;
    errorResponse["status"] = "error";
    errorResponse["message"] = "Invalid speed value";

    // Send error response
    return sendResponse(fd, jsonHelper_.createJson(errorResponse));
  }
}

esp_err_t FollowerEndpoints::handleSetSampleTime(int fd, const std::map<std::string, std::string> &data) {
  // Parse the sample time from the request
  std::string sampleTimeStr = jsonHelper_.getValue(data, "sampleTime", "");

  // Check if we have the required parameter
  if (sampleTimeStr.empty()) {
    // Prepare error response
    std::map<std::string, std::string> errorResponse;
    errorResponse["status"] = "error";
    errorResponse["message"] = "Missing sampleTime parameter";

    // Send error response
    return sendResponse(fd, jsonHelper_.createJson(errorResponse));
  }

  // Convert string value to float
  try {
    float sampleTime = std::stof(sampleTimeStr);

    // Validate and update the sample time
    if (sampleTime > 0) {
      pidSettings_.sampleTime = sampleTime;

      ESP_LOGI(TAG, "Updated sample time: %.2f ms", sampleTime);

      // Prepare success response
      std::map<std::string, std::string> response;
      response["status"] = "ok";
      response["message"] = "Sample time updated";

      // Send response
      return sendResponse(fd, jsonHelper_.createJson(response));
    } else {
      // Prepare error response
      std::map<std::string, std::string> errorResponse;
      errorResponse["status"] = "error";
      errorResponse["message"] = "Sample time must be greater than 0";

      // Send error response
      return sendResponse(fd, jsonHelper_.createJson(errorResponse));
    }
  } catch (const std::exception &e) {
    // Prepare error response
    std::map<std::string, std::string> errorResponse;
    errorResponse["status"] = "error";
    errorResponse["message"] = "Invalid sample time value";

    // Send error response
    return sendResponse(fd, jsonHelper_.createJson(errorResponse));
  }
}

std::string FollowerEndpoints::createSensorDataJson() {
  // We'll use the createJsonWithBuilder method to create a more complex JSON directly
  return jsonHelper_.createJsonWithBuilder([this](cJSON *root) {
    // Add command type for the client to identify the response
    cJSON_AddStringToObject(root, "type", "sensorData");

    // Create line sensors array
    cJSON *lineSensorsArray = cJSON_AddArrayToObject(root, "lineSensors");
    for (const auto &sensor : sensorData_.lineSensors) {
      cJSON_AddItemToArray(lineSensorsArray, cJSON_CreateNumber(sensor));
    }

    // Add all other sensor data
    cJSON_AddNumberToObject(root, "leftEncoder", sensorData_.leftEncoder);
    cJSON_AddNumberToObject(root, "rightEncoder", sensorData_.rightEncoder);
    cJSON_AddNumberToObject(root, "leftSpeed", sensorData_.leftSpeed);
    cJSON_AddNumberToObject(root, "rightSpeed", sensorData_.rightSpeed);
    cJSON_AddNumberToObject(root, "accelX", sensorData_.accelX);
    cJSON_AddNumberToObject(root, "accelY", sensorData_.accelY);
    cJSON_AddNumberToObject(root, "accelZ", sensorData_.accelZ);
    cJSON_AddNumberToObject(root, "gyroX", sensorData_.gyroX);
    cJSON_AddNumberToObject(root, "gyroY", sensorData_.gyroY);
    cJSON_AddNumberToObject(root, "gyroZ", sensorData_.gyroZ);

    // Add timestamp (in milliseconds)
    cJSON_AddNumberToObject(root, "timestamp", esp_timer_get_time() / 1000);
  });
}

std::string FollowerEndpoints::createPIDSettingsJson() {
  return jsonHelper_.createJsonWithBuilder([this](cJSON *root) {
    // Add command type for the client to identify the response
    cJSON_AddStringToObject(root, "type", "pidSettings");

    // Add the PID settings
    cJSON_AddNumberToObject(root, "kp", pidSettings_.kp);
    cJSON_AddNumberToObject(root, "ki", pidSettings_.ki);
    cJSON_AddNumberToObject(root, "kd", pidSettings_.kd);
    cJSON_AddNumberToObject(root, "sampleTime", pidSettings_.sampleTime);
    cJSON_AddNumberToObject(root, "targetSpeed", pidSettings_.targetSpeed);
  });
}