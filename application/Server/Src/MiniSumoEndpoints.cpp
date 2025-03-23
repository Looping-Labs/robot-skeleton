#include "MiniSumoEndpoints.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "MiniSumoEndpoints";

MiniSumoEndpoints::MiniSumoEndpoints(WebSocketHandler *wsHandler)
    : RobotEndpoints(wsHandler, "/ws/minisumo"),
      isStreaming_(false),
      lastBroadcast_(0) {

  // Initialize sensor data with default values
  sensorData_.lineSensorLeft = 0;
  sensorData_.lineSensorRight = 0;
  sensorData_.opponentLeft = 0;
  sensorData_.opponentCenter = 0;
  sensorData_.opponentRight = 0;
}

esp_err_t MiniSumoEndpoints::init() {
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

  ESP_LOGI(TAG, "MiniSumo endpoints initialized");
  return ESP_OK;
}

esp_err_t MiniSumoEndpoints::updateSensors(uint32_t broadcastThreshold) {
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

esp_err_t MiniSumoEndpoints::handleGetSensorData(int fd, const std::map<std::string, std::string> &data) {
  // Create JSON with current sensor data
  std::string jsonData = createSensorDataJson();

  // Send the response
  return sendResponse(fd, jsonData);
}

esp_err_t MiniSumoEndpoints::handleStartStreaming(int fd, const std::map<std::string, std::string> &data) {
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

esp_err_t MiniSumoEndpoints::handleStopStreaming(int fd, const std::map<std::string, std::string> &data) {
  // Disable streaming mode
  isStreaming_ = false;

  // Prepare response
  std::map<std::string, std::string> response;
  response["status"] = "ok";
  response["message"] = "Streaming stopped";

  // Send response
  return sendResponse(fd, jsonHelper_.createJson(response));
}

std::string MiniSumoEndpoints::createSensorDataJson() {
  // Using the createJsonWithBuilder method for consistent approach
  return jsonHelper_.createJsonWithBuilder([this](cJSON *root) {
    // Add command type for the client to identify the response
    cJSON_AddStringToObject(root, "type", "sensorData");

    // Add the sensor data
    cJSON_AddNumberToObject(root, "lineSensorLeft", sensorData_.lineSensorLeft);
    cJSON_AddNumberToObject(root, "lineSensorRight", sensorData_.lineSensorRight);
    cJSON_AddNumberToObject(root, "opponentLeft", sensorData_.opponentLeft);
    cJSON_AddNumberToObject(root, "opponentCenter", sensorData_.opponentCenter);
    cJSON_AddNumberToObject(root, "opponentRight", sensorData_.opponentRight);

    // Add timestamp (in milliseconds)
    cJSON_AddNumberToObject(root, "timestamp", esp_timer_get_time() / 1000);
  });
}