#include "RobotEndpoints.h"
#include "esp_log.h"

static const char *TAG = "RobotEndpoints";

RobotEndpoints::RobotEndpoints(WebSocketHandler *wsHandler, const std::string &basePath)
    : wsHandler_(wsHandler), basePath_(basePath) {
}

esp_err_t RobotEndpoints::handleMessage(const WebSocketMessage &message) {
  // Only handle text messages
  if (message.type != HTTPD_WS_TYPE_TEXT) {
    ESP_LOGW(TAG, "Received non-text message, ignoring");
    return ESP_OK;
  }

  ESP_LOGI(TAG, "Received message on %s: %s", basePath_.c_str(), message.payload.c_str());

  // Parse the message and route to appropriate handler
  return parseCommand(message);
}

const std::string &RobotEndpoints::getBasePath() const {
  return basePath_;
}

esp_err_t RobotEndpoints::broadcast(const std::string &message, httpd_ws_type_t type) {
  if (wsHandler_ == nullptr) {
    ESP_LOGE(TAG, "WebSocketHandler is null");
    return ESP_FAIL;
  }

  return wsHandler_->broadcastToPath(basePath_, message, type);
}

esp_err_t RobotEndpoints::sendResponse(int fd, const std::string &message, httpd_ws_type_t type) {
  if (wsHandler_ == nullptr) {
    ESP_LOGE(TAG, "WebSocketHandler is null");
    return ESP_FAIL;
  }

  return wsHandler_->sendFrame(fd, message, type);
}

esp_err_t RobotEndpoints::parseCommand(const WebSocketMessage &message) {
  // Parse the JSON message
  std::map<std::string, std::string> jsonData;
  esp_err_t ret = jsonHelper_.parse(message.payload, jsonData);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to parse JSON: %s", message.payload.c_str());

    // Send error response
    std::map<std::string, std::string> errorResponse;
    errorResponse["error"] = "Invalid JSON format";
    sendResponse(message.fd, jsonHelper_.createJson(errorResponse));

    return ret;
  }

  // Get the command from the JSON
  std::string command = jsonHelper_.getValue(jsonData, "command", "");
  if (command.empty()) {
    ESP_LOGW(TAG, "No command specified in message");

    // Send error response
    std::map<std::string, std::string> errorResponse;
    errorResponse["error"] = "No command specified";
    sendResponse(message.fd, jsonHelper_.createJson(errorResponse));

    return ESP_OK;
  }

  // Find the handler for this command
  auto handlerIt = commandHandlers_.find(command);
  if (handlerIt == commandHandlers_.end()) {
    ESP_LOGW(TAG, "Unknown command: %s", command.c_str());

    // Send error response
    std::map<std::string, std::string> errorResponse;
    errorResponse["error"] = "Unknown command: " + command;
    sendResponse(message.fd, jsonHelper_.createJson(errorResponse));

    return ESP_OK;
  }

  // Call the handler
  return handlerIt->second(message.fd, jsonData);
}