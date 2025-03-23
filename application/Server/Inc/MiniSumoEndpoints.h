#pragma once

#include "RobotEndpoints.h"
#include <functional>
#include <map>
#include <string>

/**
 * @brief Handles WebSocket endpoints for the Mini-Sumo robot
 *
 * This class implements the specific endpoints and functionality
 * for the Mini-Sumo robot with two infrared sensors for line detection
 * and three infrared sensors for opponent detection.
 */
class MiniSumoEndpoints : public RobotEndpoints {
public:
  /**
   * @brief Constructs a new Mini Sumo Endpoints object
   *
   * @param wsHandler WebSocket handler for communication
   */
  MiniSumoEndpoints(WebSocketHandler *wsHandler);

  /**
   * @brief Initializes the Mini-Sumo endpoints
   *
   * Registers all command handlers for this robot type
   *
   * @return esp_err_t ESP_OK on success
   */
  esp_err_t init() override;

  /**
   * @brief Updates the sensor values and broadcasts to clients if needed
   *
   * This method should be called periodically to update the sensor values
   * and broadcast them to connected clients
   *
   * @param broadcastThreshold Minimum time between broadcasts in milliseconds
   * @return esp_err_t ESP_OK on success
   */
  esp_err_t updateSensors(uint32_t broadcastThreshold = 100);

private:
  /**
   * @brief Handles the "getSensorData" command
   *
   * @param fd Client socket descriptor
   * @param data Parsed JSON data
   * @return esp_err_t ESP_OK on success
   */
  esp_err_t handleGetSensorData(int fd, const std::map<std::string, std::string> &data);

  /**
   * @brief Handles the "startStreaming" command
   *
   * @param fd Client socket descriptor
   * @param data Parsed JSON data
   * @return esp_err_t ESP_OK on success
   */
  esp_err_t handleStartStreaming(int fd, const std::map<std::string, std::string> &data);

  /**
   * @brief Handles the "stopStreaming" command
   *
   * @param fd Client socket descriptor
   * @param data Parsed JSON data
   * @return esp_err_t ESP_OK on success
   */
  esp_err_t handleStopStreaming(int fd, const std::map<std::string, std::string> &data);

  /**
   * @brief Creates a JSON object with the current sensor data
   *
   * @return std::string JSON string with sensor data
   */
  std::string createSensorDataJson();

  // Mini-Sumo sensor data
  struct {
    int lineSensorLeft;  // Line sensor left value
    int lineSensorRight; // Line sensor right value
    int opponentLeft;    // Opponent sensor left value
    int opponentCenter;  // Opponent sensor center value
    int opponentRight;   // Opponent sensor right value
  } sensorData_;

  bool isStreaming_;       // Flag for streaming mode
  uint32_t lastBroadcast_; // Timestamp of last broadcast
};