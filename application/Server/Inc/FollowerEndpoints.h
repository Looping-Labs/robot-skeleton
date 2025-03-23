#pragma once

#include "RobotEndpoints.h"
#include <array>
#include <functional>
#include <map>
#include <string>

/**
 * @brief Handles WebSocket endpoints for the Line Follower robot
 *
 * This class implements the specific endpoints and functionality
 * for the Line Follower robot with 16 infrared sensors,
 * quadrature encoders, and MPU for acceleration.
 */
class FollowerEndpoints : public RobotEndpoints {
public:
  /**
   * @brief Constructs a new Follower Endpoints object
   *
   * @param wsHandler WebSocket handler for communication
   */
  FollowerEndpoints(WebSocketHandler *wsHandler);

  /**
   * @brief Initializes the Follower endpoints
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
   * @brief Handles the "setPIDConstants" command
   *
   * @param fd Client socket descriptor
   * @param data Parsed JSON data
   * @return esp_err_t ESP_OK on success
   */
  esp_err_t handleSetPIDConstants(int fd, const std::map<std::string, std::string> &data);

  /**
   * @brief Handles the "setSpeed" command
   *
   * @param fd Client socket descriptor
   * @param data Parsed JSON data
   * @return esp_err_t ESP_OK on success
   */
  esp_err_t handleSetSpeed(int fd, const std::map<std::string, std::string> &data);

  /**
   * @brief Handles the "setSampleTime" command
   *
   * @param fd Client socket descriptor
   * @param data Parsed JSON data
   * @return esp_err_t ESP_OK on success
   */
  esp_err_t handleSetSampleTime(int fd, const std::map<std::string, std::string> &data);

  /**
   * @brief Creates a JSON object with the current sensor data
   *
   * @return std::string JSON string with sensor data
   */
  std::string createSensorDataJson();

  /**
   * @brief Creates a JSON object with the current PID settings
   *
   * @return std::string JSON string with PID settings
   */
  std::string createPIDSettingsJson();

  // Follower sensor data
  struct {
    std::array<int, 16> lineSensors; // Line sensor array (16 sensors)
    int leftEncoder;                 // Left motor encoder count
    int rightEncoder;                // Right motor encoder count
    float leftSpeed;                 // Left motor speed
    float rightSpeed;                // Right motor speed
    float accelX;                    // X-axis acceleration
    float accelY;                    // Y-axis acceleration
    float accelZ;                    // Z-axis acceleration
    float gyroX;                     // X-axis angular velocity
    float gyroY;                     // Y-axis angular velocity
    float gyroZ;                     // Z-axis angular velocity
  } sensorData_;

  // PID controller settings
  struct {
    float kp;          // Proportional constant
    float ki;          // Integral constant
    float kd;          // Derivative constant
    float sampleTime;  // PID sample time in milliseconds
    float targetSpeed; // Target speed
  } pidSettings_;

  bool isStreaming_;       // Flag for streaming mode
  uint32_t lastBroadcast_; // Timestamp of last broadcast
};