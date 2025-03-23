#pragma once

#include "JsonHelper.h"
#include "WebSocketHandler.h"
#include <functional>
#include <map>
#include <memory>
#include <string>

// Forward declaration to handle circular dependency
struct WebSocketMessage;

/**
 * @brief Base class for robot endpoints
 *
 * This abstract class defines the interface for all robot-specific endpoint handlers
 */
class RobotEndpoints {
public:
  /**
   * @brief Constructs a new Robot Endpoints object
   *
   * @param wsHandler WebSocketHandler to use for communication
   * @param basePath Base path for this robot's endpoints
   */
  RobotEndpoints(WebSocketHandler *wsHandler, const std::string &basePath);

  /**
   * @brief Destroys the Robot Endpoints object
   */
  virtual ~RobotEndpoints() = default;

  /**
   * @brief Initializes the endpoints
   *
   * @return esp_err_t ESP_OK on success
   */
  virtual esp_err_t init() = 0;

  /**
   * @brief Handles incoming WebSocket messages
   *
   * @param message The WebSocket message
   * @return esp_err_t ESP_OK on success
   */
  virtual esp_err_t handleMessage(const WebSocketMessage &message);

  /**
   * @brief Gets the base path for this endpoint
   *
   * @return const std::string& The base path
   */
  const std::string &getBasePath() const;

  /**
   * @brief Broadcasts a message to all clients connected to this endpoint
   *
   * @param message The message to broadcast
   * @param type The WebSocket frame type
   * @return esp_err_t ESP_OK on success
   */
  esp_err_t broadcast(const std::string &message, httpd_ws_type_t type = HTTPD_WS_TYPE_TEXT);

  /**
   * @brief Sends a response to a specific client
   *
   * @param fd The client socket descriptor
   * @param message The message to send
   * @param type The WebSocket frame type
   * @return esp_err_t ESP_OK on success
   */
  esp_err_t sendResponse(int fd, const std::string &message, httpd_ws_type_t type = HTTPD_WS_TYPE_TEXT);

protected:
  /**
   * @brief Parses a command from a message and routes it to the appropriate handler
   *
   * @param message The WebSocket message
   * @return esp_err_t ESP_OK on success
   */
  virtual esp_err_t parseCommand(const WebSocketMessage &message);

  /**
   * @brief Type definition for command handlers
   */
  using CommandHandler = std::function<esp_err_t(int fd, const std::map<std::string, std::string> &)>;

  WebSocketHandler *wsHandler_;                           // WebSocket handler for communication
  std::string basePath_;                                  // Base path for this endpoint
  std::map<std::string, CommandHandler> commandHandlers_; // Command handlers
  JsonHelper jsonHelper_;                                 // JSON helper for parsing/creating messages
};