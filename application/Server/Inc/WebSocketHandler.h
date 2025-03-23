#pragma once

#include "JsonHelper.h"
#include "esp_http_server.h"
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

// Forward declaration for circular dependency
class RobotEndpoints;

/**
 * @brief WebSocket message with metadata
 */
struct WebSocketMessage {
  int fd;               // Socket file descriptor
  std::string payload;  // Message content
  httpd_ws_type_t type; // Message type (text/binary)
};

/**
 * @brief Class to handle WebSocket connections and message routing
 */
class WebSocketHandler {
public:
  using MessageCallback = std::function<void(const WebSocketMessage &)>;

  /**
   * @brief Constructs a new WebSocket Handler
   */
  WebSocketHandler();

  /**
   * @brief Initializes the WebSocket handler
   *
   * @param server The HTTP server handle
   * @return esp_err_t ESP_OK on success
   */
  esp_err_t init(httpd_handle_t server);

  /**
   * @brief Registers a endpoint path with a handler
   *
   * @param path The URI path to register
   * @param endpoints Pointer to the endpoints object that will handle requests to this path
   * @return esp_err_t ESP_OK on success
   */
  esp_err_t registerEndpoint(const std::string &path, RobotEndpoints *endpoints);

  /**
   * @brief Handles incoming WebSocket frame
   *
   * This is the main entry point for WebSocket messages
   *
   * @param req The HTTP request containing the WebSocket frame
   * @return esp_err_t ESP_OK on success
   */
  esp_err_t handleFrame(httpd_req_t *req);

  /**
   * @brief Sends a WebSocket frame to a client
   *
   * @param fd Socket file descriptor for the client
   * @param message Message to send
   * @param type Type of WebSocket frame (text/binary)
   * @return esp_err_t ESP_OK on success
   */
  esp_err_t sendFrame(int fd, const std::string &message, httpd_ws_type_t type = HTTPD_WS_TYPE_TEXT);

  /**
   * @brief Broadcasts a message to all connected clients
   *
   * @param message Message to broadcast
   * @param type Type of WebSocket frame (text/binary)
   * @return esp_err_t ESP_OK on success
   */
  esp_err_t broadcast(const std::string &message, httpd_ws_type_t type = HTTPD_WS_TYPE_TEXT);

  /**
   * @brief Broadcasts a message to clients with a specific path prefix
   *
   * @param pathPrefix Path prefix to match
   * @param message Message to broadcast
   * @param type Type of WebSocket frame (text/binary)
   * @return esp_err_t ESP_OK on success
   */
  esp_err_t broadcastToPath(const std::string &pathPrefix, const std::string &message, httpd_ws_type_t type = HTTPD_WS_TYPE_TEXT);

  /**
   * @brief Gets the JSON helper instance
   *
   * @return JsonHelper& Reference to the JSON helper
   */
  JsonHelper &getJsonHelper();

  /**
   * @brief Processes a client disconnection
   *
   * @param fd Socket file descriptor of the disconnected client
   */
  void handleClientDisconnect(int fd);

private:
  /**
   * @brief Asynchronously sends a WebSocket frame
   *
   * @param arg Async response argument
   */
  static void asyncSendFrame(void *arg);

  /**
   * @brief Structure for async response
   */
  struct AsyncSendArg {
    httpd_handle_t hd;    // Server handle
    int fd;               // Socket fd
    std::string payload;  // Message to send
    httpd_ws_type_t type; // Message type
  };

  /**
   * @brief Client connection info
   */
  struct ClientInfo {
    int fd;           // Socket descriptor
    std::string path; // URI path
  };

  httpd_handle_t server_;                          // HTTP server handle
  std::map<std::string, RobotEndpoints *> routes_; // Registered routes
  std::vector<ClientInfo> clients_;                // Connected clients
  std::mutex clientsMutex_;                        // Mutex for client list access
  JsonHelper jsonHelper_;                          // JSON utilities
};