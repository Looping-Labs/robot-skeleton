#pragma once

#include "FollowerEndpoints.h"
#include "MiniSumoEndpoints.h"
#include "WebSocketHandler.h"
#include "esp_http_server.h"
#include <memory>
#include <string>

/**
 * @brief Main server class for the robot WebSocket API
 *
 * This class manages the HTTP server and WebSocket connections,
 * handling initialization, WiFi connection, and routing.
 */
class Server {
public:
  /**
   * @brief Constructs a new Server object
   *
   * @param ssid WiFi SSID to connect to
   * @param password WiFi password
   * @param port Server port to listen on
   */
  Server(const std::string &ssid = "",
         const std::string &password = "",
         int port = 80);

  /**
   * @brief Destroys the Server object
   * Stops the server and cleans up resources
   */
  ~Server();

  /**
   * @brief Initializes the server
   *
   * @return esp_err_t ESP_OK on success
   */
  esp_err_t init();

  /**
   * @brief Starts the server
   *
   * @return esp_err_t ESP_OK on success
   */
  esp_err_t start();

  /**
   * @brief Stops the server
   *
   * @return esp_err_t ESP_OK on success
   */
  esp_err_t stop();

  /**
   * @brief Updates all robot endpoints
   *
   * This method should be called in the main loop to update sensor data
   * and handle periodic tasks.
   *
   * @return esp_err_t ESP_OK on success
   */
  esp_err_t update();

  /**
   * @brief Gets the Mini-Sumo endpoints object
   *
   * @return MiniSumoEndpoints* Pointer to Mini-Sumo endpoints
   */
  MiniSumoEndpoints *getMiniSumoEndpoints();

  /**
   * @brief Gets the Follower endpoints object
   *
   * @return FollowerEndpoints* Pointer to Follower endpoints
   */
  FollowerEndpoints *getFollowerEndpoints();

private:
  /**
   * @brief Initializes WiFi connection
   *
   * @return esp_err_t ESP_OK on success
   */
  esp_err_t initWiFi();

  /**
   * @brief Event handler for WiFi and IP events
   *
   * @param arg User argument
   * @param event_base Event base
   * @param event_id Event ID
   * @param event_data Event data
   */
  static void eventHandler(void *arg, esp_event_base_t event_base,
                           int32_t event_id, void *event_data);

  /**
   * @brief Starts the HTTP server
   *
   * @return esp_err_t ESP_OK on success
   */
  esp_err_t startHttpServer();

  std::string ssid_;     // WiFi SSID
  std::string password_; // WiFi password
  int port_;             // Server port

  httpd_handle_t server_;                                // HTTP server handle
  std::unique_ptr<WebSocketHandler> wsHandler_;          // WebSocket handler
  std::unique_ptr<MiniSumoEndpoints> miniSumoEndpoints_; // Mini-Sumo endpoints
  std::unique_ptr<FollowerEndpoints> followerEndpoints_; // Follower endpoints

  bool isInitialized_;   // Flag indicating if server is initialized
  bool isRunning_;       // Flag indicating if server is running
  bool isWiFiConnected_; // Flag indicating if WiFi is connected
};