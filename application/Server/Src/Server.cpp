#include "Server.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

static const char *TAG = "RobotServer";

// Default SSID and password if not provided
#ifndef CONFIG_WIFI_SSID
#define CONFIG_WIFI_SSID "robot_network"
#endif

#ifndef CONFIG_WIFI_PASSWORD
#define CONFIG_WIFI_PASSWORD "robot123"
#endif

#ifndef CONFIG_WEBSOCKET_SERVER_PORT
#define CONFIG_WEBSOCKET_SERVER_PORT 80
#endif

// Structure to hold server instance for event callbacks
typedef struct {
  Server *server;
} server_context_t;

// Global context for event callbacks
static server_context_t serverContext = {nullptr};

Server::Server(const std::string &ssid, const std::string &password, int port)
    : ssid_(ssid.empty() ? CONFIG_WIFI_SSID : ssid),
      password_(password.empty() ? CONFIG_WIFI_PASSWORD : password),
      port_(port == 0 ? CONFIG_WEBSOCKET_SERVER_PORT : port),
      server_(nullptr),
      isInitialized_(false),
      isRunning_(false),
      isWiFiConnected_(false) {
}

Server::~Server() {
  stop();
}

esp_err_t Server::init() {
  ESP_LOGI(TAG, "Initializing server");

  // Initialize NVS flash
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  // Initialize TCP/IP stack and event loop
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());

  // Set server context for callbacks
  serverContext.server = this;

  // Initialize WiFi
  ret = initWiFi();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize WiFi");
    return ret;
  }

  // Create the WebSocket handler
  wsHandler_ = std::make_unique<WebSocketHandler>();

  // Create the endpoint handlers
  miniSumoEndpoints_ = std::make_unique<MiniSumoEndpoints>(wsHandler_.get());
  followerEndpoints_ = std::make_unique<FollowerEndpoints>(wsHandler_.get());

  isInitialized_ = true;

  ESP_LOGI(TAG, "Server initialized");
  return ESP_OK;
}

esp_err_t Server::start() {
  if (!isInitialized_) {
    ESP_LOGE(TAG, "Server not initialized. Call init() first.");
    return ESP_FAIL;
  }

  if (isRunning_) {
    ESP_LOGW(TAG, "Server already running");
    return ESP_OK;
  }

  // Start the HTTP server
  esp_err_t ret = startHttpServer();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to start HTTP server");
    return ret;
  }

  // Initialize the WebSocket handler with the server
  ret = wsHandler_->init(server_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize WebSocket handler");
    return ret;
  }

  // Initialize and register the endpoint handlers
  ret = miniSumoEndpoints_->init();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize Mini-Sumo endpoints");
    return ret;
  }

  ret = followerEndpoints_->init();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize Follower endpoints");
    return ret;
  }

  // Register the endpoints with the WebSocket handler
  ret = wsHandler_->registerEndpoint("/ws/minisumo", miniSumoEndpoints_.get());
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to register Mini-Sumo endpoint");
    return ret;
  }

  ret = wsHandler_->registerEndpoint("/ws/follower", followerEndpoints_.get());
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to register Follower endpoint");
    return ret;
  }

  isRunning_ = true;

  ESP_LOGI(TAG, "Server started on port %d", port_);
  return ESP_OK;
}

esp_err_t Server::stop() {
  if (!isRunning_) {
    return ESP_OK;
  }

  // Stop the HTTP server
  if (server_ != nullptr) {
    esp_err_t ret = httpd_stop(server_);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "Failed to stop HTTP server");
      return ret;
    }
    server_ = nullptr;
  }

  isRunning_ = false;

  ESP_LOGI(TAG, "Server stopped");
  return ESP_OK;
}

esp_err_t Server::update() {
  if (!isRunning_) {
    return ESP_OK;
  }

  // Update the endpoint handlers
  esp_err_t ret = miniSumoEndpoints_->updateSensors();
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "Error updating Mini-Sumo endpoints");
  }

  ret = followerEndpoints_->updateSensors();
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "Error updating Follower endpoints");
  }

  return ESP_OK;
}

MiniSumoEndpoints *Server::getMiniSumoEndpoints() {
  return miniSumoEndpoints_.get();
}

FollowerEndpoints *Server::getFollowerEndpoints() {
  return followerEndpoints_.get();
}

esp_err_t Server::initWiFi() {
  ESP_LOGI(TAG, "Initializing WiFi with SSID: %s", ssid_.c_str());

  // Create the default WiFi station
  esp_netif_create_default_wifi_sta();

  // Initialize WiFi with default config
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  // Register event handlers
  ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &eventHandler, &serverContext));
  ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &eventHandler, &serverContext));

  // Configure WiFi station
  wifi_config_t wifi_config = {};
  strncpy((char *)wifi_config.sta.ssid, ssid_.c_str(), sizeof(wifi_config.sta.ssid) - 1);
  strncpy((char *)wifi_config.sta.password, password_.c_str(), sizeof(wifi_config.sta.password) - 1);

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));

  // Start WiFi
  ESP_ERROR_CHECK(esp_wifi_start());

  ESP_LOGI(TAG, "Connecting to WiFi network...");

  // Connect to the AP
  ESP_ERROR_CHECK(esp_wifi_connect());

  return ESP_OK;
}

void Server::eventHandler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data) {
  server_context_t *context = static_cast<server_context_t *>(arg);
  Server *server = context->server;

  if (event_base == WIFI_EVENT) {
    if (event_id == WIFI_EVENT_STA_START) {
      ESP_LOGI(TAG, "WiFi station started");
    } else if (event_id == WIFI_EVENT_STA_CONNECTED) {
      ESP_LOGI(TAG, "Connected to WiFi network");
    } else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
      ESP_LOGW(TAG, "Disconnected from WiFi network");

      if (server != nullptr) {
        server->isWiFiConnected_ = false;

        // Try to reconnect
        esp_wifi_connect();
      }
    }
  } else if (event_base == IP_EVENT) {
    if (event_id == IP_EVENT_STA_GOT_IP) {
      ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
      ESP_LOGI(TAG, "Got IP address: " IPSTR, IP2STR(&event->ip_info.ip));

      if (server != nullptr) {
        server->isWiFiConnected_ = true;
      }
    }
  }
}

esp_err_t Server::startHttpServer() {
  // Configure the server
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = port_;
  config.max_uri_handlers = 10;
  config.stack_size = 8192;

  // Start the server
  esp_err_t ret = httpd_start(&server_, &config);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to start HTTP server: %d", ret);
    return ret;
  }

  ESP_LOGI(TAG, "HTTP server started");
  return ESP_OK;
}