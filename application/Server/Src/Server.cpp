#include "Server.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

static const char *TAG = "RobotServer";

// Default SSID and password if not provided
#ifndef CONFIG_WIFI_SSID
#define CONFIG_WIFI_SSID "CCTET"
#endif

#ifndef CONFIG_WIFI_PASSWORD
#define CONFIG_WIFI_PASSWORD "Guapi*2025*CCTET*"
#endif

#ifndef CONFIG_WEBSOCKET_SERVER_PORT
#define CONFIG_WEBSOCKET_SERVER_PORT 6969
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
      isWiFiConnected_(false),
      hasApInfo_(false) {
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

  // Update WiFi status
  updateWifiStatus();

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

void Server::eventHandler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
  server_context_t *context = static_cast<server_context_t *>(arg);
  Server *server = context->server;

  if (server == nullptr) {
    ESP_LOGE(TAG, "Server context is null in event handler");
    return;
  }

  if (event_base == WIFI_EVENT) {
    if (event_id == WIFI_EVENT_STA_START) {
      ESP_LOGI(TAG, "WiFi station started");
      // Reset AP info when starting
      server->hasApInfo_ = false;
    } else if (event_id == WIFI_EVENT_STA_CONNECTED) {
      wifi_event_sta_connected_t *event = (wifi_event_sta_connected_t *)event_data;
      ESP_LOGI(TAG, "Connected to WiFi network: %s (channel: %d)",
               event->ssid, event->channel);

      // We're connected to the AP, but we need an IP address before we can communicate
      server->hasApInfo_ = false;

      // Try to get AP info
      if (esp_wifi_sta_get_ap_info(&server->apInfo_) == ESP_OK) {
        server->hasApInfo_ = true;
        ESP_LOGI(TAG, "WiFi signal strength: %d dBm", server->apInfo_.rssi);
      }
    } else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
      wifi_event_sta_disconnected_t *event = (wifi_event_sta_disconnected_t *)event_data;

      // Convert reason code to string for more informative log
      const char *reason_str;
      switch (event->reason) {
      case WIFI_REASON_UNSPECIFIED:
        reason_str = "Unspecified";
        break;
      case WIFI_REASON_AUTH_EXPIRE:
        reason_str = "Auth expired";
        break;
      case WIFI_REASON_AUTH_LEAVE:
        reason_str = "Auth leave";
        break;
      case WIFI_REASON_ASSOC_EXPIRE:
        reason_str = "Association expired";
        break;
      case WIFI_REASON_ASSOC_TOOMANY:
        reason_str = "Too many associations";
        break;
      case WIFI_REASON_NOT_AUTHED:
        reason_str = "Not authenticated";
        break;
      case WIFI_REASON_NOT_ASSOCED:
        reason_str = "Not associated";
        break;
      case WIFI_REASON_ASSOC_LEAVE:
        reason_str = "Association leave";
        break;
      case WIFI_REASON_ASSOC_NOT_AUTHED:
        reason_str = "Association not authenticated";
        break;
      case WIFI_REASON_DISASSOC_PWRCAP_BAD:
        reason_str = "Disassociated due to bad power capability";
        break;
      case WIFI_REASON_DISASSOC_SUPCHAN_BAD:
        reason_str = "Disassociated due to bad supported channels";
        break;
      case WIFI_REASON_BSS_TRANSITION_DISASSOC:
        reason_str = "BSS transition disassociation";
        break;
      case WIFI_REASON_IE_INVALID:
        reason_str = "Invalid IE";
        break;
      case WIFI_REASON_MIC_FAILURE:
        reason_str = "MIC failure";
        break;
      case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT:
        reason_str = "4-way handshake timeout";
        break;
      case WIFI_REASON_GROUP_KEY_UPDATE_TIMEOUT:
        reason_str = "Group key update timeout";
        break;
      case WIFI_REASON_IE_IN_4WAY_DIFFERS:
        reason_str = "IE in 4-way handshake differs";
        break;
      case WIFI_REASON_GROUP_CIPHER_INVALID:
        reason_str = "Invalid group cipher";
        break;
      case WIFI_REASON_PAIRWISE_CIPHER_INVALID:
        reason_str = "Invalid pairwise cipher";
        break;
      case WIFI_REASON_AKMP_INVALID:
        reason_str = "Invalid AKMP";
        break;
      case WIFI_REASON_UNSUPP_RSN_IE_VERSION:
        reason_str = "Unsupported RSN IE version";
        break;
      case WIFI_REASON_INVALID_RSN_IE_CAP:
        reason_str = "Invalid RSN IE capability";
        break;
      case WIFI_REASON_802_1X_AUTH_FAILED:
        reason_str = "802.1X authentication failed";
        break;
      case WIFI_REASON_CIPHER_SUITE_REJECTED:
        reason_str = "Cipher suite rejected";
        break;
      case WIFI_REASON_BEACON_TIMEOUT:
        reason_str = "Beacon timeout";
        break;
      case WIFI_REASON_NO_AP_FOUND:
        reason_str = "No AP found";
        break;
      case WIFI_REASON_AUTH_FAIL:
        reason_str = "Authentication failed";
        break;
      case WIFI_REASON_ASSOC_FAIL:
        reason_str = "Association failed";
        break;
      case WIFI_REASON_HANDSHAKE_TIMEOUT:
        reason_str = "Handshake timeout";
        break;
      case WIFI_REASON_CONNECTION_FAIL:
        reason_str = "Connection failed";
        break;
      case WIFI_REASON_AP_TSF_RESET:
        reason_str = "AP TSF reset";
        break;
      default:
        reason_str = "Unknown reason";
        break;
      }

      ESP_LOGW(TAG, "Disconnected from WiFi. Reason: %s (%d)", reason_str, event->reason);

      // Reset connection status
      server->isWiFiConnected_ = false;
      server->hasApInfo_ = false;

      // Try to reconnect
      ESP_LOGI(TAG, "Attempting to reconnect to WiFi...");
      esp_wifi_connect();
    }
  } else if (event_base == IP_EVENT) {
    if (event_id == IP_EVENT_STA_GOT_IP) {
      ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
      ESP_LOGI(TAG, "Got IP address: " IPSTR, IP2STR(&event->ip_info.ip));
      ESP_LOGI(TAG, "Netmask: " IPSTR, IP2STR(&event->ip_info.netmask));
      ESP_LOGI(TAG, "Gateway: " IPSTR, IP2STR(&event->ip_info.gw));

      // Update connection status
      server->isWiFiConnected_ = true;

      // Try to get AP info again if we don't have it yet
      if (!server->hasApInfo_) {
        if (esp_wifi_sta_get_ap_info(&server->apInfo_) == ESP_OK) {
          server->hasApInfo_ = true;
          ESP_LOGI(TAG, "Connected to: %s, Signal: %d dBm",
                   server->apInfo_.ssid, server->apInfo_.rssi);
        }
      }
    } else if (event_id == IP_EVENT_STA_LOST_IP) {
      ESP_LOGW(TAG, "Lost IP address");
      server->isWiFiConnected_ = false;
    }
  }
}

// Now implement the new methods for WiFi information

std::string Server::getIpAddress() const {
  if (!isWiFiConnected_) {
    return "";
  }

  esp_netif_ip_info_t ip_info;
  esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");

  if (netif != NULL && esp_netif_get_ip_info(netif, &ip_info) == ESP_OK) {
    char ip_str[16]; // Maximum length of "xxx.xxx.xxx.xxx" + null terminator
    sprintf(ip_str, IPSTR, IP2STR(&ip_info.ip));
    return std::string(ip_str);
  }

  return "";
}

int8_t Server::getWifiSignalStrength() const {
  if (!isWiFiConnected_ || !hasApInfo_) {
    return 0;
  }

  return apInfo_.rssi;
}

std::string Server::getConnectedSSID() const {
  if (!isWiFiConnected_ || !hasApInfo_) {
    return "";
  }

  // Convert SSID bytes to string
  // Note: SSID is not always null-terminated in the struct
  return std::string(reinterpret_cast<const char *>(apInfo_.ssid),
                     strnlen(reinterpret_cast<const char *>(apInfo_.ssid), sizeof(apInfo_.ssid)));
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

void Server::updateWifiStatus() {
  static uint32_t lastStatusCheck = 0;
  static const uint32_t STATUS_CHECK_INTERVAL = 10000; // Check every 10 seconds

  uint32_t currentTime = esp_timer_get_time() / 1000;

  // Only check periodically to avoid excessive API calls
  if (currentTime - lastStatusCheck >= STATUS_CHECK_INTERVAL) {
    lastStatusCheck = currentTime;

    if (isWiFiConnected_) {
      // Refresh AP info
      wifi_ap_record_t newApInfo;
      if (esp_wifi_sta_get_ap_info(&newApInfo) == ESP_OK) {
        // Check if signal strength has changed significantly
        if (!hasApInfo_ || abs(newApInfo.rssi - apInfo_.rssi) > 5) {
          ESP_LOGI(TAG, "WiFi signal strength: %d dBm", newApInfo.rssi);
        }
        apInfo_ = newApInfo;
        hasApInfo_ = true;
      } else {
        // If we can't get AP info but we're supposedly connected, something's wrong
        ESP_LOGW(TAG, "Failed to get AP info despite WiFi being connected");
      }

      // Check if we still have an IP address
      esp_netif_ip_info_t ip_info;
      esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");

      if (netif == NULL || esp_netif_get_ip_info(netif, &ip_info) != ESP_OK ||
          ip_info.ip.addr == 0) {
        ESP_LOGW(TAG, "Lost IP address, WiFi connection may be unstable");
        isWiFiConnected_ = false;

        // Try to reconnect
        esp_wifi_disconnect();
        esp_wifi_connect();
      }
    } else {
      // If we're not connected, try to reconnect
      static uint32_t reconnectAttempts = 0;

      if (reconnectAttempts < 5 || reconnectAttempts % 6 == 0) { // Try frequently at first, then every ~1 minute
        ESP_LOGI(TAG, "Attempting to reconnect to WiFi... (attempt %lu)", reconnectAttempts + 1);
        esp_wifi_disconnect();
        esp_wifi_connect();
      }

      reconnectAttempts++;
    }
  }
}