#include "WebSocketHandler.h"
#include "RobotEndpoints.h"
#include "esp_log.h"
#include <algorithm>
#include <cstring>
#include <mutex>

static const char *TAG = "WebSocketHandler";

// Static handler function that will be registered with the HTTP server
static esp_err_t webSocketHandler(httpd_req_t *req) {
  WebSocketHandler *handler = static_cast<WebSocketHandler *>(req->user_ctx);
  if (handler == nullptr) {
    ESP_LOGE(TAG, "WebSocketHandler context is null");
    return ESP_FAIL;
  }

  // Route the request to our C++ handler
  return handler->handleFrame(req);
}

WebSocketHandler::WebSocketHandler() : server_(nullptr) {
}

esp_err_t WebSocketHandler::init(httpd_handle_t server) {
  server_ = server;
  return ESP_OK;
}

esp_err_t WebSocketHandler::registerEndpoint(const std::string &path, RobotEndpoints *endpoints) {
  if (server_ == nullptr) {
    ESP_LOGE(TAG, "Server not initialized. Call init() first.");
    return ESP_FAIL;
  }

  // Store the route mapping
  routes_[path] = endpoints;

  // Create the URI handler for this path
  httpd_uri_t uri_handler = {
      .uri = path.c_str(),
      .method = HTTP_GET,
      .handler = webSocketHandler,
      .user_ctx = this,
      .is_websocket = true};

  // Register with the HTTP server
  esp_err_t ret = httpd_register_uri_handler(server_, &uri_handler);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to register URI handler for path %s: %d", path.c_str(), ret);
    return ret;
  }

  ESP_LOGI(TAG, "Registered endpoint: %s", path.c_str());
  return ESP_OK;
}

esp_err_t WebSocketHandler::handleFrame(httpd_req_t *req) {
  // Handle WebSocket handshake (open connection)
  if (req->method == HTTP_GET) {
    ESP_LOGI(TAG, "Handshake for %s", req->uri);

    // Add client to our list
    std::lock_guard<std::mutex> lock(clientsMutex_);
    int fd = httpd_req_to_sockfd(req);

    // Check if client is already in our list
    auto it = std::find_if(clients_.begin(), clients_.end(),
                           [fd](const ClientInfo &client) { return client.fd == fd; });

    if (it == clients_.end()) {
      // Add new client
      ClientInfo client = {.fd = fd, .path = req->uri};
      clients_.push_back(client);
      ESP_LOGI(TAG, "New client connected: fd=%d, path=%s, total=%d",
               fd, req->uri, clients_.size());
    }

    return ESP_OK;
  }

  // Handle incoming WebSocket frame
  httpd_ws_frame_t ws_pkt;
  memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));

  // First, get the frame length
  esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to get frame length: %d", ret);
    return ret;
  }

  // Check if we have payload data
  if (ws_pkt.len == 0) {
    ESP_LOGW(TAG, "Received empty frame");
    return ESP_OK;
  }

  // Allocate memory for payload
  uint8_t *buf = new uint8_t[ws_pkt.len + 1];
  if (buf == nullptr) {
    ESP_LOGE(TAG, "Failed to allocate memory for frame payload");
    return ESP_ERR_NO_MEM;
  }

  // Set the buffer as the payload
  ws_pkt.payload = buf;

  // Get the actual payload data
  ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to get frame payload: %d", ret);
    delete[] buf;
    return ret;
  }

  // Null-terminate if it's a text message
  if (ws_pkt.type == HTTPD_WS_TYPE_TEXT) {
    buf[ws_pkt.len] = 0;
  }

  // Create a WebSocketMessage to pass to the handler
  WebSocketMessage msg = {
      .fd = httpd_req_to_sockfd(req),
      .payload = std::string(reinterpret_cast<char *>(buf), ws_pkt.len),
      .type = ws_pkt.type};

  // Clean up the buffer, we've copied the data
  delete[] buf;

  // Get the path from the URI
  std::string uri(req->uri);

  // Find the handler for this path
  for (const auto &route : routes_) {
    if (uri.find(route.first) == 0) {
      // This handler handles this path
      if (route.second != nullptr) {
        route.second->handleMessage(msg);
        return ESP_OK;
      }
    }
  }

  // No handler found for this path
  ESP_LOGW(TAG, "No handler found for path: %s", uri.c_str());

  // Echo the message back as a fallback
  return sendFrame(msg.fd, msg.payload, msg.type);
}

esp_err_t WebSocketHandler::sendFrame(int fd, const std::string &message, httpd_ws_type_t type) {
  if (server_ == nullptr) {
    ESP_LOGE(TAG, "Server not initialized");
    return ESP_FAIL;
  }

  // Create async send argument
  auto *arg = new AsyncSendArg{
      .hd = server_,
      .fd = fd,
      .payload = message,
      .type = type};

  if (arg == nullptr) {
    ESP_LOGE(TAG, "Failed to allocate memory for async send");
    return ESP_ERR_NO_MEM;
  }

  // Queue the work to send asynchronously
  esp_err_t ret = httpd_queue_work(server_, asyncSendFrame, arg);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to queue async send: %d", ret);
    delete arg;
    return ret;
  }

  return ESP_OK;
}

void WebSocketHandler::asyncSendFrame(void *arg) {
  auto *sendArg = static_cast<AsyncSendArg *>(arg);

  // Create the WebSocket packet
  httpd_ws_frame_t ws_pkt;
  memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
  ws_pkt.payload = reinterpret_cast<uint8_t *>(const_cast<char *>(sendArg->payload.data()));
  ws_pkt.len = sendArg->payload.length();
  ws_pkt.type = sendArg->type;

  // Send the frame
  esp_err_t ret = httpd_ws_send_frame_async(sendArg->hd, sendArg->fd, &ws_pkt);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to send async frame: %d", ret);
  }

  // Clean up
  delete sendArg;
}

esp_err_t WebSocketHandler::broadcast(const std::string &message, httpd_ws_type_t type) {
  std::lock_guard<std::mutex> lock(clientsMutex_);

  esp_err_t result = ESP_OK;
  for (const auto &client : clients_) {
    esp_err_t ret = sendFrame(client.fd, message, type);
    if (ret != ESP_OK) {
      // If any send fails, we'll return an error, but we'll still try to send to all clients
      result = ret;
    }
  }

  return result;
}

esp_err_t WebSocketHandler::broadcastToPath(const std::string &pathPrefix,
                                            const std::string &message,
                                            httpd_ws_type_t type) {
  std::lock_guard<std::mutex> lock(clientsMutex_);

  esp_err_t result = ESP_OK;
  for (const auto &client : clients_) {
    if (client.path.find(pathPrefix) == 0) {
      esp_err_t ret = sendFrame(client.fd, message, type);
      if (ret != ESP_OK) {
        result = ret;
      }
    }
  }

  return result;
}

JsonHelper &WebSocketHandler::getJsonHelper() {
  return jsonHelper_;
}

void WebSocketHandler::handleClientDisconnect(int fd) {
  std::lock_guard<std::mutex> lock(clientsMutex_);

  auto it = std::find_if(clients_.begin(), clients_.end(),
                         [fd](const ClientInfo &client) { return client.fd == fd; });

  if (it != clients_.end()) {
    ESP_LOGI(TAG, "Client disconnected: fd=%d, path=%s, remaining=%d",
             fd, it->path.c_str(), clients_.size() - 1);
    clients_.erase(it);
  }
}