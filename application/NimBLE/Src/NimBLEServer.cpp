// nimble/Src/NimBLEServer.cpp
#include "NimBLEServer.h"
#include "NimBLEService.h"
#include "esp_log.h"

namespace NimBLE {

  static const char *TAG = "NimBLEServer";

  // Forward declaration of GAP event handler
  static int gap_event_handler(struct ble_gap_event *event, void *arg);

  NimBLEServer::NimBLEServer()
      : m_connectCallback(nullptr),
        m_disconnectCallback(nullptr),
        m_connHandle(BLE_HS_CONN_HANDLE_NONE) {
    ESP_LOGI(TAG, "Server created");
  }

  NimBLEServer::~NimBLEServer() {
    // Clean up services
    for (auto service : m_services) {
      delete service;
    }
    m_services.clear();
  }

  NimBLEService *NimBLEServer::createService(const std::string &serviceUUID) {
    NimBLEService *pService = new NimBLEService(this, serviceUUID);
    m_services.push_back(pService);
    ESP_LOGI(TAG, "Service created with UUID: %s", serviceUUID.c_str());
    return pService;
  }

  void NimBLEServer::setConnectCallback(ServerCallback callback) {
    m_connectCallback = callback;
  }

  void NimBLEServer::setDisconnectCallback(ServerCallback callback) {
    m_disconnectCallback = callback;
  }

  int NimBLEServer::handleGAPEvent(struct ble_gap_event *event) {
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
      ESP_LOGI(TAG, "BLE GAP EVENT CONNECT %s",
               event->connect.status == 0 ? "OK!" : "FAILED!");

      if (event->connect.status == 0) {
        // Connection established
        m_connHandle = event->connect.conn_handle;

        // Call the connect callback if set
        if (m_connectCallback) {
          m_connectCallback(this);
        }
      } else {
        // If connection failed, restart advertising
        ble_app_advertise();
      }
      break;

    case BLE_GAP_EVENT_DISCONNECT:
      ESP_LOGI(TAG, "BLE GAP EVENT DISCONNECT");
      m_connHandle = BLE_HS_CONN_HANDLE_NONE;

      // Call the disconnect callback if set
      if (m_disconnectCallback) {
        m_disconnectCallback(this);
      }

      // Restart advertising after disconnection
      ble_app_advertise();
      break;

    case BLE_GAP_EVENT_ADV_COMPLETE:
      ESP_LOGI(TAG, "BLE GAP EVENT ADV COMPLETE");
      ble_app_advertise();
      break;

    default:
      break;
    }

    return 0;
  }

  // Static function for advertising
  static void ble_app_advertise(void) {
    // To be implemented in advertising class
  }

  // Static GAP event handler
  static int gap_event_handler(struct ble_gap_event *event, void *arg) {
    NimBLEServer *server = static_cast<NimBLEServer *>(arg);
    return server->handleGAPEvent(event);
  }

} // namespace NimBLE