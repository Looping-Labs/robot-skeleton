// nimble/Inc/NimBLEServer.h
#pragma once

#include "NimBLETypes.h"
#include <string>
#include <vector>

namespace NimBLE {

  /**
   * @brief Class for managing a BLE GATT server
   *
   * This class handles server operations including service creation
   * and connection events.
   */
  class NimBLEServer {
  public:
    /**
     * @brief Create a service with the specified UUID
     * @param serviceUUID UUID for the service
     * @return Pointer to the created service
     */
    NimBLEService *createService(const std::string &serviceUUID);

    /**
     * @brief Set callback for connection events
     * @param callback Function to call when a client connects
     */
    void setConnectCallback(ServerCallback callback);

    /**
     * @brief Set callback for disconnection events
     * @param callback Function to call when a client disconnects
     */
    void setDisconnectCallback(ServerCallback callback);

    /**
     * @brief Internal method to handle GAP events
     * @param event The GAP event to handle
     * @return 0 on success, error code otherwise
     */
    int handleGAPEvent(struct ble_gap_event *event);

  private:
    friend class NimBLEDevice; // Allow NimBLEDevice to create servers

    // Constructor (private, can only be created by NimBLEDevice)
    NimBLEServer();
    ~NimBLEServer();

    std::vector<NimBLEService *> m_services;
    ServerCallback m_connectCallback;
    ServerCallback m_disconnectCallback;
    uint16_t m_connHandle;
  };

} // namespace NimBLE