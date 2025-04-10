// nimble/Inc/NimBLEService.h
#pragma once

#include "NimBLETypes.h"
#include "host/ble_gatt.h"
#include <string>
#include <vector>

namespace NimBLE {

  /**
   * @brief Class for managing a BLE GATT service
   *
   * This class handles service operations including characteristic creation
   * and service lifecycle management.
   */
  class NimBLEService {
  public:
    /**
     * @brief Create a characteristic with the specified UUID and properties
     * @param characteristicUUID UUID for the characteristic
     * @param properties Bit flags for characteristic properties
     * @return Pointer to the created characteristic
     */
    NimBLECharacteristic *createCharacteristic(
        const std::string &characteristicUUID,
        uint32_t properties = PROPERTY_READ | PROPERTY_WRITE);

    /**
     * @brief Start the service
     * @return true if successful, false otherwise
     */
    bool start();

    /**
     * @brief Get the service UUID
     * @return Service UUID as string
     */
    const std::string &getUUID() const;

  private:
    friend class NimBLEServer; // Allow NimBLEServer to create services

    // Constructor (private, can only be created by NimBLEServer)
    NimBLEService(NimBLEServer *server, const std::string &uuid);
    ~NimBLEService();

    NimBLEServer *m_server;
    std::string m_uuid;
    std::vector<NimBLECharacteristic *> m_characteristics;
    ble_gatt_svc_def m_svcDef;
    bool m_started;
  };

} // namespace NimBLE