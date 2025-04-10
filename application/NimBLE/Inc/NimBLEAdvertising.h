// nimble/Inc/NimBLEAdvertising.h
#pragma once

#include "NimBLETypes.h"
#include <string>
#include <vector>

namespace NimBLE {

  /**
   * @brief Class for managing BLE advertising
   *
   * This class handles advertising configuration and control.
   */
  class NimBLEAdvertising {
  public:
    /**
     * @brief Start advertising
     * @return true if successful, false otherwise
     */
    bool start();

    /**
     * @brief Stop advertising
     * @return true if successful, false otherwise
     */
    bool stop();

    /**
     * @brief Add a service UUID to advertise
     * @param serviceUUID UUID of the service to advertise
     */
    void addServiceUUID(const std::string &serviceUUID);

    /**
     * @brief Set the advertised device name
     * @param name Name to advertise
     */
    void setName(const std::string &name);

  private:
    friend class NimBLEDevice; // Allow NimBLEDevice to create advertising

    // Constructor (private, can only be created by NimBLEDevice)
    NimBLEAdvertising();
    ~NimBLEAdvertising();

    std::vector<std::string> m_serviceUUIDs;
    std::string m_name;
    bool m_advertising;
  };

} // namespace NimBLE