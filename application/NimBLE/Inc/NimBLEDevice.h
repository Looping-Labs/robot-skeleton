#pragma once

#include "NimBLETypes.h"
#include <string>

namespace NimBLE {

  /**
   * @brief Main class for managing BLE device functionality
   *
   * This class serves as the primary entry point for BLE operations
   * and manages the lifecycle of the BLE stack.
   */
  class NimBLEDevice {
  public:
    /**
     * @brief Initialize the BLE device with the specified name
     * @param deviceName Name to be advertised
     */
    static void init(const std::string &deviceName);

    /**
     * @brief Deinitialize the BLE device
     */
    static void deinit();

    /**
     * @brief Create a BLE server
     * @return Pointer to the created server
     */
    static NimBLEServer *createServer();

    /**
     * @brief Get the advertising instance
     * @return Pointer to the advertising instance
     */
    static NimBLEAdvertising *getAdvertising();

    /**
     * @brief Set the device name
     * @param deviceName The new device name
     */
    static void setDeviceName(const std::string &deviceName);

  private:
    static NimBLEServer *m_pServer;
    static NimBLEAdvertising *m_pAdvertising;
    static uint8_t m_addrType;
    static bool m_initialized;

    // Host task and sync callback
    static void host_task(void *param);
    static void ble_on_sync(void);

    // Private constructor to prevent instantiation
    NimBLEDevice() = default;
  };

} // namespace NimBLE