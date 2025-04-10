// nimble/Inc/NimBLECharacteristic.h
#pragma once

#include "NimBLETypes.h"
#include "host/ble_gatt.h"
#include <string>

namespace NimBLE {

  /**
   * @brief Class for managing a BLE GATT characteristic
   *
   * This class handles characteristic operations including value
   * setting/getting and callback management.
   */
  class NimBLECharacteristic {
  public:
    /**
     * @brief Set the value of the characteristic
     * @param value The new value as a string
     * @return true if successful, false otherwise
     */
    bool setValue(const std::string &value);

    /**
     * @brief Set the value of the characteristic
     * @param data Pointer to the data buffer
     * @param length Length of the data
     * @return true if successful, false otherwise
     */
    bool setValue(const uint8_t *data, size_t length);

    /**
     * @brief Get the current value of the characteristic
     * @return The value as a string
     */
    std::string getValue() const;

    /**
     * @brief Set callback for read operations
     * @param callback Function to call when the characteristic is read
     */
    void setReadCallback(CharacteristicCallback callback);

    /**
     * @brief Set callback for write operations
     * @param callback Function to call when the characteristic is written
     */
    void setWriteCallback(CharacteristicCallback callback);

    /**
     * @brief Handle read operations (internal use)
     * @param ctxt The access context
     * @return 0 on success, error code otherwise
     */
    int handleRead(struct ble_gatt_access_ctxt *ctxt);

    /**
     * @brief Handle write operations (internal use)
     * @param ctxt The access context
     * @return 0 on success, error code otherwise
     */
    int handleWrite(struct ble_gatt_access_ctxt *ctxt);

  private:
    friend class NimBLEService; // Allow NimBLEService to create characteristics

    // Constructor (private, can only be created by NimBLEService)
    NimBLECharacteristic(
        NimBLEService *service,
        const std::string &uuid,
        uint32_t properties);
    ~NimBLECharacteristic();

    NimBLEService *m_service;
    std::string m_uuid;
    uint32_t m_properties;
    std::string m_value;
    CharacteristicCallback m_readCallback;
    CharacteristicCallback m_writeCallback;
    ble_gatt_chr_def m_chrDef;

    // Static callback for NimBLE
    static int access_cb(uint16_t conn_handle, uint16_t attr_handle,
                         struct ble_gatt_access_ctxt *ctxt, void *arg);
  };

} // namespace NimBLE