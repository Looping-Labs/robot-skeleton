#pragma once

#include "stdio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "esp_nimble_hci.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include <cstdint>
#include <string>

namespace NimBLE {

  // Forward declarations
  class NimBLEServer;
  class NimBLEService;
  class NimBLECharacteristic;
  class NimBLEAdvertising;

  /**
   * @brief Callback function types for BLE events
   */
  using ServerCallback = void (*)(NimBLEServer *server);
  using CharacteristicCallback = void (*)(NimBLECharacteristic *characteristic, struct ble_gatt_access_ctxt *ctxt);

  /**
   * @brief Properties for BLE characteristics
   */
  enum CharacteristicProperty {
    PROPERTY_READ = BLE_GATT_CHR_F_READ,
    PROPERTY_WRITE = BLE_GATT_CHR_F_WRITE,
    PROPERTY_NOTIFY = BLE_GATT_CHR_F_NOTIFY,
    PROPERTY_INDICATE = BLE_GATT_CHR_F_INDICATE,
    PROPERTY_BROADCAST = BLE_GATT_CHR_F_BROADCAST
  };

  /**
   * @brief UUID wrapper class to simplify UUID handling
   */
  class UUID {
  public:
    /**
     * @brief Construct a new UUID from a string
     * @param uuidStr UUID string in format "ABCD" (16-bit) or full UUID format
     */
    UUID(const std::string &uuidStr);

    /**
     * @brief Get the underlying NimBLE UUID structure
     */
    ble_uuid_any_t *getNative();

  private:
    ble_uuid_any_t m_uuid;
  };

} // namespace NimBLE