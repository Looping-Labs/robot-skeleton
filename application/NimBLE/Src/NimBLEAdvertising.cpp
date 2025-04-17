// nimble/Src/NimBLEAdvertising.cpp
#include "../Inc/NimBLEAdvertising.h"
#include "../Inc/NimBLEDevice.h"
#include "esp_log.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include <algorithm>
#include <cstring>

namespace NimBLE {

  static const char *TAG = "NimBLEAdvertising";

  // Forward declaration for the GAP event handler function
  static int gap_event_handler(struct ble_gap_event *event, void *arg);

  NimBLEAdvertising::NimBLEAdvertising()
      : m_advertising(false),
        m_uuidArray(nullptr) {
    ESP_LOGI(TAG, "Advertising instance created");
  }

  NimBLEAdvertising::~NimBLEAdvertising() {
    if (m_advertising) {
      stop();
    }

    // Free the UUID array if it exists
    if (m_uuidArray) {
      delete[] m_uuidArray;
      m_uuidArray = nullptr;
    }
  }

  bool NimBLEAdvertising::start() {
    if (m_advertising) {
      ESP_LOGW(TAG, "Advertising already started");
      return true;
    }

    // Set up advertising fields
    struct ble_hs_adv_fields fields;
    memset(&fields, 0, sizeof(fields));

    // Set the device name
    if (!m_name.empty()) {
      fields.name = reinterpret_cast<const uint8_t *>(m_name.c_str());
      fields.name_len = m_name.length();
      fields.name_is_complete = 1;
    } else {
      // If no name is set, use the gap device name
      const char *device_name = ble_svc_gap_device_name();
      fields.name = reinterpret_cast<const uint8_t *>(device_name);
      fields.name_len = strlen(device_name);
      fields.name_is_complete = 1;
    }

    // Clean up any existing UUID array before creating a new one
    if (m_uuidArray) {
      delete[] m_uuidArray;
      m_uuidArray = nullptr;
    }

    // Set service UUIDs if available
    if (!m_serviceUUIDs.empty()) {
      // Allocate memory for the UUIDs that will persist until next advertising change
      m_uuidArray = new ble_uuid16_t[m_serviceUUIDs.size()];

      int valid_uuids = 0;
      for (size_t i = 0; i < m_serviceUUIDs.size(); i++) {
        const std::string &uuidStr = m_serviceUUIDs[i];

        // Parse the UUID string into a 16-bit integer
        // Supporting simple 16-bit UUIDs like "180F" or "ABCD"
        uint16_t uuid_val = 0;
        if (sscanf(uuidStr.c_str(), "%hx", &uuid_val) != 1) {
          ESP_LOGE(TAG, "Invalid UUID format: %s", uuidStr.c_str());
          continue;
        }

        // Fill the UUID structure in our array
        m_uuidArray[valid_uuids].u.type = BLE_UUID_TYPE_16;
        m_uuidArray[valid_uuids].value = uuid_val;
        valid_uuids++;
      }

      if (valid_uuids > 0) {
        // Now set the field with a pointer to our UUID array
        // The fields.uuids16 field expects a pointer to an array of ble_uuid16_t
        fields.uuids16 = m_uuidArray;
        fields.num_uuids16 = valid_uuids;
        fields.uuids16_is_complete = 1;
      }
    }

    // Set the advertising fields
    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
      ESP_LOGE(TAG, "Error setting advertising fields: %d", rc);
      return false;
    }

    // Set advertising parameters
    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND; // Connectable
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN; // Discoverable

    // Get the address type
    uint8_t addr_type;
    rc = ble_hs_id_infer_auto(0, &addr_type);
    if (rc != 0) {
      ESP_LOGE(TAG, "Error determining address type: %d", rc);
      return false;
    }

    // Start advertising
    rc = ble_gap_adv_start(
        addr_type,
        NULL,           // No direct address
        BLE_HS_FOREVER, // Advertise indefinitely
        &adv_params,
        gap_event_handler, // Event handler function
        this               // Pass the advertising instance as context
    );

    if (rc != 0) {
      ESP_LOGE(TAG, "Error starting advertising: %d", rc);
      return false;
    }

    m_advertising = true;
    ESP_LOGI(TAG, "Advertising started successfully");
    return true;
  }

  bool NimBLEAdvertising::stop() {
    if (!m_advertising) {
      ESP_LOGW(TAG, "Advertising not started");
      return true;
    }

    int rc = ble_gap_adv_stop();
    if (rc != 0) {
      ESP_LOGE(TAG, "Error stopping advertising: %d", rc);
      return false;
    }

    m_advertising = false;
    ESP_LOGI(TAG, "Advertising stopped");
    return true;
  }

  void NimBLEAdvertising::addServiceUUID(const std::string &serviceUUID) {
    // Add the UUID if it's not already in the list
    if (std::find(m_serviceUUIDs.begin(), m_serviceUUIDs.end(), serviceUUID) == m_serviceUUIDs.end()) {
      m_serviceUUIDs.push_back(serviceUUID);
      ESP_LOGI(TAG, "Added service UUID: %s", serviceUUID.c_str());
    }

    // If we're already advertising, restart it to apply the new UUID
    if (m_advertising) {
      stop();
      start();
    }
  }

  void NimBLEAdvertising::setName(const std::string &name) {
    m_name = name;
    ESP_LOGI(TAG, "Set advertising name: %s", name.c_str());

    // If we're already advertising, restart it to apply the new name
    if (m_advertising) {
      stop();
      start();
    }
  }

  // Static GAP event handler function
  static int gap_event_handler(struct ble_gap_event *event, void *arg) {
    NimBLEAdvertising *advertising = static_cast<NimBLEAdvertising *>(arg);

    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
      ESP_LOGI(TAG, "BLE GAP EVENT CONNECT %s",
               event->connect.status == 0 ? "OK!" : "FAILED!");
      // Connection handling is done in the NimBLEServer class
      break;

    case BLE_GAP_EVENT_DISCONNECT:
      ESP_LOGI(TAG, "BLE GAP EVENT DISCONNECT");
      // Restart advertising after disconnection
      if (advertising) {
        advertising->start();
      }
      break;

    case BLE_GAP_EVENT_ADV_COMPLETE:
      ESP_LOGI(TAG, "BLE GAP EVENT ADV COMPLETE");
      // Restart advertising after completion
      if (advertising) {
        advertising->start();
      }
      break;

    default:
      break;
    }

    return 0;
  }

} // namespace NimBLE