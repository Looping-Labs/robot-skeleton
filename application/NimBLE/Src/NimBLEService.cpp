// nimble/Src/NimBLEService.cpp
#include "NimBLEService.h"
#include "NimBLECharacteristic.h"
#include "NimBLEServer.h"
#include "esp_log.h"

namespace NimBLE {

  static const char *TAG = "NimBLEService";

  NimBLEService::NimBLEService(NimBLEServer *server, const std::string &uuid)
      : m_server(server),
        m_uuid(uuid),
        m_started(false) {
    // Initialize service definition
    memset(&m_svcDef, 0, sizeof(m_svcDef));
    m_svcDef.type = BLE_GATT_SVC_TYPE_PRIMARY;
    // UUID will be set when service is started
  }

  NimBLEService::~NimBLEService() {
    // Clean up characteristics
    for (auto characteristic : m_characteristics) {
      delete characteristic;
    }
    m_characteristics.clear();
  }

  NimBLECharacteristic *NimBLEService::createCharacteristic(
      const std::string &characteristicUUID,
      uint32_t properties) {
    NimBLECharacteristic *pCharacteristic = new NimBLECharacteristic(
        this,
        characteristicUUID,
        properties);

    m_characteristics.push_back(pCharacteristic);
    ESP_LOGI(TAG, "Characteristic created with UUID: %s", characteristicUUID.c_str());

    return pCharacteristic;
  }

  bool NimBLEService::start() {
    if (m_started) {
      ESP_LOGW(TAG, "Service already started");
      return true;
    }

    if (m_characteristics.empty()) {
      ESP_LOGW(TAG, "No characteristics defined for service");
      return false;
    }

    // Create UUID - correctly handling the 16-bit UUID case
    ble_uuid16_t uuid16;
    ble_uuid_t *uuid_ptr = nullptr;

    if (m_uuid.length() <= 4) {
      // 16-bit UUID
      uint16_t uuid_val = 0;
      sscanf(m_uuid.c_str(), "%hx", &uuid_val);

      uuid16.u.type = BLE_UUID_TYPE_16;
      uuid16.value = uuid_val;
      uuid_ptr = &uuid16.u;
    } else {
      // 128-bit UUID not implemented for simplicity
      ESP_LOGE(TAG, "128-bit UUIDs not implemented yet");
      return false;
    }

    // Prepare service definition with the proper UUID pointer
    m_svcDef.uuid = uuid_ptr;

    // Create characteristic definitions
    std::vector<ble_gatt_chr_def> chr_defs;
    for (auto characteristic : m_characteristics) {
      chr_defs.push_back(characteristic->m_chrDef);
    }

    // Null-terminate the characteristic definitions
    ble_gatt_chr_def null_chr = {0};
    chr_defs.push_back(null_chr);

    // Set the characteristics in the service definition
    m_svcDef.characteristics = chr_defs.data();

    // Create the service definition array with a null terminator
    ble_gatt_svc_def svc_defs[2] = {
        m_svcDef,
        {0}};

    // Register the service with NimBLE
    int rc = ble_gatts_count_cfg(svc_defs);
    if (rc != 0) {
      ESP_LOGE(TAG, "ble_gatts_count_cfg failed: %d", rc);
      return false;
    }

    rc = ble_gatts_add_svcs(svc_defs);
    if (rc != 0) {
      ESP_LOGE(TAG, "ble_gatts_add_svcs failed: %d", rc);
      return false;
    }

    m_started = true;
    ESP_LOGI(TAG, "Service started with UUID: %s", m_uuid.c_str());
    return true;
  }

  const std::string &NimBLEService::getUUID() const {
    return m_uuid;
  }

} // namespace NimBLE