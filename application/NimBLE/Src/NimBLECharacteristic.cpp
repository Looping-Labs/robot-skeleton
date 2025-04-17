// nimble/Src/NimBLECharacteristic.cpp
#include "../Inc/NimBLECharacteristic.h"
#include "../Inc/NimBLEService.h"
#include "esp_log.h"
#include "os/os_mbuf.h"

namespace NimBLE {

  static const char *TAG = "NimBLECharacteristic";

  // Static callback for NimBLE
  int NimBLECharacteristic::access_cb(uint16_t conn_handle, uint16_t attr_handle,
                                      struct ble_gatt_access_ctxt *ctxt, void *arg) {
    NimBLECharacteristic *characteristic = static_cast<NimBLECharacteristic *>(arg);

    switch (ctxt->op) {
    case BLE_GATT_ACCESS_OP_READ_CHR:
      return characteristic->handleRead(ctxt);

    case BLE_GATT_ACCESS_OP_WRITE_CHR:
      return characteristic->handleWrite(ctxt);

    default:
      return BLE_ATT_ERR_UNLIKELY;
    }
  }

  NimBLECharacteristic::NimBLECharacteristic(
      NimBLEService *service,
      const std::string &uuid,
      uint32_t properties) : m_service(service),
                             m_uuid(uuid),
                             m_properties(properties),
                             m_readCallback(nullptr),
                             m_writeCallback(nullptr) {

    // Initialize characteristic definition
    memset(&m_chrDef, 0, sizeof(m_chrDef));

    // Properly create the UUID structure
    ble_uuid16_t *uuid16 = new ble_uuid16_t();
    if (m_uuid.length() <= 4) {
      // 16-bit UUID
      uint16_t uuid_val = 0;
      sscanf(m_uuid.c_str(), "%hx", &uuid_val);
      uuid16->u.type = BLE_UUID_TYPE_16;
      uuid16->value = uuid_val;

      // Store the pointer to the base type
      m_chrDef.uuid = &uuid16->u;
    } else {
      // 128-bit UUID not implemented for simplicity
      ESP_LOGE(TAG, "128-bit UUIDs not implemented yet");
      delete uuid16;
      m_chrDef.uuid = nullptr;
    }

    m_chrDef.flags = properties;
    m_chrDef.access_cb = access_cb;
    m_chrDef.arg = this;
  }

  NimBLECharacteristic::~NimBLECharacteristic() {
    // Clean up UUID if dynamically allocated
    if (m_chrDef.uuid) {
      delete m_chrDef.uuid;
    }
  }

  bool NimBLECharacteristic::setValue(const std::string &value) {
    m_value = value;
    return true;
  }

  bool NimBLECharacteristic::setValue(const uint8_t *data, size_t length) {
    m_value.assign(reinterpret_cast<const char *>(data), length);
    return true;
  }

  std::string NimBLECharacteristic::getValue() const {
    return m_value;
  }

  void NimBLECharacteristic::setReadCallback(CharacteristicCallback callback) {
    m_readCallback = callback;
  }

  void NimBLECharacteristic::setWriteCallback(CharacteristicCallback callback) {
    m_writeCallback = callback;
  }

  int NimBLECharacteristic::handleRead(struct ble_gatt_access_ctxt *ctxt) {
    // Call the read callback if set
    if (m_readCallback) {
      m_readCallback(this, ctxt);
      return 0;
    }

    // Otherwise, use the stored value
    int rc = os_mbuf_append(ctxt->om, m_value.data(), m_value.length());
    if (rc != 0) {
      ESP_LOGE(TAG, "Error appending value to mbuf: %d", rc);
      return BLE_ATT_ERR_INSUFFICIENT_RES;
    }

    ESP_LOGI(TAG, "Read characteristic value: %s", m_value.c_str());
    return 0;
  }

  int NimBLECharacteristic::handleWrite(struct ble_gatt_access_ctxt *ctxt) {
    // Get the data from the mbuf
    uint16_t dataLen = OS_MBUF_PKTLEN(ctxt->om);
    std::vector<uint8_t> data(dataLen);

    int rc = os_mbuf_copydata(ctxt->om, 0, dataLen, data.data());
    if (rc != 0) {
      ESP_LOGE(TAG, "Error copying data from mbuf: %d", rc);
      return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

    // Update the stored value
    m_value.assign(reinterpret_cast<char *>(data.data()), dataLen);

    ESP_LOGI(TAG, "Wrote characteristic value: %s", m_value.c_str());

    // Call the write callback if set
    if (m_writeCallback) {
      m_writeCallback(this, ctxt);
    }

    return 0;
  }

} // namespace NimBLE