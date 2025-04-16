// nimble/Src/NimBLEDevice.cpp
#include "NimBLEDevice.h"
#include "NimBLEServer.h"
#include "NimBLEAdvertising.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

namespace NimBLE {

static const char* TAG = "NimBLEDevice";

// Static member initialization
NimBLEServer* NimBLEDevice::m_pServer = nullptr;
NimBLEAdvertising* NimBLEDevice::m_pAdvertising = nullptr;
uint8_t NimBLEDevice::m_addrType = 0;
bool NimBLEDevice::m_initialized = false;

void NimBLEDevice::init(const std::string& deviceName) {
    if (m_initialized) {
        ESP_LOGW(TAG, "BLE already initialized");
        return;
    }
    
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGI(TAG, "Erasing NVS flash...");
        nvs_flash_erase();
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Initialize NimBLE host stack (includes controller initialization in ESP-IDF 5.x)
    ESP_LOGI(TAG, "Initializing NimBLE stack...");
    nimble_port_init();
    
    // Initialize NimBLE configuration
    ble_svc_gap_device_name_set(deviceName.c_str());
    ble_svc_gap_init();
    ble_svc_gatt_init();
    
    // Set sync callback
    ble_hs_cfg.sync_cb = ble_on_sync;
    
    // Start the host task
    nimble_port_freertos_init(host_task);
    
    // Create advertising instance
    m_pAdvertising = new NimBLEAdvertising();
    
    m_initialized = true;
    ESP_LOGI(TAG, "BLE initialized with device name: %s", deviceName.c_str());
}

void NimBLEDevice::deinit() {
    if (!m_initialized) {
        return;
    }
    
    // Clean up resources
    if (m_pServer) {
        delete m_pServer;
        m_pServer = nullptr;
    }
    
    if (m_pAdvertising) {
        delete m_pAdvertising;
        m_pAdvertising = nullptr;
    }
    
    // Stop and deinitialize NimBLE
    ESP_LOGI(TAG, "Stopping NimBLE stack...");
    nimble_port_stop();
    
    // In ESP-IDF 5.x, nimble_port_deinit handles controller deinitialization
    nimble_port_deinit();
    
    m_initialized = false;
    ESP_LOGI(TAG, "BLE deinitialized");
}

NimBLEServer* NimBLEDevice::createServer() {
    if (!m_initialized) {
        ESP_LOGE(TAG, "BLE not initialized, call init() first");
        return nullptr;
    }
    
    if (!m_pServer) {
        m_pServer = new NimBLEServer();
    }
    
    return m_pServer;
}

NimBLEAdvertising* NimBLEDevice::getAdvertising() {
    if (!m_initialized) {
        ESP_LOGE(TAG, "BLE not initialized, call init() first");
        return nullptr;
    }
    
    return m_pAdvertising;
}

void NimBLEDevice::setDeviceName(const std::string& deviceName) {
    if (!m_initialized) {
        ESP_LOGE(TAG, "BLE not initialized, call init() first");
        return;
    }
    
    ble_svc_gap_device_name_set(deviceName.c_str());
    ESP_LOGI(TAG, "Device name set to: %s", deviceName.c_str());
}

void NimBLEDevice::host_task(void* param) {
    ESP_LOGI(TAG, "BLE host task started");
    nimble_port_run(); // This function will return only when nimble_port_stop() is executed
    nimble_port_freertos_deinit();
    ESP_LOGI(TAG, "BLE host task terminated");
}

void NimBLEDevice::ble_on_sync(void) {
    ESP_LOGI(TAG, "BLE host synced");
    
    // Determine the best address type automatically
    ble_hs_id_infer_auto(0, &m_addrType);
    
    // Start advertising if we have an advertising instance
    if (m_pAdvertising) {
        m_pAdvertising->start();
    }
}

} // namespace NimBLE