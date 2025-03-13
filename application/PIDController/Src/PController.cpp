#include "PController.h"
#include "esp_log.h"
#include "esp_mac.h"
#include <stdexcept>

#define LOG_LEVEL_LOCAL ESP_LOG_VERBOSE
#define TAG "PController"

namespace controller {

  PController::PController(float Kp, uint32_t dt_ms, float min_output, float max_output)
      : BaseController(dt_ms, min_output, max_output), Kp(Kp) {
    ESP_LOGI(TAG, "P Controller created with Kp=%.3f, dt=%.3fs", Kp, dt);
  }

  esp_err_t PController::init() {
    try {
      // Call base class initialization
      BaseController::init();

      // No state to reset for P controller
      reset();
      ESP_LOGI(TAG, "P Controller initialized successfully");
      return ESP_OK;
    } catch (const std::exception &e) {
      ESP_LOGE(TAG, "Failed to initialize P controller: %s", e.what());
      return ESP_FAIL;
    }
  }

  void PController::reset() {
    // Nothing to reset for P controller
    output = 0.0f;
    ESP_LOGI(TAG, "P Controller state reset");
  }

  float PController::compute(float error) {
    // Simple proportional control
    output = Kp * error;

    // Apply output limits
    output = applyLimits(output, min_output, max_output);

    // Log computation at debug level
    ESP_LOGD(TAG, "P computation: error=%.3f, P=%.3f, output=%.3f", error, Kp * error, output);

    return output;
  }

  void PController::setKp(float Kp) {
    this->Kp = Kp;
    ESP_LOGI(TAG, "Kp set to %.3f", Kp);
  }

  float PController::getKp() const {
    return Kp;
  }

} // namespace controller