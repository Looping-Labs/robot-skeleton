#include "PDController.h"
#include "esp_log.h"
#include "esp_mac.h"
#include <stdexcept>

#define LOG_LEVEL_LOCAL ESP_LOG_VERBOSE
#define TAG "PDController"

namespace controller {

  PDController::PDController(float Kp, float Kd, uint32_t dt_ms, float min_output, float max_output)
      : BaseController(dt_ms, min_output, max_output),
        Kp(Kp), Kd(Kd), prev_error(0.0f) {
    ESP_LOGI(TAG, "PD Controller created with Kp=%.3f, Kd=%.3f, dt=%.3fs", Kp, Kd, dt);
  }

  esp_err_t PDController::init() {
    try {
      // Call base class initialization
      BaseController::init();

      // Reset controller state
      reset();
      ESP_LOGI(TAG, "PD Controller initialized successfully");
      return ESP_OK;
    } catch (const std::exception &e) {
      ESP_LOGE(TAG, "Failed to initialize PD controller: %s", e.what());
      return ESP_FAIL;
    }
  }

  void PDController::reset() {
    // Reset state variable
    prev_error = 0.0f;
    output = 0.0f;
    ESP_LOGI(TAG, "PD Controller state reset");
  }

  float PDController::compute(float error) {
    // Proportional term
    float p_term = Kp * error;

    // Derivative term (on error change, not just setpoint change)
    // This helps avoid derivative Kick when setpoint changes
    float error_rate = (error - prev_error) / dt;
    float d_term = Kd * error_rate;

    // Store current error for next iteration
    prev_error = error;

    // Calculate total output
    output = p_term + d_term;

    // Apply output limits
    output = applyLimits(output, min_output, max_output);

    // Log detailed computation at debug level
    ESP_LOGD(TAG, "PD computation: error=%.3f, P=%.3f, D=%.3f, output=%.3f", error, p_term, d_term, output);

    return output;
  }

  void PDController::setKp(float Kp) {
    this->Kp = Kp;
    ESP_LOGI(TAG, "Kp set to %.3f", Kp);
  }

  void PDController::setKd(float Kd) {
    this->Kd = Kd;
    ESP_LOGI(TAG, "Kd set to %.3f", Kd);
  }

  void PDController::setGains(float Kp, float Kd) {
    this->Kp = Kp;
    this->Kd = Kd;
    ESP_LOGI(TAG, "PD gains set to Kp=%.3f, Kd=%.3f", Kp, Kd);
  }

  float PDController::getKp() const {
    return Kp;
  }

  float PDController::getKd() const {
    return Kd;
  }

} // namespace controller