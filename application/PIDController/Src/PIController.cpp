#include "PIController.h"
#include "esp_log.h"
#include "esp_mac.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

#define LOG_LEVEL_LOCAL ESP_LOG_VERBOSE
#define TAG "PIController"

namespace controller {

  PIController::PIController(float Kp, float Ki, uint32_t dt_ms, float min_output, float max_output)
      : BaseController(dt_ms, min_output, max_output),
        Kp(Kp), Ki(Ki), integral(0.0f),
        anti_windup(max_output) {
    // Log creation of controller with parameters
    ESP_LOGI(TAG, "PI Controller created with Kp=%.3f, Ki=%.3f, dt=%.3fs", Kp, Ki, dt);
  }

  esp_err_t PIController::init() {
    try {
      // Call base class initialization
      BaseController::init();

      // Reset controller state
      reset();
      ESP_LOGI(TAG, "PI Controller initialized successfully");
      return ESP_OK;
    } catch (const std::exception &e) {
      ESP_LOGE(TAG, "Failed to initialize PI controller: %s", e.what());
      return ESP_FAIL;
    }
  }

  void PIController::reset() {
    // Reset integral term
    integral = 0.0f;
    ESP_LOGI(TAG, "PI Controller state reset");
  }

  float PIController::compute(float error) {
    // Proportional term
    float p_term = Kp * error;

    // Integral term with anti-windup
    // Calculate new integral value
    integral += Ki * error * dt;

    // Apply anti-windup - limit integral term
    integral = applyLimits(integral, -anti_windup, anti_windup);

    // Calculate total output
    output = p_term + integral;

    // Apply output limits
    output = applyLimits(output, min_output, max_output);

    // Log detailed computation at debug level
    ESP_LOGD(TAG, "PI computation: error=%.3f, P=%.3f, I=%.3f, output=%.3f",
            error, p_term, integral, output);

    return output;
  }

  void PIController::setKp(float Kp) {
    this->Kp = Kp;
    ESP_LOGI(TAG, "Kp set to %.3f", Kp);
  }

  void PIController::setKi(float Ki) {
    this->Ki = Ki;
    ESP_LOGI(TAG, "Ki set to %.3f", Ki);

    // Reset integral when changing Ki to prevent sudden jumps
    integral = 0.0f;
  }

  void PIController::setGains(float Kp, float Ki) {
    this->Kp = Kp;
    this->Ki = Ki;

    // Reset integral when changing gains to prevent sudden jumps
    integral = 0.0f;

    ESP_LOGI(TAG, "PI gains set to Kp=%.3f, Ki=%.3f", Kp, Ki);
  }

  void PIController::setAntiWindupLimit(float limit) {
    // Ensure limit is positive
    limit = std::abs(limit);

    // Cap the anti-windup limit to the output limit
    limit = std::min(limit, std::abs(max_output));

    this->anti_windup = limit;

    // Re-clamp integral term to new limit
    integral = applyLimits(integral, -anti_windup, anti_windup);

    ESP_LOGI(TAG, "Anti-windup limit set to %.3f", limit);
  }

  float PIController::getKp() const {
    return Kp;
  }

  float PIController::getKi() const {
    return Ki;
  }

  float PIController::getIntegral() const {
    return integral;
  }

} // namespace controller