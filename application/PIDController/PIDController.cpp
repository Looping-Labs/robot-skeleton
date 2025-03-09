#include "PIDController.h"
#include "esp_log.h"
#include "esp_mac.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

#define LOG_LEVEL_LOCAL ESP_LOG_VERBOSE
#define TAG "PIDController"

namespace controller {

PIDController::PIDController(float Kp, float Ki, float Kd, uint32_t dt_ms, float min_output, float max_output)
    : BaseController(dt_ms, min_output, max_output),
      Kp(Kp), Ki(Ki), Kd(Kd),
      integral(0.0f), prev_error(0.0f),
      anti_windup(max_output) {
  // Log creation of controller with parameters
  ESP_LOGI(TAG, "PID Controller created with Kp=%.3f, Ki=%.3f, Kd=%.3f, dt=%.3fs",
           Kp, Ki, Kd, dt);
}

esp_err_t PIDController::init() {
  try {
    // Call base class initialization
    BaseController::init();

    // Reset controller state
    reset();
    ESP_LOGI(TAG, "PID Controller initialized successfully");
    return ESP_OK;
  } catch (const std::exception &e) {
    ESP_LOGE(TAG, "Failed to initialize PID controller: %s", e.what());
    return ESP_FAIL;
  }
}

void PIDController::reset() {
  // Reset all state variables
  integral = 0.0f;
  prev_error = 0.0f;
  output = 0.0f;
  ESP_LOGI(TAG, "PID Controller state reset");
}

float PIDController::compute(float error) {
  // Proportional term
  float p_term = Kp * error;

  // Integral term with anti-windup
  // Calculate new integral value
  integral += Ki * error * dt;

  // Apply anti-windup - limit integral term
  integral = applyLimits(integral, -anti_windup, anti_windup);
  float i_term = integral;

  // Derivative term (on error change, not just setpoint change)
  // This helps avoid derivative Kick when setpoint changes
  float error_rate = (error - prev_error) / dt;
  float d_term = Kd * error_rate;

  // Store current error for next iteration
  prev_error = error;

  // Calculate total output
  output = p_term + i_term + d_term;

  // Apply output limits
  output = applyLimits(output, min_output, max_output);

  // Log detailed computation at debug level
  ESP_LOGD(TAG, "PID computation: error=%.3f, P=%.3f, I=%.3f, D=%.3f, output=%.3f",
           error, p_term, i_term, d_term, output);

  return output;
}

void PIDController::setKp(float Kp) {
  this->Kp = Kp;
  ESP_LOGI(TAG, "Kp set to %.3f", Kp);
}

void PIDController::setKi(float Ki) {
  this->Ki = Ki;
  ESP_LOGI(TAG, "Ki set to %.3f", Ki);

  // Reset integral when changing Ki to prevent sudden jumps
  integral = 0.0f;
}

void PIDController::setKd(float Kd) {
  this->Kd = Kd;
  ESP_LOGI(TAG, "Kd set to %.3f", Kd);
}

void PIDController::setGains(float Kp, float Ki, float Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;

  // Reset integral when changing gains to prevent sudden jumps
  integral = 0.0f;

  ESP_LOGI(TAG, "PID gains set to Kp=%.3f, Ki=%.3f, Kd=%.3f", Kp, Ki, Kd);
}

void PIDController::setAntiWindupLimit(float limit) {
  // Ensure limit is positive
  limit = std::abs(limit);

  // Cap the anti-windup limit to the output limit
  limit = std::min(limit, std::abs(max_output));

  this->anti_windup = limit;

  // Re-clamp integral term to new limit
  integral = applyLimits(integral, -anti_windup, anti_windup);

  ESP_LOGI(TAG, "Anti-windup limit set to %.3f", limit);
}

float PIDController::getKp() const {
  return Kp;
}

float PIDController::getKi() const {
  return Ki;
}

float PIDController::getKd() const {
  return Kd;
}

float PIDController::getIntegral() const {
  return integral;
}

} // namespace controller