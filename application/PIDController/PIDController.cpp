#include "PIDController.h"
#include "esp_log.h"
#include "esp_mac.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

// Define logging tag for this module
#define TAG "PIDController"
#define LOG_LEVEL_LOCAL ESP_LOG_VERBOSE

namespace pidController {
  PIDController::PIDController(float Kp, float Ki, float Kd, float dt_ms, float min_output, float max_output): 
                               Kp(Kp), Ki(Ki), Kd(Kd), dt(dt_ms / 1000.0f), min_output(min_output), max_output(max_output), 
                               anti_windup(max_output), use_p(true), use_i(true), use_d(true) {

   ESP_LOGI(TAG, "PID Controller initialized with Kp=%.3f, Ki=%.3f, Kd=%.3f, dt=%.3f", Kp, Ki, Kd, dt);

  }

  esp_err_t PIDController::init() {
    try {
      this->reset();
      ESP_LOGI(TAG, "PID Controller initialized successfully");
      return ESP_OK;
    } catch (const std::exception& e) {
        ESP_LOGE(TAG, "Failed to initialize PID controller: %s", e.what());
        return ESP_FAIL;
    }
  }

  void PIDController::reset() {
    this->integral = 0.0f;
    this->prev_error = 0.0f;
    this->output = 0.0f;

    ESP_LOGI(TAG, "PID Controller state reset");
  }

  float PIDController::compute(float error) {
    // Proportional term
    float p_term = use_p ? Kp * error : 0.0f;

    // Integral term with anti-windup
    float i_term = 0.0f;
    if (use_i) {
      // Calculate new integral term
      integral += Ki * error * dt;
      
      // Apply anti-windup - limit integral term to anti_windup
      if(integral > anti_windup) {
        integral = anti_windup;
      } else if(integral < -anti_windup) {
        integral = -anti_windup;
      }

      i_term = integral;
    }

    // Derivative term
    float d_term = 0.0f;
    if (use_d) {
      // Calculate derivative term (change in error over time)
      float error_rate = (error - prev_error) / dt;
      d_term = Kd * error_rate;
    }

    // Store current error for next iteration
    prev_error = error;

    // Calculate PID output
    output = p_term + i_term + d_term;

    // Apply output limits
    output = std::clamp(output, min_output, max_output);

    // Log detailed computation at debug level
    ESP_LOGD(TAG, "PID computation: error=%.3f, P=%.3f, I=%.3f, D=%.3f, output=%.3f",
      error, p_term, i_term, d_term, output);
 
    return output;
  }

  float PIDController::computeWithSetpoint(float measured_value) {
    float error = setpoint - measured_value;
    return compute(error);
  }

  void PIDController::setMode(bool p_on, bool i_on, bool d_on) {
    // Check if we are disabling all terms, which is not allowed
    if(!p_on && !i_on && !d_on) {
      throw std::invalid_argument("At least one PID term must be enabled");
    }
    this->use_p = p_on;
    this->use_i = i_on;
    this->use_d = d_on;

    // Reset integral component if integral term is disabled
    if(!i_on) {
      integral = 0.0f;
    }

    ESP_LOGI(TAG, "PID mode set to %s%s%s", 
      use_p ? "P" : "", use_i ? "I" : "", use_d ? "D" : "");
  }

  void PIDController::setKp(float Kp) {
    this->Kp = Kp;
    ESP_LOGI(TAG, "Kp set to %.3f", Kp);
  }

  void PIDController::setKi(float Ki) {
    this->Ki = Ki;
    ESP_LOGI(TAG, "Ki set to %.3f", Ki);
    
    // Reset integral when changing Ki to prevent sudden jumps
    if (use_i) {
        integral = 0.0f;
    }
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
    if (use_i) {
        integral = 0.0f;
    }
    
    ESP_LOGI(TAG, "PID gains set to Kp=%.3f, Ki=%.3f, Kd=%.3f", Kp, Ki, Kd);
  }

  void PIDController::setSampleTime(uint32_t dt_ms) {
    // Ensure dt is not zero to prevent division by zero in derivative calculation
    if (dt_ms == 0) {
        ESP_LOGW(TAG, "Cannot set dt to zero. Using minimum value of 1ms.");
        dt_ms = 1;
    }
    
    this->dt = dt_ms / 1000.0f;  // Convert ms to seconds
    ESP_LOGI(TAG, "dt set to %.3fs", dt);
  }

  void PIDController::setOutputLimits(float min_output, float max_output) {
    // Ensure min is less than max
    if (min_output >= max_output) {
        ESP_LOGW(TAG, "Invalid output limits: min (%.3f) must be less than max (%.3f)", 
                 min_output, max_output);
        return;
    }
    
    this->min_output = min_output;
    this->max_output = max_output;
    
    // Adjust anti-windup if needed
    if (std::abs(anti_windup) > max_output) {
        anti_windup = max_output;
    }
    
    // Clamp current output to new limits
    output = std::clamp(output, min_output, max_output);
    
    ESP_LOGI(TAG, "Output limits set to [%.3f, %.3f]", min_output, max_output);
  }

  void PIDController::setAntiWindupLimit(float limit) {
    // Ensure limit is positive
    limit = std::abs(limit);
    
    // Cap the anti-windup limit to the output limit
    if (limit > max_output) {
        ESP_LOGW(TAG, "Anti-windup limit (%.3f) exceeds max output (%.3f), capping at max output", 
                 limit, max_output);
        limit = max_output;
    }
    
    this->anti_windup = limit;
    
    // Re-clamp integral term to new limit
    if (integral > anti_windup) {
        integral = anti_windup;
    } else if (integral < -anti_windup) {
        integral = -anti_windup;
    }
    
    ESP_LOGI(TAG, "Anti-windup limit set to %.3f", limit);
  }

  void PIDController::setSetpoint(float setpoint) {
    this->setpoint = setpoint;
    ESP_LOGD(TAG, "Setpoint set to %.3f", setpoint);
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

  uint32_t PIDController::getSampleTime() const {
      return dt;
  }

  float PIDController::getSetpoint() const {
      return setpoint;
  }

  float PIDController::getOutput() const {
      return output;
  }

  float PIDController::getIntegral() const {
      return integral;
  }
}
