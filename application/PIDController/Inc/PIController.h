#pragma once

#include "BaseController.h"

namespace controller {
  /**
   * @brief Proportional-Integral Controller implementation
   *
   * This controller uses proportional and integral terms to control a system.
   * Suitable for systems where steady-state error must be eliminated but
   * minimal overshoot is desired.
   */
  class PIController : public BaseController {
    private:
      /**
       * @brief PI Controller parameters
       * 
       * @var Kp: Proportional gain
       * @var Ki: Integral gain
       * @var integral: Accumulated error (integral term)
       * @var anti_windup: Anti-windup limit for integral term
       */
      float Kp;
      float Ki;
      float integral;  
      float anti_windup; 

    public:
      /**
       * @brief Construct a new PI Controller
       *
       * @param Kp: Proportional gain
       * @param Ki: Integral gain
       * @param dt_ms: Time step in milliseconds (default 1 ms)
       * @param min_output: Minimum output value (default -1023)
       * @param max_output: Maximum output value (default 1023)
       */
      PIController(float Kp, float Ki, uint32_t dt_ms = 1, float min_output = -1023.0f, float max_output = 1023.0f);

      /**
       * @brief Initialize the PI controller
       *
       * @return esp_err_t ESP_OK on success, ESP_FAIL otherwise
       */
      esp_err_t init() override;

      /**
       * @brief Reset the controller state (integral term)
       */
      void reset() override;

      /**
       * @brief Calculate PI output based on error
       *
       * @param error: Current error (setpoint - measured_value)
       * @return float Controller output between min_output and max_output
       */
      float compute(float error) override;

      /**
       * @brief Set the proportional gain
       *
       * @param Kp: Proportional gain
       */
      void setKp(float Kp);

      /**
       * @brief Set the integral gain
       *
       * @param Ki: Integral gain
       */
      void setKi(float Ki);

      /**
       * @brief Set both PI gains
       *
       * @param Kp: Proportional gain
       * @param Ki: Integral gain
       */
      void setGains(float Kp, float Ki);

      /**
       * @brief Set the anti-windup limit
       *
       * @param limit: Anti-windup limit for integral term
       */
      void setAntiWindupLimit(float limit);

      /**
       * @brief Get the proportional gain
       *
       * @return float Proportional gain
       */
      float getKp() const;

      /**
       * @brief Get the integral gain
       *
       * @return float Integral gain
       */
      float getKi() const;

      /**
       * @brief Get the integral term
       *
       * @return float Integral term
       */
      float getIntegral() const;
  };
} // namespace controller