#pragma once

#include "BaseController.h"

namespace controller {

  /**
   * @brief Proportional-Derivative Controller implementation
   *
   * Controller that uses proportional and derivative terms.
   * Provides improved stability and reduced overshoot compared to P-only control.
   * Suitable for systems where damping/stability is important.
   */
  class PDController : public BaseController {
    private:
      /**
       * @brief PD Controller parameters
       * 
       * @var Kp: Proportional gain
       * @var Kd: Derivative gain
       * @var prev_error: Previous error (for derivative term)
       */
      float Kp; 
      float Kd; 
      float prev_error; 

    public:
      /**
       * @brief Construct a new PD Controller
       *
       * @param Kp: Proportional gain
       * @param Kd: Derivative gain
       * @param dt_ms: Time step in milliseconds (default 1 ms)
       * @param min_output: Minimum output value (default -1023)
       * @param max_output: Maximum output value (default 1023)
       */
      PDController(float Kp, float Kd, uint32_t dt_ms = 1, float min_output = -1023.0f, float max_output = 1023.0f);

      /**
       * @brief Initialize the PD controller
       *
       * @return esp_err_t ESP_OK on success, ESP_FAIL otherwise
       */
      esp_err_t init() override;

      /**
       * @brief Reset the controller state (previous error)
       */
      void reset() override;

      /**
       * @brief Calculate PD output based on error
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
       * @brief Set the derivative gain
       *
       * @param Kd: Derivative gain
       */
      void setKd(float Kd);

      /**
       * @brief Set both PD gains
       *
       * @param Kp: Proportional gain
       * @param Kd: Derivative gain
       */
      void setGains(float Kp, float Kd);

      /**
       * @brief Get the proportional gain
       *
       * @return float Proportional gain
       */
      float getKp() const;

      /**
       * @brief Get the derivative gain
       *
       * @return float Derivative gain
       */
      float getKd() const;
  };

} // namespace controller