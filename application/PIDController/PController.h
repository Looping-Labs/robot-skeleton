#pragma once

#include "BaseController.h"

namespace controller {

  /**
   * @brief Proportional-Only Controller implementation
   * 
   * Simplest controller that uses only proportional action.
   * Fast response but may have steady-state error.
   * Suitable for simple applications where some error is acceptable.
   */
  class PController : public BaseController {
    private:
      /**
       * @brief Proportional gain
       * 
       * @var Kp: Proportional gain
       */
      float Kp;            

    public:
      /**
       * @brief Construct a new P Controller
       * 
       * @param Kp: Proportional gain
       * @param dt_ms: Time step in milliseconds (used for consistency with other controllers)
       * @param min_output: Minimum output value (default -1023)
       * @param max_output: Maximum output value (default 1023)
       */
      PController(float Kp, uint32_t dt_ms = 1, float min_output = -1023.0f, float max_output = 1023.0f);
      
      /**
       * @brief Initialize the P controller
       * 
       * @return esp_err_t ESP_OK on success, ESP_FAIL otherwise
       */
      esp_err_t init() override;
      
      /**
       * @brief Reset the controller state (nothing to reset for P controller)
       */
      void reset() override;
      
      /**
       * @brief Calculate P output based on error
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
       * @brief Get the proportional gain
       * 
       * @return float Proportional gain
       */
      float getKp() const;
  };

} // namespace controller