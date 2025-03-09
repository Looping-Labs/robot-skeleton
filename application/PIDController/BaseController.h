#pragma once

#include "esp_err.h"
#include <stdint.h>

namespace controller {
  /**
   * @brief Base Controller class for different control strategies
   *
   * Abstract base class that defines the common interface and functionality
   * for all controller types (P, PI, PD, PID).
   */
  class BaseController {
    protected:
      /**
       * @brief Common controller parameters
       * 
       * @var setpoint: Desired target value
       * @var output: Controller output
       * @var dt: Time step in seconds (converted from milliseconds)
       * @var min_output: Minimum output value
       * @var max_output: Maximum output value
       */
      float setpoint;       
      float output;   
      float dt;       
      float min_output; 
      float max_output; 

      /**
       * @brief Apply limits to a value
       *
       * @param value: Value to limit
       * @param min: Minimum allowed value
       * @param max: Maximum allowed value
       * @return float Limited value
       */
      float applyLimits(float value, float min, float max) const;

    public:
      /**
       * @brief Construct a new Base Controller
       *
       * @param dt_ms: Time step in milliseconds (default 1 ms)
       * @param min_output: Minimum output value (default -1023)
       * @param max_output: Maximum output value (default 1023)
       */
      BaseController(uint32_t dt_ms = 1, float min_output = -1023.0f, float max_output = 1023.0f);

      /**
       * @brief Virtual destructor for proper cleanup in derived classes
       */
      virtual ~BaseController() = default;

      /**
       * @brief Initialize the controller
       *
       * @return esp_err_t ESP_OK on success, ESP_FAIL otherwise
       */
      virtual esp_err_t init();

      /**
       * @brief Reset the controller state
       */
      virtual void reset() = 0;

      /**
       * @brief Calculate controller output based on error
       *
       * @param error: Current error (setpoint - measured_value)
       * @return float Controller output between min_output and max_output
       */
      virtual float compute(float error) = 0;

      /**
       * @brief Calculate controller output based on setpoint and measured value
       *
       * @param measured_value: Current measured value
       * @return float Controller output between min_output and max_output
       */
      float computeWithSetpoint(float measured_value);

      /**
       * @brief Set the sample time
       *
       * @param dt_ms: Time step in milliseconds
       */
      void setSampleTime(uint32_t dt_ms);

      /**
       * @brief Set the output limits
       *
       * @param min_output: Minimum output value
       * @param max_output: Maximum output value
       */
      void setOutputLimits(float min_output, float max_output);

      /**
       * @brief Set the setpoint
       *
       * @param setpoint: Desired target value
       */
      void setSetpoint(float setpoint);

      /**
       * @brief Get the sample time
       *
       * @return float Time step in seconds
       */
      float getSampleTime() const;

      /**
       * @brief Get the setpoint
       *
       * @return float Setpoint
       */
      float getSetpoint() const;

      /**
       * @brief Get the last output
       *
       * @return float Last output
       */
      float getOutput() const;
  };
} // namespace controller