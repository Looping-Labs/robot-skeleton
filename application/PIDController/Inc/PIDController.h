#pragma once

#include "BaseController.h"

namespace controller {

  /**
   * @brief Full Proportional-Integral-Derivative Controller implementation
   * 
   * This controller uses all three terms (P, I, and D) to provide precise control
   * with fast response time, minimal steady-state error, and good stability.
   * Suitable for complex systems requiring high performance.
   */
  class PIDController : public BaseController {
    private:
      /**
       * @brief PID controller parameters
       * 
       * @var Kp: Proportional gain
       * @var Ki: Integral gain
       * @var Kd: Derivative gain
       * @var integral: Accumulated error (integral term)
       * @var prev_error: Previous error (for derivative term)
       * @var anti_windup: Anti-windup limit for integral term
       */
      float Kp;            
      float Ki;               
      float Kd;                 
      float integral;      
      float prev_error;    
      float anti_windup;   

    public:
      /**
       * @brief Construct a new PID Controller
       * 
       * @param Kp: Proportional gain
       * @param Ki: Integral gain
       * @param Kd: Derivative gain
       * @param dt_ms: Time step in milliseconds
       * @param min_output: Minimum output value (default -1023)
       * @param max_output: Maximum output value (default 1023)
       */
      PIDController(float Kp, float Ki, float Kd, uint32_t dt_ms = 1,
                  float min_output = -1023.0f, float max_output = 1023.0f);
      
      /**
       * @brief Initialize the PID controller
       * 
       * @return esp_err_t ESP_OK on success, ESP_FAIL otherwise
       */
      esp_err_t init() override;
      
      /**
       * @brief Reset the controller state (integral and previous error)
       */
      void reset() override;
      
      /**
       * @brief Calculate PID output based on error
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
       * @brief Set the derivative gain
       * 
       * @param Kd: Derivative gain
       */
      void setKd(float Kd);
      
      /**
       * @brief Set all PID gains
       * 
       * @param Kp: Proportional gain
       * @param Ki: Integral gain
       * @param Kd: Derivative gain
       */
      void setGains(float Kp, float Ki, float Kd);
      
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
       * @brief Get the derivative gain
       * 
       * @return float Derivative gain
       */
      float getKd() const;
      
      /**
       * @brief Get the integral term
       * 
       * @return float Integral term
       */
      float getIntegral() const;
  };
} // namespace controller