#pragma once

#include "esp_err.h"
#include "stdint.h"

namespace PIDController {
  /**
   * @brief PID Controller class for various control systems
   *
   * This class implements a generic PID controller that can be used for various
   * control applications such as line following, motor speed control, etc.
   * It supports PI, PD, or full PID modes and includes anti-windup protection.
   * 
   * @param Kp: Proportional gain
   * @param Ki: Integral gain
   * @param Kd: Derivative gain
   * @param setpoint: Desired setpoint value
   * @return: Output value of the controller in the range [-1023, 1023]
   */

  class PIDController {
    private:
      /*PID parameters*/
      float Kp;
      float Ki;
      float Kd;
      float dt;

      /* PID state variables */
      float setpoint;
      float prev_error;
      float integral;
      float output;

      /* PID configuration */
      float min_output;
      float max_output;
      float anti_windup;

      /* Mode flags */
      bool use_p;
      bool use_i;
      bool use_d;

    public:
      /**
       * @brief Construct a new PID Controller object
       * 
       * @param Kp: Proportional gain
       * @param Ki: Integral gain
       * @param Kd: Derivative gain
       * @param dt_ms: Time step in milliseconds
       * @param min_output: Minimum output value (default: -1023)
       * @param max_output: Maximum output value (default: 1023)
       */

      PIDController(float Kp, float Ki, float Kd, float dt_ms, float min_output = -1023.0f, float max_output = 1023.0f);

      /**
       * @brief Initialize the PID controller
       * 
       * @return esp_err_t: ESP_OK if successful, ESP_FAIL otherwise
       */

      esp_err_t init();

      /**
       * @brief Reset the PID controller state (integral, previous error)
       */

      void reset();

      /**
       * @brief Calculate the PID output based on the current error
       * 
       * @param error: Current error value (setpoint - measured value)
       * @return float Control output value between min_output and max_output
       */
      float compute(float error);

      /**
       * @brief Calculate PID output based on setpoint and measured value
       * 
       * @param measured_value: Current measured value
       * @return float Control output value between min_output and max_output
       */

      float computeWithSetpoint(float measured_value);

      /**
       * @brief Set the PID mode (P, PI, PD, PID)
       * 
       * @param p_on: Enable proportional term
       * @param i_on: Enable integral term
       * @param d_on: Enable derivative term
       */
      void setMode(bool p_on, bool i_on, bool d_on);

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
       * @brief Set the sample time for the PID controller
       * 
       * @param dt_ms: Sample time in milliseconds
       */
      void setSampleTime(uint32_t dt_ms);

      /**
       * @brief Set the output limits for the PID controller
       * 
       * @param min_output: Minimum output value
       * @param max_output: Maximum output value 
       */
      void setOutputLimits(float min_output, float max_output);

      /**
       * @brief Set the anti-windup limit for the PID controller
       * 
       * @param limit: Anti-windup limit
       */
      void setAntiWindupLimit(float limit);

      /**
       * @brief Set the setpoint value for the PID controller
       * 
       * @param setpoint: Desired setpoint value
       */
      void setSetpoint(float setpoint);

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
       * @brief Get the sample time
       * 
       * @return uint32_t Sample time in milliseconds
       */
      uint32_t getSampleTime() const;

      /**
       * @brief Get the Setpoint value
       * 
       * @return float Setpoint value
       */
      float getSetpoint() const;
      
      /**
       * @brief Get the last output value
       * 
       * @return float Output value
       */
      float getOutput() const;

      /**
       * @brief Get the integral term
       * 
       * @return float Integral term
       */
      float getIntegral() const;
  };
}