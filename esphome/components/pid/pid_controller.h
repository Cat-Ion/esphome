#pragma once

#include <math.h>
#include "esphome/core/hal.h"

namespace esphome {
namespace pid {

struct PIDController {
  float update(float setpoint, float process_value) {
    // e(t) ... error at timestamp t
    // r(t) ... setpoint
    // y(t) ... process value (sensor reading)
    // u(t) ... output value

    float dt = calculate_relative_time_();

    // e(t) := r(t) - y(t)
    error = setpoint - process_value;

    // p(t) := K_p * e(t)
    proportional_term = kp * error;

    // i(t) := K_i * \int_{0}^{t} e(t) dt
    accumulated_integral_ += error * dt * ki;
    integral_term = accumulated_integral_;

    // d(t) := K_d * de(t)/dt
    float derivative = 0.0f;
    if (dt != 0.0f)
      derivative = (process_value - previous_value_) / dt;
    derivative_term = kd * derivative;

    // constrain output: u(t) := p(t) + i(t) + d(t)
    float output = proportional_term + integral_term + derivative_term;
    if (!std::isnan(min_output) && output < min_output) {
      accumulated_integral_ -= min_output - output;
      output = min_output;
    } else if (!std::isnan(max_output) && output > max_output) {
      accumulated_integral_ -= max_output - output;
      output = max_output;
    }

    previous_value_ = process_value;
    previous_error_ = error;

    return output;
  }

  void reset_accumulated_integral() { accumulated_integral_ = 0; }

  void set_kp(float v) {
    const float diff_out = (v - kp) * previous_error_;
    this->accumulated_integral_ -= diff_out;
    this->kp = v;
  }
  void set_ki(float v) { this->ki = v; }
  void set_kd(float v) { this->kd = v; }

  /// Proportional gain K_p.
  float kp = 0;
  /// Integral gain K_i.
  float ki = 0;
  /// Differential gain K_d.
  float kd = 0;

  float min_output = NAN;
  float max_output = NAN;

  // Store computed values in struct so that values can be monitored through sensors
  float error;
  float proportional_term;
  float integral_term;
  float derivative_term;

 protected:
  float calculate_relative_time_() {
    uint32_t now = millis();
    uint32_t dt = now - this->last_time_;
    if (last_time_ == 0) {
      last_time_ = now;
      return 0.0f;
    }
    last_time_ = now;
    return dt / 1000.0f;
  }

  /// Error and value from previous update used for derivative term
  float previous_error_ = 0;
  float previous_value_ = 0;
  /// Accumulated integral value
  float accumulated_integral_ = 0;
  uint32_t last_time_ = 0;
};

}  // namespace pid
}  // namespace esphome
