#include "controller/pid.hpp"

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <string>

namespace controller
{

Pid::Pid(double kp, double ki, double kd, double sampling_period_s)
: kp_(kp),
  ki_(ki),
  kd_(kd),
  sampling_period_s_(sampling_period_s),
  min_output_(0.0),
  max_output_(0.0),
  output_limits_enabled_(false),
  min_integral_(0.0),
  max_integral_(0.0),
  integral_limits_enabled_(false),
  integral_(0.0),
  previous_error_(0.0),
  initialized_(false)
{
  validate_sampling_period(sampling_period_s_);
}

void Pid::set_gains(double kp, double ki, double kd)
{
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
}

void Pid::set_sampling_period(double sampling_period_s)
{
  validate_sampling_period(sampling_period_s);
  sampling_period_s_ = sampling_period_s;
}

void Pid::set_output_limits(double min_output, double max_output)
{
  validate_limit_pair(min_output, max_output, "output_limits");
  min_output_ = min_output;
  max_output_ = max_output;
  output_limits_enabled_ = true;
}

void Pid::clear_output_limits()
{
  output_limits_enabled_ = false;
}

void Pid::set_integral_limits(double min_integral, double max_integral)
{
  validate_limit_pair(min_integral, max_integral, "integral_limits");
  min_integral_ = min_integral;
  max_integral_ = max_integral;
  integral_limits_enabled_ = true;
}

void Pid::clear_integral_limits()
{
  integral_limits_enabled_ = false;
}

void Pid::reset()
{
  integral_ = 0.0;
  previous_error_ = 0.0;
  initialized_ = false;
}

double Pid::update(double setpoint, double measurement)
{
  return update_from_error(setpoint - measurement);
}

double Pid::update_from_error(double error)
{
  if (!initialized_) {
    previous_error_ = error;
    initialized_ = true;
  }

  double derivative = 0.0;
  if (sampling_period_s_ > 0.0) {
    derivative = (error - previous_error_) / sampling_period_s_;
  }
  // Anti-windup of conditional integrals
  double next_integral = integral_;
  if (sampling_period_s_ > 0.0) {
    next_integral += error * sampling_period_s_;
  }
  if (integral_limits_enabled_) {
    next_integral = clamp(next_integral, min_integral_, max_integral_);
  }

  bool allow_integral_update = true;
  if (output_limits_enabled_) {
    const double candidate_output = kp_ * error + ki_ * next_integral + kd_ * derivative;
    if ((candidate_output > max_output_ && error > 0.0) ||
      (candidate_output < min_output_ && error < 0.0))
    {
      allow_integral_update = false;
    }
  }

  if (allow_integral_update) {
    integral_ = next_integral;
  }

  double output = kp_ * error + ki_ * integral_ + kd_ * derivative;
  if (output_limits_enabled_) {
    output = clamp(output, min_output_, max_output_);
  }

  previous_error_ = error;
  return output;
}

void Pid::validate_sampling_period(double sampling_period_s)
{
  if (sampling_period_s < 0.0) {
    throw std::runtime_error("sampling_period_s must be non-negative");
  }
}

void Pid::validate_limit_pair(double min_value, double max_value, const char * name)
{
  if (min_value > max_value) {
    throw std::runtime_error(std::string(name) + ": min must be less than or equal to max");
  }
}

double Pid::clamp(double value, double min_value, double max_value)
{
  return std::max(min_value, std::min(value, max_value));
}

}  // namespace controller
