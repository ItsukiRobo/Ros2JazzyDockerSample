#include "signal_utility/high_pass_filter.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace signal_utility
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

void validate_cutoff_frequency(double cutoff_frequency_hz)
{
  if (cutoff_frequency_hz < 0.0) {
    throw std::runtime_error("cutoff_frequency_hz must be non-negative");
  }
}

void validate_sampling_period(double sampling_period_s)
{
  if (sampling_period_s < 0.0) {
    throw std::runtime_error("sampling_period_s must be non-negative");
  }
}

}  // namespace

BilinearHighPassFilter::BilinearHighPassFilter(double cutoff_frequency_hz, double sampling_period_s)
: cutoff_frequency_hz_(cutoff_frequency_hz),
  sampling_period_s_(sampling_period_s),
  previous_input_(0.0),
  previous_output_(0.0),
  value_(std::numeric_limits<double>::quiet_NaN()),
  initialized_(false)
{
  validate_cutoff_frequency(cutoff_frequency_hz_);
  validate_sampling_period(sampling_period_s_);
}

void BilinearHighPassFilter::set_cutoff_frequency(double cutoff_frequency_hz)
{
  validate_cutoff_frequency(cutoff_frequency_hz);
  cutoff_frequency_hz_ = cutoff_frequency_hz;
}

void BilinearHighPassFilter::set_sampling_period(double sampling_period_s)
{
  validate_sampling_period(sampling_period_s);
  sampling_period_s_ = sampling_period_s;
}

void BilinearHighPassFilter::reset(double initial_value)
{
  previous_input_ = initial_value;
  previous_output_ = 0.0;
  value_ = 0.0;
  initialized_ = true;
}

void BilinearHighPassFilter::clear()
{
  previous_input_ = 0.0;
  previous_output_ = 0.0;
  value_ = std::numeric_limits<double>::quiet_NaN();
  initialized_ = false;
}

double BilinearHighPassFilter::update(double input)
{
  if (!initialized_ || cutoff_frequency_hz_ == 0.0 || sampling_period_s_ == 0.0) {
    reset(input);
    return value_;
  }

  const double k = std::tan(kPi * cutoff_frequency_hz_ * sampling_period_s_);
  const double norm = 1.0 / (1.0 + k);
  const double a0 = norm;
  const double a1 = -a0;
  const double b1 = (k - 1.0) * norm;

  value_ = a0 * input + a1 * previous_input_ - b1 * previous_output_;
  previous_input_ = input;
  previous_output_ = value_;
  return value_;
}

}  // namespace signal_utility
