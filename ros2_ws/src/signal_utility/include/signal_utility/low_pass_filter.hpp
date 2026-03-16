#ifndef SIGNAL_UTILITY__LOW_PASS_FILTER_HPP_
#define SIGNAL_UTILITY__LOW_PASS_FILTER_HPP_

namespace signal_utility
{

class BilinearLowPassFilter
{
public:
  BilinearLowPassFilter(double cutoff_frequency_hz = 0.0, double sampling_period_s = 0.0);

  void set_cutoff_frequency(double cutoff_frequency_hz);
  void set_sampling_period(double sampling_period_s);

  void reset(double initial_value);
  void clear();
  double update(double input);

private:
  double cutoff_frequency_hz_;
  double sampling_period_s_;
  double previous_input_;
  double previous_output_;
  double value_;
  bool initialized_;
};

class FormulaLowPassFilter
{
public:
  FormulaLowPassFilter(double cutoff_frequency_hz = 0.0, double sampling_period_s = 0.0);

  void set_cutoff_frequency(double cutoff_frequency_hz);
  void set_sampling_period(double sampling_period_s);

  void reset(double initial_value);
  void clear();
  double update(double input);

private:
  double cutoff_frequency_hz_;
  double sampling_period_s_;
  double value_;
  bool initialized_;
};

}  // namespace signal_utility

#endif  // SIGNAL_UTILITY__LOW_PASS_FILTER_HPP_
