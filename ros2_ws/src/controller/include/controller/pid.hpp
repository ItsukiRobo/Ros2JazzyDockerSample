#ifndef CONTROLLER__PID_HPP_
#define CONTROLLER__PID_HPP_

namespace controller
{

class Pid
{
public:
  Pid(double kp = 0.0, double ki = 0.0, double kd = 0.0, double sampling_period_s = 0.0);

  void set_gains(double kp, double ki, double kd);
  void set_sampling_period(double sampling_period_s);
  void set_output_limits(double min_output, double max_output);
  void clear_output_limits();
  void set_integral_limits(double min_integral, double max_integral);
  void clear_integral_limits();
  void reset();

  double update(double setpoint, double measurement);
  double update_from_error(double error);

private:
  static void validate_sampling_period(double sampling_period_s);
  static void validate_limit_pair(double min_value, double max_value, const char * name);
  static double clamp(double value, double min_value, double max_value);

  double kp_;
  double ki_;
  double kd_;
  double sampling_period_s_;

  double min_output_;
  double max_output_;
  bool output_limits_enabled_;

  double min_integral_;
  double max_integral_;
  bool integral_limits_enabled_;

  double integral_;
  double previous_error_;
  bool initialized_;
};

}  // namespace controller

#endif  // CONTROLLER__PID_HPP_
