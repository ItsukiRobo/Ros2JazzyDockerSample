#include "controller/action/track_sine_force.hpp"
#include "controller/action/track_sine_length.hpp"
#include "controller/pid.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kDefaultControlPeriodS = 0.01;
constexpr double kCylinderDiameterM = 0.020;
constexpr double kRodDiameterM = 0.010;
constexpr double kMaxPressureKPa = 1000.0;
constexpr double kMinPressureKPa = 0.0;
constexpr double kVoltageOffsetV = 5.0;
constexpr double kMinCommandVoltageV = -5.0;
constexpr double kMaxCommandVoltageV = 5.0;
constexpr double kMinVoltageV = 0.0;
constexpr double kMaxVoltageV = 10.0;
constexpr size_t kOutputChannelCount = 8;
constexpr double kHeadAreaM2 = kPi * kCylinderDiameterM * kCylinderDiameterM / 4.0;
constexpr double kRodAreaM2 =
  kHeadAreaM2 - (kPi * kRodDiameterM * kRodDiameterM / 4.0);

static_assert(kRodAreaM2 > 0.0, "rod area must be smaller than cylinder area");

double clamp(double value, double min_value, double max_value)
{
  return std::max(min_value, std::min(value, max_value));
}

double command_to_output_voltage(double command_voltage_v)
{
  return clamp(command_voltage_v + kVoltageOffsetV, kMinVoltageV, kMaxVoltageV);
}

double pressure_from_force_kpa(double force_n, double area_m2)
{
  if (area_m2 <= 0.0) {
    throw std::runtime_error("area_m2 must be positive");
  }
  return force_n / area_m2 / 1000.0;
}

enum class ControlMode : int32_t
{
  kHold = 0,
  kForceSineTracking = 1,
  kLengthSineTracking = 2,
};

}  // namespace

class CylinderForceControllerNode : public rclcpp::Node
{
public:
  using TrackSineForce = controller::action::TrackSineForce;
  using GoalHandleTrackSineForce = rclcpp_action::ServerGoalHandle<TrackSineForce>;
  using TrackSineLength = controller::action::TrackSineLength;
  using GoalHandleTrackSineLength = rclcpp_action::ServerGoalHandle<TrackSineLength>;

  CylinderForceControllerNode()
  : Node("cylinder_controller")
  {
    this->declare_parameter<std::string>("subscribe_topic_name", "/pressure_force_and_length");
    this->declare_parameter<std::string>("publish_topic_name", "/controller/output_voltage");
    this->declare_parameter<std::string>(
      "debug_publish_topic_name", "/debug/cylinder_controller/targets");
    this->declare_parameter<std::string>(
      "action_name", "/cylinder_controller/track_sine_force");
    this->declare_parameter<std::string>(
      "length_action_name", "/cylinder_controller/track_sine_length");
    this->declare_parameter<double>("control_period_s", kDefaultControlPeriodS);
    this->declare_parameter<int>("head_pressure_index", 0);
    this->declare_parameter<int>("rod_pressure_index", 1);
    this->declare_parameter<int>("force_index", 6);
    this->declare_parameter<int>("length_index", 7);
    this->declare_parameter<double>("kp", 0.02);
    this->declare_parameter<double>("ki", 0.0);
    this->declare_parameter<double>("kd", 0.0);
    this->declare_parameter<double>("force_kp", 0.1);
    this->declare_parameter<double>("force_ki", 0.0);
    this->declare_parameter<double>("force_kd", 0.0);
    this->declare_parameter<double>("force_output_limit_n", 200.0);
    this->declare_parameter<double>("length_kp", 0.1);
    this->declare_parameter<double>("length_ki", 0.0);
    this->declare_parameter<double>("length_kd", 0.0);
    this->declare_parameter<double>("base_pressure_kpa", 50.0);
    this->declare_parameter<double>("startup_target_force_n", 0.0);
    this->declare_parameter<double>("feasible_force_min_n", -200.0);
    this->declare_parameter<double>("feasible_force_max_n", 0.0);

    subscribe_topic_name_ = this->get_parameter("subscribe_topic_name").as_string();
    publish_topic_name_ = this->get_parameter("publish_topic_name").as_string();
    debug_publish_topic_name_ = this->get_parameter("debug_publish_topic_name").as_string();
    force_action_name_ = this->get_parameter("action_name").as_string();
    length_action_name_ = this->get_parameter("length_action_name").as_string();
    control_period_s_ = this->get_parameter("control_period_s").as_double();
    head_pressure_index_ = this->get_parameter("head_pressure_index").as_int();
    rod_pressure_index_ = this->get_parameter("rod_pressure_index").as_int();
    force_index_ = this->get_parameter("force_index").as_int();
    length_index_ = this->get_parameter("length_index").as_int();
    base_pressure_kpa_ = this->get_parameter("base_pressure_kpa").as_double();
    hold_target_force_n_ = this->get_parameter("startup_target_force_n").as_double();
    feasible_force_min_n_ = this->get_parameter("feasible_force_min_n").as_double();
    feasible_force_max_n_ = this->get_parameter("feasible_force_max_n").as_double();

    const double kp = this->get_parameter("kp").as_double();
    const double ki = this->get_parameter("ki").as_double();
    const double kd = this->get_parameter("kd").as_double();
    const double force_kp = this->get_parameter("force_kp").as_double();
    const double force_ki = this->get_parameter("force_ki").as_double();
    const double force_kd = this->get_parameter("force_kd").as_double();
    const double force_output_limit_n = this->get_parameter("force_output_limit_n").as_double();
    const double length_kp = this->get_parameter("length_kp").as_double();
    const double length_ki = this->get_parameter("length_ki").as_double();
    const double length_kd = this->get_parameter("length_kd").as_double();

    validate_parameters(force_output_limit_n);

    head_pid_.set_gains(kp, ki, kd);
    head_pid_.set_sampling_period(control_period_s_);
    head_pid_.set_output_limits(kMinCommandVoltageV, kMaxCommandVoltageV);

    rod_pid_.set_gains(kp, ki, kd);
    rod_pid_.set_sampling_period(control_period_s_);
    rod_pid_.set_output_limits(kMinCommandVoltageV, kMaxCommandVoltageV);

    force_pid_.set_gains(force_kp, force_ki, force_kd);
    force_pid_.set_sampling_period(control_period_s_);
    force_pid_.set_output_limits(-force_output_limit_n, force_output_limit_n);

    length_pid_.set_gains(length_kp, length_ki, length_kd);
    length_pid_.set_sampling_period(control_period_s_);
    length_pid_.set_output_limits(feasible_force_min_n_, feasible_force_max_n_);

    publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
      publish_topic_name_, rclcpp::QoS(10));
    debug_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
      debug_publish_topic_name_, rclcpp::QoS(10));
    subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      subscribe_topic_name_, rclcpp::QoS(10),
      std::bind(&CylinderForceControllerNode::measurement_callback, this, std::placeholders::_1));

    force_action_server_ = rclcpp_action::create_server<TrackSineForce>(
      this, force_action_name_,
      std::bind(
        &CylinderForceControllerNode::handle_force_goal, this, std::placeholders::_1,
        std::placeholders::_2),
      std::bind(&CylinderForceControllerNode::handle_force_cancel, this, std::placeholders::_1),
      std::bind(&CylinderForceControllerNode::handle_force_accepted, this, std::placeholders::_1));

    length_action_server_ = rclcpp_action::create_server<TrackSineLength>(
      this, length_action_name_,
      std::bind(
        &CylinderForceControllerNode::handle_length_goal, this, std::placeholders::_1,
        std::placeholders::_2),
      std::bind(&CylinderForceControllerNode::handle_length_cancel, this, std::placeholders::_1),
      std::bind(&CylinderForceControllerNode::handle_length_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "cylinder_controller started. subscribe='%s' force_action='%s' length_action='%s'",
      subscribe_topic_name_.c_str(), force_action_name_.c_str(), length_action_name_.c_str());
  }

private:
  struct TargetPressures
  {
    double head_kpa;
    double rod_kpa;
  };

  struct SineTrajectory
  {
    double amplitude;
    double offset;
    double frequency_hz;
    double duration_s;
    double phase_rad;
    std::chrono::steady_clock::time_point start_time;
  };

  void validate_parameters(double force_output_limit_n)
  {
    if (control_period_s_ < 0.0) {
      throw std::runtime_error("control_period_s must be non-negative");
    }
    if (head_pressure_index_ < 0 || rod_pressure_index_ < 0 || force_index_ < 0 || length_index_ < 0) {
      throw std::runtime_error(
              "head_pressure_index, rod_pressure_index, force_index, length_index must be non-negative");
    }
    if (!std::isfinite(base_pressure_kpa_) || base_pressure_kpa_ < 0.0) {
      throw std::runtime_error("base_pressure_kpa must be finite and non-negative");
    }
    if (!std::isfinite(hold_target_force_n_)) {
      throw std::runtime_error("startup_target_force_n must be finite");
    }
    if (!std::isfinite(force_output_limit_n) || force_output_limit_n <= 0.0) {
      throw std::runtime_error("force_output_limit_n must be finite and positive");
    }
    if (!std::isfinite(feasible_force_min_n_) || !std::isfinite(feasible_force_max_n_)) {
      throw std::runtime_error("feasible_force_min_n and feasible_force_max_n must be finite");
    }
    if (feasible_force_min_n_ > feasible_force_max_n_) {
      throw std::runtime_error("feasible_force_min_n must be less than or equal to feasible_force_max_n");
    }
    if (feasible_force_max_n_ > 0.0) {
      throw std::runtime_error("feasible_force_max_n must be <= 0.0 for this mechanism");
    }
    hold_target_force_n_ = clamp(hold_target_force_n_, feasible_force_min_n_, feasible_force_max_n_);
  }

  static double mode_to_number(ControlMode mode)
  {
    return static_cast<double>(static_cast<int32_t>(mode));
  }

  static bool is_finite_measurement(double value)
  {
    return std::isfinite(value);
  }

  TargetPressures convert_force_to_target_pressures(double commanded_force_n) const
  {
    TargetPressures target{base_pressure_kpa_, base_pressure_kpa_};
    const double clamped_force_n =
      clamp(commanded_force_n, feasible_force_min_n_, feasible_force_max_n_);
    if (clamped_force_n > 0.0) {
      target.head_kpa += pressure_from_force_kpa(clamped_force_n, kHeadAreaM2);
    } else if (clamped_force_n < 0.0) {
      target.rod_kpa += pressure_from_force_kpa(-clamped_force_n, kRodAreaM2);
    }

    target.head_kpa = clamp(target.head_kpa, kMinPressureKPa, kMaxPressureKPa);
    target.rod_kpa = clamp(target.rod_kpa, kMinPressureKPa, kMaxPressureKPa);
    return target;
  }

  void reset_all_pids()
  {
    head_pid_.reset();
    rod_pid_.reset();
    force_pid_.reset();
    length_pid_.reset();
  }

  void update_sampling_period(const std::chrono::steady_clock::time_point & now)
  {
    if (last_measurement_time_.has_value()) {
      const double dt_seconds = std::chrono::duration<double>(now - *last_measurement_time_).count();
      head_pid_.set_sampling_period(dt_seconds);
      rod_pid_.set_sampling_period(dt_seconds);
      force_pid_.set_sampling_period(dt_seconds);
      length_pid_.set_sampling_period(dt_seconds);
    }
    last_measurement_time_ = now;
  }

  static double compute_sine_target(
    const SineTrajectory & trajectory,
    const std::chrono::steady_clock::time_point & now)
  {
    const double elapsed_s = std::chrono::duration<double>(now - trajectory.start_time).count();
    return trajectory.offset +
           trajectory.amplitude *
           std::sin(2.0 * kPi * trajectory.frequency_hz * elapsed_s + trajectory.phase_rad);
  }

  double current_desired_force_n(const std::chrono::steady_clock::time_point & now)
  {
    switch (control_mode_) {
      case ControlMode::kForceSineTracking:
        if (active_force_sine_.has_value()) {
          return clamp(
            compute_sine_target(*active_force_sine_, now),
            feasible_force_min_n_, feasible_force_max_n_);
        }
        break;
      case ControlMode::kLengthSineTracking:
        if (active_length_sine_.has_value()) {
          target_length_mm_ = compute_sine_target(*active_length_sine_, now);
          return clamp(
            length_pid_.update(target_length_mm_, measured_length_mm_),
            feasible_force_min_n_, feasible_force_max_n_);
        }
        break;
      case ControlMode::kHold:
        break;
    }
    target_length_mm_ = measured_length_mm_;
    return hold_target_force_n_;
  }

  void publish_command(
    double head_output_voltage_v,
    double rod_output_voltage_v,
    double desired_force_n,
    double commanded_force_n,
    const TargetPressures & target_pressures)
  {
    std_msgs::msg::Float32MultiArray out_msg;
    out_msg.data.assign(kOutputChannelCount, 5.0f);
    out_msg.data[static_cast<size_t>(head_pressure_index_)] =
      static_cast<float>(head_output_voltage_v);
    out_msg.data[static_cast<size_t>(rod_pressure_index_)] =
      static_cast<float>(rod_output_voltage_v);
    publisher_->publish(out_msg);

    std_msgs::msg::Float32MultiArray debug_msg;
    debug_msg.data.push_back(static_cast<float>(mode_to_number(control_mode_)));
    debug_msg.data.push_back(static_cast<float>(hold_target_force_n_));
    debug_msg.data.push_back(static_cast<float>(desired_force_n));
    debug_msg.data.push_back(static_cast<float>(commanded_force_n));
    debug_msg.data.push_back(static_cast<float>(measured_force_n_));
    debug_msg.data.push_back(static_cast<float>(target_length_mm_));
    debug_msg.data.push_back(static_cast<float>(measured_length_mm_));
    debug_msg.data.push_back(static_cast<float>(target_pressures.head_kpa));
    debug_msg.data.push_back(static_cast<float>(target_pressures.rod_kpa));
    debug_msg.data.push_back(static_cast<float>(measured_head_pressure_kpa_));
    debug_msg.data.push_back(static_cast<float>(measured_rod_pressure_kpa_));
    debug_publisher_->publish(debug_msg);
  }

  void measurement_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);

    const size_t head_index = static_cast<size_t>(head_pressure_index_);
    const size_t rod_index = static_cast<size_t>(rod_pressure_index_);
    const size_t force_index = static_cast<size_t>(force_index_);
    const size_t length_index = static_cast<size_t>(length_index_);
    if (
      head_index >= msg->data.size() || rod_index >= msg->data.size() ||
      force_index >= msg->data.size() || length_index >= msg->data.size())
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "measurement indices (%zu, %zu, %zu, %zu) are out of range for size %zu",
        head_index, rod_index, force_index, length_index, msg->data.size());
      return;
    }

    const double head_pressure_kpa = msg->data[head_index];
    const double rod_pressure_kpa = msg->data[rod_index];
    const double measured_force_n = msg->data[force_index];
    const double measured_length_mm = msg->data[length_index];
    if (
      !is_finite_measurement(head_pressure_kpa) ||
      !is_finite_measurement(rod_pressure_kpa) ||
      !is_finite_measurement(measured_force_n) ||
      !is_finite_measurement(measured_length_mm))
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "latest pressure/force/length measurement is not finite");
      return;
    }

    const auto now = std::chrono::steady_clock::now();
    update_sampling_period(now);

    measured_head_pressure_kpa_ = head_pressure_kpa;
    measured_rod_pressure_kpa_ = rod_pressure_kpa;
    measured_force_n_ = measured_force_n;
    measured_length_mm_ = measured_length_mm;

    const double desired_force_n = current_desired_force_n(now);
    const double force_correction_n = force_pid_.update(desired_force_n, measured_force_n_);
    const double commanded_force_n = clamp(
      desired_force_n + force_correction_n,
      feasible_force_min_n_, feasible_force_max_n_);
    const TargetPressures target_pressures = convert_force_to_target_pressures(commanded_force_n);

    const double head_command_voltage_v =
      head_pid_.update(target_pressures.head_kpa, measured_head_pressure_kpa_);
    const double rod_command_voltage_v =
      rod_pid_.update(target_pressures.rod_kpa, measured_rod_pressure_kpa_);
    const double head_output_voltage_v = command_to_output_voltage(head_command_voltage_v);
    const double rod_output_voltage_v = command_to_output_voltage(rod_command_voltage_v);

    desired_force_n_ = desired_force_n;
    publish_command(
      head_output_voltage_v, rod_output_voltage_v, desired_force_n_, commanded_force_n,
      target_pressures);
  }

  bool validate_common_sine_goal(
    double amplitude, double frequency_hz, double duration_s, double phase_rad) const
  {
    return std::isfinite(amplitude) && amplitude >= 0.0 &&
           std::isfinite(frequency_hz) && frequency_hz >= 0.0 &&
           std::isfinite(duration_s) && duration_s > 0.0 &&
           std::isfinite(phase_rad);
  }

  bool validate_force_sine_range(double amplitude_n, double offset_n) const
  {
    const double min_force_n = offset_n - amplitude_n;
    const double max_force_n = offset_n + amplitude_n;
    return min_force_n >= feasible_force_min_n_ && max_force_n <= feasible_force_max_n_;
  }

  rclcpp_action::GoalResponse handle_force_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const TrackSineForce::Goal> goal) const
  {
    if (!std::isfinite(goal->offset_n)) {
      RCLCPP_WARN(this->get_logger(), "Rejecting force goal: offset_n must be finite");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (
      !validate_common_sine_goal(
        goal->amplitude_n, goal->frequency_hz, goal->duration_s, goal->phase_rad))
    {
      RCLCPP_WARN(this->get_logger(), "Rejecting force goal: invalid amplitude/frequency/duration/phase");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (!validate_force_sine_range(goal->amplitude_n, goal->offset_n)) {
      RCLCPP_WARN(this->get_logger(), "Rejecting force goal: requested force waveform is outside feasible range");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_force_cancel(
    const std::shared_ptr<GoalHandleTrackSineForce> goal_handle)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (active_force_goal_ == goal_handle) {
      active_force_goal_.reset();
      active_force_sine_.reset();
      control_mode_ = ControlMode::kHold;
      reset_all_pids();
      last_measurement_time_.reset();
    }
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_force_accepted(const std::shared_ptr<GoalHandleTrackSineForce> goal_handle)
  {
    std::thread(
      std::bind(&CylinderForceControllerNode::execute_force_goal, this, std::placeholders::_1),
      goal_handle).detach();
  }

  void execute_force_goal(const std::shared_ptr<GoalHandleTrackSineForce> goal_handle)
  {
    std::shared_ptr<GoalHandleTrackSineForce> previous_force_goal;
    std::shared_ptr<GoalHandleTrackSineLength> previous_length_goal;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      previous_force_goal = active_force_goal_;
      previous_length_goal = active_length_goal_;
      const auto goal = goal_handle->get_goal();
      active_force_goal_ = goal_handle;
      active_length_goal_.reset();
      active_force_sine_ = SineTrajectory{
        goal->amplitude_n,
        goal->offset_n,
        goal->frequency_hz,
        goal->duration_s,
        goal->phase_rad,
        std::chrono::steady_clock::now()};
      active_length_sine_.reset();
      control_mode_ = ControlMode::kForceSineTracking;
      reset_all_pids();
      last_measurement_time_.reset();
    }

    if (previous_force_goal && previous_force_goal != goal_handle) {
      auto result = std::make_shared<TrackSineForce::Result>();
      result->success = false;
      result->message = "Preempted by a newer force goal";
      previous_force_goal->abort(result);
    }
    if (previous_length_goal) {
      auto result = std::make_shared<TrackSineLength::Result>();
      result->success = false;
      result->message = "Preempted by a force goal";
      previous_length_goal->abort(result);
    }

    using namespace std::chrono_literals;
    while (rclcpp::ok()) {
      auto feedback = std::make_shared<TrackSineForce::Feedback>();
      bool should_succeed = false;
      bool should_cancel = false;
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (active_force_goal_ != goal_handle || control_mode_ != ControlMode::kForceSineTracking) {
          return;
        }

        if (!active_force_sine_.has_value()) {
          return;
        }

        const auto now = std::chrono::steady_clock::now();
        feedback->elapsed_time_s =
          std::chrono::duration<double>(now - active_force_sine_->start_time).count();
        feedback->target_force_n = clamp(
          compute_sine_target(*active_force_sine_, now),
          feasible_force_min_n_, feasible_force_max_n_);
        feedback->measured_force_n = measured_force_n_;

        if (goal_handle->is_canceling()) {
          active_force_goal_.reset();
          active_force_sine_.reset();
          control_mode_ = ControlMode::kHold;
          reset_all_pids();
          last_measurement_time_.reset();
          should_cancel = true;
        } else if (feedback->elapsed_time_s >= active_force_sine_->duration_s) {
          active_force_goal_.reset();
          active_force_sine_.reset();
          control_mode_ = ControlMode::kHold;
          reset_all_pids();
          last_measurement_time_.reset();
          should_succeed = true;
        }
      }

      goal_handle->publish_feedback(feedback);

      if (should_cancel) {
        auto result = std::make_shared<TrackSineForce::Result>();
        result->success = false;
        result->message = "Canceled";
        goal_handle->canceled(result);
        return;
      }
      if (should_succeed) {
        auto result = std::make_shared<TrackSineForce::Result>();
        result->success = true;
        result->message = "Completed force sine tracking";
        goal_handle->succeed(result);
        return;
      }

      std::this_thread::sleep_for(20ms);
    }
  }

  rclcpp_action::GoalResponse handle_length_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const TrackSineLength::Goal> goal) const
  {
    if (!std::isfinite(goal->offset_mm)) {
      RCLCPP_WARN(this->get_logger(), "Rejecting length goal: offset_mm must be finite");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (
      !validate_common_sine_goal(
        goal->amplitude_mm, goal->frequency_hz, goal->duration_s, goal->phase_rad))
    {
      RCLCPP_WARN(this->get_logger(), "Rejecting length goal: invalid amplitude/frequency/duration/phase");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_length_cancel(
    const std::shared_ptr<GoalHandleTrackSineLength> goal_handle)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (active_length_goal_ == goal_handle) {
      active_length_goal_.reset();
      active_length_sine_.reset();
      control_mode_ = ControlMode::kHold;
      reset_all_pids();
      last_measurement_time_.reset();
    }
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_length_accepted(const std::shared_ptr<GoalHandleTrackSineLength> goal_handle)
  {
    std::thread(
      std::bind(&CylinderForceControllerNode::execute_length_goal, this, std::placeholders::_1),
      goal_handle).detach();
  }

  void execute_length_goal(const std::shared_ptr<GoalHandleTrackSineLength> goal_handle)
  {
    std::shared_ptr<GoalHandleTrackSineForce> previous_force_goal;
    std::shared_ptr<GoalHandleTrackSineLength> previous_length_goal;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      previous_force_goal = active_force_goal_;
      previous_length_goal = active_length_goal_;
      const auto goal = goal_handle->get_goal();
      active_length_goal_ = goal_handle;
      active_force_goal_.reset();
      active_length_sine_ = SineTrajectory{
        goal->amplitude_mm,
        goal->offset_mm,
        goal->frequency_hz,
        goal->duration_s,
        goal->phase_rad,
        std::chrono::steady_clock::now()};
      active_force_sine_.reset();
      control_mode_ = ControlMode::kLengthSineTracking;
      reset_all_pids();
      last_measurement_time_.reset();
    }

    if (previous_force_goal) {
      auto result = std::make_shared<TrackSineForce::Result>();
      result->success = false;
      result->message = "Preempted by a length goal";
      previous_force_goal->abort(result);
    }
    if (previous_length_goal && previous_length_goal != goal_handle) {
      auto result = std::make_shared<TrackSineLength::Result>();
      result->success = false;
      result->message = "Preempted by a newer length goal";
      previous_length_goal->abort(result);
    }

    using namespace std::chrono_literals;
    while (rclcpp::ok()) {
      auto feedback = std::make_shared<TrackSineLength::Feedback>();
      bool should_succeed = false;
      bool should_cancel = false;
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (active_length_goal_ != goal_handle || control_mode_ != ControlMode::kLengthSineTracking) {
          return;
        }

        if (!active_length_sine_.has_value()) {
          return;
        }

        const auto now = std::chrono::steady_clock::now();
        feedback->elapsed_time_s =
          std::chrono::duration<double>(now - active_length_sine_->start_time).count();
        feedback->target_length_mm = compute_sine_target(*active_length_sine_, now);
        feedback->measured_length_mm = measured_length_mm_;

        if (goal_handle->is_canceling()) {
          active_length_goal_.reset();
          active_length_sine_.reset();
          control_mode_ = ControlMode::kHold;
          reset_all_pids();
          last_measurement_time_.reset();
          should_cancel = true;
        } else if (feedback->elapsed_time_s >= active_length_sine_->duration_s) {
          active_length_goal_.reset();
          active_length_sine_.reset();
          control_mode_ = ControlMode::kHold;
          reset_all_pids();
          last_measurement_time_.reset();
          should_succeed = true;
        }
      }

      goal_handle->publish_feedback(feedback);

      if (should_cancel) {
        auto result = std::make_shared<TrackSineLength::Result>();
        result->success = false;
        result->message = "Canceled";
        goal_handle->canceled(result);
        return;
      }
      if (should_succeed) {
        auto result = std::make_shared<TrackSineLength::Result>();
        result->success = true;
        result->message = "Completed length sine tracking";
        goal_handle->succeed(result);
        return;
      }

      std::this_thread::sleep_for(20ms);
    }
  }

  controller::Pid head_pid_;
  controller::Pid rod_pid_;
  controller::Pid force_pid_;
  controller::Pid length_pid_;
  std::string subscribe_topic_name_;
  std::string publish_topic_name_;
  std::string debug_publish_topic_name_;
  std::string force_action_name_;
  std::string length_action_name_;
  double control_period_s_{kDefaultControlPeriodS};
  int head_pressure_index_{0};
  int rod_pressure_index_{1};
  int force_index_{0};
  int length_index_{0};
  double base_pressure_kpa_{50.0};
  double hold_target_force_n_{0.0};
  double feasible_force_min_n_{-200.0};
  double feasible_force_max_n_{0.0};
  double measured_head_pressure_kpa_{0.0};
  double measured_rod_pressure_kpa_{0.0};
  double measured_force_n_{0.0};
  double measured_length_mm_{0.0};
  double desired_force_n_{0.0};
  double target_length_mm_{0.0};
  ControlMode control_mode_{ControlMode::kHold};
  std::optional<SineTrajectory> active_force_sine_;
  std::optional<SineTrajectory> active_length_sine_;
  std::optional<std::chrono::steady_clock::time_point> last_measurement_time_;
  std::shared_ptr<GoalHandleTrackSineForce> active_force_goal_;
  std::shared_ptr<GoalHandleTrackSineLength> active_length_goal_;
  std::mutex state_mutex_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr debug_publisher_;
  rclcpp_action::Server<TrackSineForce>::SharedPtr force_action_server_;
  rclcpp_action::Server<TrackSineLength>::SharedPtr length_action_server_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CylinderForceControllerNode>());
  rclcpp::shutdown();
  return 0;
}
