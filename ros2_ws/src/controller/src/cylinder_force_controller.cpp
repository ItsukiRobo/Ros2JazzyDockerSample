#include "controller/action/track_sine_force.hpp"
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
  kSineTracking = 1,
};

}  // namespace

class CylinderForceControllerNode : public rclcpp::Node
{
public:
  using TrackSineForce = controller::action::TrackSineForce;
  using GoalHandleTrackSineForce = rclcpp_action::ServerGoalHandle<TrackSineForce>;

  CylinderForceControllerNode()
  : Node("cylinder_force_controller")
  {
    this->declare_parameter<std::string>("subscribe_topic_name", "/pressure_and_force");
    this->declare_parameter<std::string>("publish_topic_name", "/controller/output_voltage");
    this->declare_parameter<std::string>(
      "debug_publish_topic_name", "/debug/cylinder_force_controller/targets");
    this->declare_parameter<std::string>(
      "action_name", "/cylinder_force_controller/track_sine_force");
    this->declare_parameter<double>("control_period_s", kDefaultControlPeriodS);
    this->declare_parameter<int>("head_pressure_index", 0);
    this->declare_parameter<int>("rod_pressure_index", 1);
    this->declare_parameter<int>("force_index", 6);
    this->declare_parameter<double>("kp", 0.02);
    this->declare_parameter<double>("ki", 0.0);
    this->declare_parameter<double>("kd", 0.0);
    this->declare_parameter<double>("force_kp", 0.1);
    this->declare_parameter<double>("force_ki", 0.0);
    this->declare_parameter<double>("force_kd", 0.0);
    this->declare_parameter<double>("force_output_limit_n", 200.0);
    this->declare_parameter<double>("base_pressure_kpa", 50.0);
    this->declare_parameter<double>("startup_target_force_n", 0.0);

    subscribe_topic_name_ = this->get_parameter("subscribe_topic_name").as_string();
    publish_topic_name_ = this->get_parameter("publish_topic_name").as_string();
    debug_publish_topic_name_ = this->get_parameter("debug_publish_topic_name").as_string();
    action_name_ = this->get_parameter("action_name").as_string();
    control_period_s_ = this->get_parameter("control_period_s").as_double();
    head_pressure_index_ = this->get_parameter("head_pressure_index").as_int();
    rod_pressure_index_ = this->get_parameter("rod_pressure_index").as_int();
    force_index_ = this->get_parameter("force_index").as_int();
    base_pressure_kpa_ = this->get_parameter("base_pressure_kpa").as_double();
    hold_target_force_n_ = this->get_parameter("startup_target_force_n").as_double();

    const double kp = this->get_parameter("kp").as_double();
    const double ki = this->get_parameter("ki").as_double();
    const double kd = this->get_parameter("kd").as_double();
    const double force_kp = this->get_parameter("force_kp").as_double();
    const double force_ki = this->get_parameter("force_ki").as_double();
    const double force_kd = this->get_parameter("force_kd").as_double();
    const double force_output_limit_n = this->get_parameter("force_output_limit_n").as_double();

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

    publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
      publish_topic_name_, rclcpp::QoS(10));
    debug_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
      debug_publish_topic_name_, rclcpp::QoS(10));
    subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      subscribe_topic_name_, rclcpp::QoS(10),
      std::bind(&CylinderForceControllerNode::measurement_callback, this, std::placeholders::_1));

    action_server_ = rclcpp_action::create_server<TrackSineForce>(
      this, action_name_,
      std::bind(&CylinderForceControllerNode::handle_goal, this, std::placeholders::_1,
      std::placeholders::_2),
      std::bind(&CylinderForceControllerNode::handle_cancel, this, std::placeholders::_1),
      std::bind(&CylinderForceControllerNode::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "cylinder_force_controller started. subscribe='%s' publish='%s' action='%s' hold_force=%.3f N",
      subscribe_topic_name_.c_str(), publish_topic_name_.c_str(), action_name_.c_str(),
      hold_target_force_n_);
  }

private:
  struct TargetPressures
  {
    double head_kpa;
    double rod_kpa;
  };

  struct SineTrajectory
  {
    double amplitude_n;
    double offset_n;
    double frequency_hz;
    double duration_s;
    double phase_rad;
    std::chrono::steady_clock::time_point start_time;
  };

  void validate_parameters(double force_output_limit_n) const
  {
    if (control_period_s_ < 0.0) {
      throw std::runtime_error("control_period_s must be non-negative");
    }
    if (head_pressure_index_ < 0 || rod_pressure_index_ < 0 || force_index_ < 0) {
      throw std::runtime_error("head_pressure_index, rod_pressure_index, force_index must be non-negative");
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
    if (commanded_force_n > 0.0) {
      target.head_kpa += pressure_from_force_kpa(commanded_force_n, kHeadAreaM2);
    } else if (commanded_force_n < 0.0) {
      target.rod_kpa += pressure_from_force_kpa(-commanded_force_n, kRodAreaM2);
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
  }

  void update_sampling_period(const std::chrono::steady_clock::time_point & now)
  {
    if (last_measurement_time_.has_value()) {
      const double dt_seconds = std::chrono::duration<double>(now - *last_measurement_time_).count();
      head_pid_.set_sampling_period(dt_seconds);
      rod_pid_.set_sampling_period(dt_seconds);
      force_pid_.set_sampling_period(dt_seconds);
    }
    last_measurement_time_ = now;
  }

  double compute_sine_target_force_n(
    const SineTrajectory & trajectory,
    const std::chrono::steady_clock::time_point & now) const
  {
    const double elapsed_s = std::chrono::duration<double>(now - trajectory.start_time).count();
    return trajectory.offset_n +
           trajectory.amplitude_n *
           std::sin(2.0 * kPi * trajectory.frequency_hz * elapsed_s + trajectory.phase_rad);
  }

  double current_target_force_n(const std::chrono::steady_clock::time_point & now) const
  {
    if (control_mode_ == ControlMode::kSineTracking && active_sine_.has_value()) {
      return compute_sine_target_force_n(*active_sine_, now);
    }
    return hold_target_force_n_;
  }

  void publish_command(
    double head_output_voltage_v,
    double rod_output_voltage_v,
    double target_force_n,
    double commanded_force_n,
    double measured_force_n,
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
    debug_msg.data.push_back(static_cast<float>(target_force_n));
    debug_msg.data.push_back(static_cast<float>(commanded_force_n));
    debug_msg.data.push_back(static_cast<float>(measured_force_n));
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
    if (head_index >= msg->data.size() || rod_index >= msg->data.size() || force_index >= msg->data.size()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "measurement indices (%zu, %zu, %zu) are out of range for size %zu",
        head_index, rod_index, force_index, msg->data.size());
      return;
    }

    const double head_pressure_kpa = msg->data[head_index];
    const double rod_pressure_kpa = msg->data[rod_index];
    const double measured_force_n = msg->data[force_index];
    if (!is_finite_measurement(head_pressure_kpa) ||
      !is_finite_measurement(rod_pressure_kpa) ||
      !is_finite_measurement(measured_force_n))
    {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "latest pressure/force measurement is not finite");
      return;
    }

    const auto now = std::chrono::steady_clock::now();
    update_sampling_period(now);

    measured_head_pressure_kpa_ = head_pressure_kpa;
    measured_rod_pressure_kpa_ = rod_pressure_kpa;
    measured_force_n_ = measured_force_n;

    const double target_force_n = current_target_force_n(now);
    const double force_correction_n = force_pid_.update(target_force_n, measured_force_n_);
    const double commanded_force_n = target_force_n + force_correction_n;
    const TargetPressures target_pressures = convert_force_to_target_pressures(commanded_force_n);

    const double head_command_voltage_v =
      head_pid_.update(target_pressures.head_kpa, measured_head_pressure_kpa_);
    const double rod_command_voltage_v =
      rod_pid_.update(target_pressures.rod_kpa, measured_rod_pressure_kpa_);
    const double head_output_voltage_v = command_to_output_voltage(head_command_voltage_v);
    const double rod_output_voltage_v = command_to_output_voltage(rod_command_voltage_v);

    last_target_force_n_ = target_force_n;
    publish_command(
      head_output_voltage_v, rod_output_voltage_v, target_force_n,
      commanded_force_n, measured_force_n_, target_pressures);
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const TrackSineForce::Goal> goal) const
  {
    if (!std::isfinite(goal->amplitude_n) || goal->amplitude_n < 0.0) {
      RCLCPP_WARN(this->get_logger(), "Rejecting goal: amplitude_n must be finite and non-negative");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (!std::isfinite(goal->offset_n)) {
      RCLCPP_WARN(this->get_logger(), "Rejecting goal: offset_n must be finite");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (!std::isfinite(goal->frequency_hz) || goal->frequency_hz < 0.0) {
      RCLCPP_WARN(this->get_logger(), "Rejecting goal: frequency_hz must be finite and non-negative");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (!std::isfinite(goal->duration_s) || goal->duration_s <= 0.0) {
      RCLCPP_WARN(this->get_logger(), "Rejecting goal: duration_s must be finite and positive");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (!std::isfinite(goal->phase_rad)) {
      RCLCPP_WARN(this->get_logger(), "Rejecting goal: phase_rad must be finite");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleTrackSineForce> goal_handle)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (active_goal_ == goal_handle) {
      control_mode_ = ControlMode::kHold;
      active_sine_.reset();
      reset_all_pids();
      last_measurement_time_.reset();
    }
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleTrackSineForce> goal_handle)
  {
    std::thread(
      std::bind(&CylinderForceControllerNode::execute_goal, this, std::placeholders::_1),
      goal_handle).detach();
  }

  void execute_goal(const std::shared_ptr<GoalHandleTrackSineForce> goal_handle)
  {
    std::shared_ptr<GoalHandleTrackSineForce> previous_goal;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      previous_goal = active_goal_;
      active_goal_ = goal_handle;
      const auto goal = goal_handle->get_goal();
      active_sine_ = SineTrajectory{
        goal->amplitude_n,
        goal->offset_n,
        goal->frequency_hz,
        goal->duration_s,
        goal->phase_rad,
        std::chrono::steady_clock::now()};
      control_mode_ = ControlMode::kSineTracking;
      reset_all_pids();
      last_measurement_time_.reset();
    }

    if (previous_goal && previous_goal != goal_handle) {
      auto result = std::make_shared<TrackSineForce::Result>();
      result->success = false;
      result->message = "Preempted by a newer goal";
      previous_goal->abort(result);
    }

    using namespace std::chrono_literals;
    while (rclcpp::ok()) {
      auto feedback = std::make_shared<TrackSineForce::Feedback>();
      bool should_succeed = false;
      bool should_cancel = false;
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (active_goal_ != goal_handle) {
          return;
        }

        const auto now = std::chrono::steady_clock::now();
        const double target_force_n = current_target_force_n(now);
        feedback->target_force_n = target_force_n;
        feedback->measured_force_n = measured_force_n_;

        if (goal_handle->is_canceling()) {
          feedback->elapsed_time_s = 0.0;
          active_goal_.reset();
          active_sine_.reset();
          control_mode_ = ControlMode::kHold;
          reset_all_pids();
          last_measurement_time_.reset();
          should_cancel = true;
        } else if (active_sine_.has_value()) {
          feedback->elapsed_time_s =
            std::chrono::duration<double>(now - active_sine_->start_time).count();
          if (feedback->elapsed_time_s >= active_sine_->duration_s) {
            active_goal_.reset();
            active_sine_.reset();
            control_mode_ = ControlMode::kHold;
            reset_all_pids();
            last_measurement_time_.reset();
            should_succeed = true;
          }
        } else {
          return;
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
        result->message = "Completed sine tracking";
        goal_handle->succeed(result);
        return;
      }

      std::this_thread::sleep_for(20ms);
    }
  }

  controller::Pid head_pid_;
  controller::Pid rod_pid_;
  controller::Pid force_pid_;
  std::string subscribe_topic_name_;
  std::string publish_topic_name_;
  std::string debug_publish_topic_name_;
  std::string action_name_;
  double control_period_s_{kDefaultControlPeriodS};
  int head_pressure_index_{0};
  int rod_pressure_index_{1};
  int force_index_{0};
  double base_pressure_kpa_{50.0};
  double hold_target_force_n_{0.0};
  double measured_head_pressure_kpa_{0.0};
  double measured_rod_pressure_kpa_{0.0};
  double measured_force_n_{0.0};
  double last_target_force_n_{0.0};
  ControlMode control_mode_{ControlMode::kHold};
  std::optional<SineTrajectory> active_sine_;
  std::optional<std::chrono::steady_clock::time_point> last_measurement_time_;
  std::shared_ptr<GoalHandleTrackSineForce> active_goal_;
  std::mutex state_mutex_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr debug_publisher_;
  rclcpp_action::Server<TrackSineForce>::SharedPtr action_server_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CylinderForceControllerNode>());
  rclcpp::shutdown();
  return 0;
}
