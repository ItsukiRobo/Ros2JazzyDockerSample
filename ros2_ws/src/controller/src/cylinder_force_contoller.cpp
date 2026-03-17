#include "controller/pid.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <optional>
#include <stdexcept>
#include <string>

class CylinderForceContollerNode : public rclcpp::Node
{
public:
  CylinderForceContollerNode()
  : Node("cylinder_force_contoller")
  {
    this->declare_parameter<std::string>("subscribe_topic_name", "/pressure");
    this->declare_parameter<std::string>("publish_topic_name", "/controller/output_voltage");
    this->declare_parameter<std::string>(
      "debug_publish_topic_name", "/debug/cylinder_force_contoller/targets");
    this->declare_parameter<int>("head_pressure_index", 0);
    this->declare_parameter<int>("rod_pressure_index", 1);
    this->declare_parameter<double>("control_period_s", 0.01);
    this->declare_parameter<double>("kp", 0.02);
    this->declare_parameter<double>("ki", 0.0);
    this->declare_parameter<double>("kd", 0.0);

    subscribe_topic_name_ = this->get_parameter("subscribe_topic_name").as_string();
    publish_topic_name_ = this->get_parameter("publish_topic_name").as_string();
    debug_publish_topic_name_ = this->get_parameter("debug_publish_topic_name").as_string();
    head_pressure_index_ = this->get_parameter("head_pressure_index").as_int();
    rod_pressure_index_ = this->get_parameter("rod_pressure_index").as_int();

    control_period_s_ = this->get_parameter("control_period_s").as_double();
    const double kp = this->get_parameter("kp").as_double();
    const double ki = this->get_parameter("ki").as_double();
    const double kd = this->get_parameter("kd").as_double();

    if (head_pressure_index_ < 0 || rod_pressure_index_ < 0) {
      throw std::runtime_error("pressure indices must be non-negative");
    }
    if (head_pressure_index_ >= 8 || rod_pressure_index_ >= 8) {
      throw std::runtime_error("pressure indices must be smaller than 8");
    }
    if (control_period_s_ <= 0.0) {
      throw std::runtime_error("control_period_s must be positive");
    }

    head_pid_.set_gains(kp, ki, kd);
    head_pid_.set_sampling_period(control_period_s_);
    head_pid_.set_output_limits(kMinCommandVoltageV, kMaxCommandVoltageV);

    rod_pid_.set_gains(kp, ki, kd);
    rod_pid_.set_sampling_period(control_period_s_);
    rod_pid_.set_output_limits(kMinCommandVoltageV, kMaxCommandVoltageV);

    publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
      publish_topic_name_, rclcpp::QoS(10));
    debug_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
      debug_publish_topic_name_, rclcpp::QoS(10));
    subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      subscribe_topic_name_, rclcpp::QoS(10),
      std::bind(&CylinderForceContollerNode::pressure_callback, this, std::placeholders::_1));

    start_time_ = std::chrono::steady_clock::now();

    RCLCPP_INFO(
      this->get_logger(),
      "cylinder_force_contoller started. subscribe='%s' publish='%s' debug_publish='%s' pid_period=%.4f s",
      subscribe_topic_name_.c_str(), publish_topic_name_.c_str(),
      debug_publish_topic_name_.c_str(), control_period_s_);
  }

private:
  struct TargetPressures
  {
    double head_kpa;
    double rod_kpa;
  };

  static constexpr double kPi = 3.14159265358979323846;
  static constexpr double kCylinderDiameterM = 0.020;
  static constexpr double kRodDiameterM = 0.010;
  static constexpr double kBasePressureKPa = 50.0;
  static constexpr double kMaxPressureKPa = 1000.0;
  static constexpr double kMinPressureKPa = 0.0;

  static constexpr double kVoltageOffsetV = 5.0;
  static constexpr double kMinCommandVoltageV = -5.0;
  static constexpr double kMaxCommandVoltageV = 5.0;
  
  static constexpr double kMinVoltageV = 0.0;
  static constexpr double kMaxVoltageV = 10.0;
  static constexpr double kTargetForceAmplitudeN = 10.0;
  static constexpr double kTargetForceFrequencyHz = 0.1;
  static constexpr size_t kOutputChannelCount = 8;

  static double clamp(double value, double min_value, double max_value)
  {
    return std::max(min_value, std::min(value, max_value));
  }

  static double command_to_output_voltage(double command_voltage_v)
  {
    return clamp(command_voltage_v + kVoltageOffsetV, kMinVoltageV, kMaxVoltageV);
  }
  static constexpr double kHeadAreaM2 =
    kPi * kCylinderDiameterM * kCylinderDiameterM / 4.0;
  static constexpr double kRodAreaM2 =
    kHeadAreaM2 - (kPi * kRodDiameterM * kRodDiameterM / 4.0);
  static_assert(kRodAreaM2 > 0.0, "rod area must be smaller than cylinder area");

  static double pressure_from_force_kpa(double force_n, double area_m2)
  {
    if (area_m2 <= 0.0) {
      throw std::runtime_error("area_m2 must be positive");
    }
    return force_n / area_m2 / 1000.0;
  }

  static TargetPressures convert_force_to_target_pressures(double target_force_n)
  {
    TargetPressures target{kBasePressureKPa, kBasePressureKPa};
    if (target_force_n > 0.0) {
      target.head_kpa += pressure_from_force_kpa(target_force_n, kHeadAreaM2);
    } else if (target_force_n < 0.0) {
      target.rod_kpa += pressure_from_force_kpa(-target_force_n, kRodAreaM2);
    }

    target.head_kpa = clamp(target.head_kpa, kMinPressureKPa, kMaxPressureKPa);
    target.rod_kpa = clamp(target.rod_kpa, kMinPressureKPa, kMaxPressureKPa);
    return target;
  }

  void pressure_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    const auto now = std::chrono::steady_clock::now();
    if (last_pressure_time_.has_value()) {
      const double dt_seconds = std::chrono::duration<double>(now - *last_pressure_time_).count();
      head_pid_.set_sampling_period(dt_seconds);
      rod_pid_.set_sampling_period(dt_seconds);
    }
    last_pressure_time_ = now;

    const size_t head_index = static_cast<size_t>(head_pressure_index_);
    const size_t rod_index = static_cast<size_t>(rod_pressure_index_);

    if (head_index >= msg->data.size() || rod_index >= msg->data.size()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "pressure indices (%zu, %zu) are out of range for pressure size %zu",
        head_index, rod_index, msg->data.size());
      return;
    }

    const double head_pressure_kpa = msg->data[head_index];
    const double rod_pressure_kpa = msg->data[rod_index];
    if (!std::isfinite(head_pressure_kpa) || !std::isfinite(rod_pressure_kpa)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "latest pressure measurement is not finite");
      return;
    }

    measured_head_pressure_kpa_ = head_pressure_kpa;
    measured_rod_pressure_kpa_ = rod_pressure_kpa;
    control_callback();
  }

  void control_callback()
  {
    const double elapsed_s =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time_).count();
    const double target_force_n =
      kTargetForceAmplitudeN * std::sin(2.0 * kPi * kTargetForceFrequencyHz * elapsed_s);
    const TargetPressures target_pressures = convert_force_to_target_pressures(target_force_n);

    const double head_command_voltage_v =
      head_pid_.update(target_pressures.head_kpa, measured_head_pressure_kpa_);
    const double rod_command_voltage_v =
      rod_pid_.update(target_pressures.rod_kpa, measured_rod_pressure_kpa_);
    const double head_output_voltage_v = command_to_output_voltage(head_command_voltage_v);
    const double rod_output_voltage_v = command_to_output_voltage(rod_command_voltage_v);

    std_msgs::msg::Float32MultiArray out_msg;
    out_msg.data.assign(kOutputChannelCount, 0.0f);
    out_msg.data[static_cast<size_t>(head_pressure_index_)] =
      static_cast<float>(head_output_voltage_v);
    out_msg.data[static_cast<size_t>(rod_pressure_index_)] =
      static_cast<float>(rod_output_voltage_v);
    publisher_->publish(out_msg);

    std_msgs::msg::Float32MultiArray debug_msg;
    debug_msg.data.push_back(static_cast<float>(target_force_n));
    debug_msg.data.push_back(static_cast<float>(target_pressures.head_kpa));
    debug_msg.data.push_back(static_cast<float>(target_pressures.rod_kpa));
    debug_publisher_->publish(debug_msg);
  }

  controller::Pid head_pid_;
  controller::Pid rod_pid_;
  std::string subscribe_topic_name_;
  std::string publish_topic_name_;
  std::string debug_publish_topic_name_;
  double control_period_s_{0.01};
  int head_pressure_index_{0};
  int rod_pressure_index_{1};
  double measured_head_pressure_kpa_{0.0};
  double measured_rod_pressure_kpa_{0.0};
  std::chrono::steady_clock::time_point start_time_{};
  std::optional<std::chrono::steady_clock::time_point> last_pressure_time_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr debug_publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CylinderForceContollerNode>());
  rclcpp::shutdown();
  return 0;
}
