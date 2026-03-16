#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/float32_multi_array.hpp"

#include <algorithm>
#include <functional>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

enum class PseType
{
  k101kPa,
  k1MPa,
};

class PressureSensorNode : public rclcpp::Node
{
public:
  PressureSensorNode()
  : Node("pressure_sensor")
  {
    this->declare_parameter<std::string>("subscribe_topic_name", "ai1616llpe/voltage");
    this->declare_parameter<std::vector<int64_t>>("sensor_idx", {0});
    this->declare_parameter<std::vector<std::string>>("sensor_type_str", {"101kPa"});
    this->declare_parameter<std::string>("publish_topic_name", "/pressure");

    this->get_parameter("subscribe_topic_name", subscribe_topic_name_);
    this->get_parameter("publish_topic_name", publish_topic_name_);

    const auto sensor_idx_param = this->get_parameter("sensor_idx").as_integer_array();
    const auto sensor_type_param = this->get_parameter("sensor_type_str").as_string_array();

    if (sensor_idx_param.empty()) {
      throw std::runtime_error("sensor_idx must not be empty");
    }
    if (sensor_idx_param.size() != sensor_type_param.size()) {
      throw std::runtime_error("sensor_idx and sensor_type_str must have the same length");
    }

    sensor_indices_.reserve(sensor_idx_param.size());
    sensor_types_.reserve(sensor_type_param.size());

    for (size_t i = 0; i < sensor_idx_param.size(); ++i) {
      if (sensor_idx_param[i] < 0) {
        throw std::runtime_error("sensor_idx must contain only non-negative values");
      }
      sensor_indices_.push_back(static_cast<size_t>(sensor_idx_param[i]));
      sensor_types_.push_back(parse_sensor_type(sensor_type_param[i]));
    }

    publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
      publish_topic_name_, rclcpp::QoS(10));
    subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      subscribe_topic_name_, rclcpp::QoS(10),
      std::bind(&PressureSensorNode::topic_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "pressure_sensor subscribes to '%s' and publishes to '%s'",
      subscribe_topic_name_.c_str(), publish_topic_name_.c_str());
  }

private:
  static constexpr float kVoltageMin = 1.0f;
  static constexpr float kVoltageMax = 5.0f;
  static constexpr float kRange101kPa = 101.0f;
  static constexpr float kRange1MPa = 1000.0f;

  static PseType parse_sensor_type(const std::string & sensor_type)
  {
    if (sensor_type == "101kPa") {
      return PseType::k101kPa;
    }
    if (sensor_type == "1MPa") {
      return PseType::k1MPa;
    }
    throw std::runtime_error("sensor_type_str must contain only \"101kPa\" or \"1MPa\"");
  }

  static float clamp01(float value)
  {
    return std::max(0.0f, std::min(1.0f, value));
  }

  float convert_voltage_to_pressure(float voltage, PseType sensor_type) const
  {
    // Assume a linear 1-5 V analog output scaled to each sensor's full range.
    const float normalized = clamp01((voltage - kVoltageMin) / (kVoltageMax - kVoltageMin));
    switch (sensor_type) {
      case PseType::k101kPa:
        return normalized * kRange101kPa;
      case PseType::k1MPa:
        return normalized * kRange1MPa;
    }
    return std::numeric_limits<float>::quiet_NaN();
  }

  void topic_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    std_msgs::msg::Float32MultiArray out_msg;
    out_msg.data.reserve(sensor_indices_.size());

    for (size_t i = 0; i < sensor_indices_.size(); ++i) {
      const size_t input_index = sensor_indices_[i];
      if (input_index >= msg->data.size()) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "sensor_idx[%zu]=%zu is out of range for input size %zu",
          i, input_index, msg->data.size());
        out_msg.data.push_back(std::numeric_limits<float>::quiet_NaN());
        continue;
      }

      out_msg.data.push_back(
        convert_voltage_to_pressure(msg->data[input_index], sensor_types_[i]));
    }

    publisher_->publish(out_msg);
  }

  std::string subscribe_topic_name_;
  std::string publish_topic_name_;
  std::vector<size_t> sensor_indices_;
  std::vector<PseType> sensor_types_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PressureSensorNode>());
  rclcpp::shutdown();
  return 0;
}
