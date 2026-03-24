#include "signal_utility/low_pass_filter.hpp"

#include "peripheral/srv/zero_loadcell.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <functional>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

enum class PressureSensorType
{
  k101kPa,
  k1MPa,
};

constexpr float kPressureVoltageMin = 1.0f;
constexpr float kPressureVoltageMax = 5.0f;
constexpr float kPressureRange101kPa = 101.0f;
constexpr float kPressureRange1MPa = 1000.0f;
constexpr double kLoadcellGainNPerV = 7.918593157 * 9.807;

float clamp01(float value)
{
  return std::max(0.0f, std::min(1.0f, value));
}

PressureSensorType parse_pressure_sensor_type(const std::string & sensor_type)
{
  if (sensor_type == "101kPa") {
    return PressureSensorType::k101kPa;
  }
  if (sensor_type == "1MPa") {
    return PressureSensorType::k1MPa;
  }
  throw std::runtime_error(
          "pressure_sensor_type_str must contain only \"101kPa\" or \"1MPa\"");
}

/**
 * @brief pressure sensor conversion and filtering
 */
class PressureSensor
{
public:
  PressureSensor(
    size_t sensor_index, PressureSensorType sensor_type, double cutoff_frequency_hz)
  : sensor_index_(sensor_index),
    sensor_type_(sensor_type),
    lpf_(cutoff_frequency_hz, 0.0)
  {
  }

  size_t sensor_index() const
  {
    return sensor_index_;
  }

  float convert(float voltage, const std::optional<double> & sampling_period_s)
  {
    if (sampling_period_s.has_value()) {
      lpf_.set_sampling_period(*sampling_period_s);
    }

    const float normalized =
      clamp01((voltage - kPressureVoltageMin) / (kPressureVoltageMax - kPressureVoltageMin));
    float pressure = std::numeric_limits<float>::quiet_NaN();
    switch (sensor_type_) {
      case PressureSensorType::k101kPa:
        pressure = normalized * kPressureRange101kPa;
        break;
      case PressureSensorType::k1MPa:
        pressure = normalized * kPressureRange1MPa;
        break;
    }
    return static_cast<float>(lpf_.update(pressure));
  }

private:
  size_t sensor_index_;
  PressureSensorType sensor_type_;
  signal_utility::BilinearLowPassFilter lpf_;
};

/**
 * @brief loadcell conversion and filtering
*/
class Loadcell
{
public:
  Loadcell(
    size_t signal_plus_index, size_t signal_minus_index, double cutoff_frequency_hz,
    double zero_balance_voltage_v)
  : signal_plus_index_(signal_plus_index),
    signal_minus_index_(signal_minus_index),
    zero_balance_voltage_v_(zero_balance_voltage_v),
    lpf_(cutoff_frequency_hz, 0.0)
  {
  }

  size_t signal_plus_index() const
  {
    return signal_plus_index_;
  }

  size_t signal_minus_index() const
  {
    return signal_minus_index_;
  }

  void zero_from_current_measurement(float signal_plus_voltage, float signal_minus_voltage)
  {
    zero_balance_voltage_v_ =
      static_cast<double>(signal_plus_voltage) - static_cast<double>(signal_minus_voltage);
  }

  float convert(
    float signal_plus_voltage, float signal_minus_voltage,
    const std::optional<double> & sampling_period_s)
  {
    if (sampling_period_s.has_value()) {
      lpf_.set_sampling_period(*sampling_period_s);
    }

    const double differential_voltage =
      static_cast<double>(signal_plus_voltage) - static_cast<double>(signal_minus_voltage) -
      zero_balance_voltage_v_;
    const double force_n = differential_voltage * kLoadcellGainNPerV;
    return static_cast<float>(lpf_.update(force_n));
  }

private:
  size_t signal_plus_index_;
  size_t signal_minus_index_;
  double zero_balance_voltage_v_;
  signal_utility::BilinearLowPassFilter lpf_;
};

}  // namespace

class PressureAndForceNode : public rclcpp::Node
{
public:
  PressureAndForceNode()
  : Node("pressure_and_force")
  {
    this->declare_parameter<std::string>("subscribe_topic_name", "ai1616llpe/voltage");
    this->declare_parameter<std::string>("publish_topic_name", "pressure_and_force");
    this->declare_parameter<std::vector<int64_t>>("pressure_sensor_idx", {0});
    this->declare_parameter<std::vector<std::string>>("pressure_sensor_type_str", {"101kPa"});
    this->declare_parameter<double>("pressure_cutoff_frequency_hz", 10.0);
    this->declare_parameter<std::vector<int64_t>>("loadcell_signal_plus_idx", {6});
    this->declare_parameter<std::vector<int64_t>>("loadcell_signal_minus_idx", {7});
    this->declare_parameter<std::vector<double>>("loadcell_cutoff_frequency_hz", {10.0});
    this->declare_parameter<std::vector<double>>("loadcell_zero_balance_voltage_v", {0.0});

    this->get_parameter("subscribe_topic_name", subscribe_topic_name_);
    this->get_parameter("publish_topic_name", publish_topic_name_);
    this->get_parameter("pressure_cutoff_frequency_hz", pressure_cutoff_frequency_hz_);

    const auto pressure_sensor_idx_param =
      this->get_parameter("pressure_sensor_idx").as_integer_array();
    const auto pressure_sensor_type_param =
      this->get_parameter("pressure_sensor_type_str").as_string_array();
    const auto loadcell_signal_plus_idx_param =
      this->get_parameter("loadcell_signal_plus_idx").as_integer_array();
    const auto loadcell_signal_minus_idx_param =
      this->get_parameter("loadcell_signal_minus_idx").as_integer_array();
    const auto loadcell_cutoff_frequency_param =
      this->get_parameter("loadcell_cutoff_frequency_hz").as_double_array();
    const auto loadcell_zero_balance_voltage_param =
      this->get_parameter("loadcell_zero_balance_voltage_v").as_double_array();

    initialize_pressure_sensors(pressure_sensor_idx_param, pressure_sensor_type_param);
    initialize_loadcells(
      loadcell_signal_plus_idx_param, loadcell_signal_minus_idx_param,
      loadcell_cutoff_frequency_param, loadcell_zero_balance_voltage_param);

    publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
      publish_topic_name_, rclcpp::QoS(10));
    subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      subscribe_topic_name_, rclcpp::QoS(10),
      std::bind(&PressureAndForceNode::topic_callback, this, std::placeholders::_1));
    zero_loadcell_service_ = this->create_service<peripheral::srv::ZeroLoadcell>(
      "/pressure_and_force/zero_loadcell",
      std::bind(
        &PressureAndForceNode::handle_zero_loadcell, this, std::placeholders::_1,
        std::placeholders::_2));

    RCLCPP_INFO(
      this->get_logger(),
      "pressure_and_force subscribes to '%s', publishes %zu pressures + %zu forces on '%s', "
      "service='%s'",
      subscribe_topic_name_.c_str(), pressure_sensors_.size(), loadcells_.size(),
      publish_topic_name_.c_str(), "/pressure_and_force/zero_loadcell");
  }

private:
  void initialize_pressure_sensors(
    const std::vector<int64_t> & pressure_sensor_idx_param,
    const std::vector<std::string> & pressure_sensor_type_param)
  {
    if (pressure_sensor_idx_param.empty()) {
      throw std::runtime_error("pressure_sensor_idx must not be empty");
    }
    if (pressure_cutoff_frequency_hz_ < 0.0) {
      throw std::runtime_error("pressure_cutoff_frequency_hz must be non-negative");
    }
    if (pressure_sensor_idx_param.size() != pressure_sensor_type_param.size()) {
      throw std::runtime_error(
              "pressure_sensor_idx and pressure_sensor_type_str must have the same length");
    }

    pressure_sensors_.reserve(pressure_sensor_idx_param.size());
    for (size_t i = 0; i < pressure_sensor_idx_param.size(); ++i) {
      if (pressure_sensor_idx_param[i] < 0) {
        throw std::runtime_error("pressure_sensor_idx must contain only non-negative values");
      }
      pressure_sensors_.emplace_back(
        static_cast<size_t>(pressure_sensor_idx_param[i]),
        parse_pressure_sensor_type(pressure_sensor_type_param[i]),
        pressure_cutoff_frequency_hz_);
    }
  }

  void initialize_loadcells(
    const std::vector<int64_t> & loadcell_signal_plus_idx_param,
    const std::vector<int64_t> & loadcell_signal_minus_idx_param,
    const std::vector<double> & loadcell_cutoff_frequency_param,
    const std::vector<double> & loadcell_zero_balance_voltage_param)
  {
    if (loadcell_signal_plus_idx_param.size() != loadcell_signal_minus_idx_param.size()) {
      throw std::runtime_error(
              "loadcell_signal_plus_idx and loadcell_signal_minus_idx must have the same length");
    }
    if (loadcell_cutoff_frequency_param.size() != loadcell_signal_plus_idx_param.size()) {
      throw std::runtime_error(
              "loadcell_cutoff_frequency_hz must have the same length as "
              "loadcell_signal_plus_idx");
    }
    if (loadcell_zero_balance_voltage_param.size() != loadcell_signal_plus_idx_param.size()) {
      throw std::runtime_error(
              "loadcell_zero_balance_voltage_v must have the same length as "
              "loadcell_signal_plus_idx");
    }

    loadcells_.reserve(loadcell_signal_plus_idx_param.size());
    for (size_t i = 0; i < loadcell_signal_plus_idx_param.size(); ++i) {
      if (loadcell_signal_plus_idx_param[i] < 0 || loadcell_signal_minus_idx_param[i] < 0) {
        throw std::runtime_error(
                "loadcell_signal_plus_idx and loadcell_signal_minus_idx must be non-negative");
      }
      if (loadcell_cutoff_frequency_param[i] < 0.0 || !std::isfinite(loadcell_cutoff_frequency_param[i])) {
        throw std::runtime_error("loadcell_cutoff_frequency_hz must be finite and non-negative");
      }
      loadcells_.emplace_back(
        static_cast<size_t>(loadcell_signal_plus_idx_param[i]),
        static_cast<size_t>(loadcell_signal_minus_idx_param[i]),
        loadcell_cutoff_frequency_param[i],
        loadcell_zero_balance_voltage_param[i]);
    }
  }

  std::optional<double> update_sampling_period()
  {
    const auto now = std::chrono::steady_clock::now();
    std::optional<double> sampling_period_s;
    if (last_callback_time_.has_value()) {
      sampling_period_s = std::chrono::duration<double>(now - *last_callback_time_).count();
    }
    last_callback_time_ = now;
    return sampling_period_s;
  }

  void handle_zero_loadcell(
    const std::shared_ptr<peripheral::srv::ZeroLoadcell::Request> request,
    std::shared_ptr<peripheral::srv::ZeroLoadcell::Response> response)
  {
    if (!last_input_msg_.has_value()) {
      response->success = false;
      response->message = "No input message has been received yet.";
      return;
    }

    if (loadcells_.empty()) {
      response->success = false;
      response->message = "No loadcell is configured.";
      return;
    }

    if (request->channel < 0 || static_cast<size_t>(request->channel) >= loadcells_.size()) {
      std::ostringstream message;
      message << "channel must be in [0, " << loadcells_.size() - 1 << "], but got "
              << request->channel;
      response->success = false;
      response->message = message.str();
      return;
    }

    const auto & input = *last_input_msg_;
    const size_t channel = static_cast<size_t>(request->channel);
    const size_t plus_index = loadcells_[channel].signal_plus_index();
    const size_t minus_index = loadcells_[channel].signal_minus_index();
    if (plus_index >= input.size() || minus_index >= input.size()) {
      std::ostringstream message;
      message << "loadcell[" << channel << "] input indices are out of range for cached input size "
              << input.size();
      response->success = false;
      response->message = message.str();
      return;
    }
    loadcells_[channel].zero_from_current_measurement(input[plus_index], input[minus_index]);

    std::ostringstream message;
    message << "Zeroed loadcell[" << channel << "]. zero_balance_voltage_v="
            << static_cast<double>(input[plus_index]) - static_cast<double>(input[minus_index]);
    response->success = true;
    response->message = message.str();
    RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
  }

  void topic_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    last_input_msg_ = msg->data;
    const std::optional<double> sampling_period_s = update_sampling_period();

    std_msgs::msg::Float32MultiArray out_msg;
    out_msg.data.reserve(pressure_sensors_.size() + loadcells_.size());

    for (size_t i = 0; i < pressure_sensors_.size(); ++i) {
      const auto & pressure_sensor = pressure_sensors_[i];
      const size_t sensor_index = pressure_sensor.sensor_index();
      if (sensor_index >= msg->data.size()) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "pressure_sensor_idx[%zu]=%zu is out of range for input size %zu",
          i, sensor_index, msg->data.size());
        out_msg.data.push_back(std::numeric_limits<float>::quiet_NaN());
        continue;
      }

      out_msg.data.push_back(
        pressure_sensors_[i].convert(msg->data[sensor_index], sampling_period_s));
    }

    for (size_t i = 0; i < loadcells_.size(); ++i) {
      const auto & loadcell = loadcells_[i];
      const size_t plus_index = loadcell.signal_plus_index();
      const size_t minus_index = loadcell.signal_minus_index();
      if (plus_index >= msg->data.size() || minus_index >= msg->data.size()) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "loadcell[%zu] input indices (%zu, %zu) are out of range for input size %zu",
          i, plus_index, minus_index, msg->data.size());
        out_msg.data.push_back(std::numeric_limits<float>::quiet_NaN());
        continue;
      }

      out_msg.data.push_back(
        loadcells_[i].convert(msg->data[plus_index], msg->data[minus_index], sampling_period_s));
    }

    publisher_->publish(out_msg);
  }

  std::string subscribe_topic_name_;
  std::string publish_topic_name_;
  double pressure_cutoff_frequency_hz_{10.0};
  std::vector<PressureSensor> pressure_sensors_;
  std::vector<Loadcell> loadcells_;
  std::optional<std::vector<float>> last_input_msg_;
  std::optional<std::chrono::steady_clock::time_point> last_callback_time_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
  rclcpp::Service<peripheral::srv::ZeroLoadcell>::SharedPtr zero_loadcell_service_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PressureAndForceNode>());
  rclcpp::shutdown();
  return 0;
}
