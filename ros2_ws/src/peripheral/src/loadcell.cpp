#include "rclcpp/rclcpp.hpp"

#include "signal_utility/low_pass_filter.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include <chrono>
#include <functional>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

/**
 * 0kgf, 0.0V
 * 4.072kgf, 0.519V
 * 8.072kgf, 1.021V
 * 12.072kgf, 1.524V
 * 16.072kgf, 2.028V
 * y = ax
 * a = 7.918593157 kgf/V
 */

class LoadcellNode : public rclcpp::Node
{
public:
  LoadcellNode()
  : Node("loadcell")
  {
    this->declare_parameter<std::string>("subscribe_topic_name", "ai1616llpe/voltage");
    this->declare_parameter<std::string>("publish_topic_name", "/loadcell");
    this->declare_parameter<bool>("publish_raw_difference", true);
    this->declare_parameter<std::vector<int64_t>>("signal_plus_idx", {0});
    this->declare_parameter<std::vector<int64_t>>("signal_minus_idx", {1});
    this->declare_parameter<std::vector<double>>("cutoff_frequency_hz", {0.0});
    this->declare_parameter<std::vector<double>>("rated_load_n", {1.0});
    this->declare_parameter<std::vector<double>>("rated_output_voltage_v", {1.0});
    this->declare_parameter<std::vector<double>>("zero_balance_voltage_v", {0.0});

    this->get_parameter("subscribe_topic_name", subscribe_topic_name_);
    this->get_parameter("publish_topic_name", publish_topic_name_);
    this->get_parameter("publish_raw_difference", publish_raw_difference_);
    const auto signal_plus_idx_param = this->get_parameter("signal_plus_idx").as_integer_array();
    const auto signal_minus_idx_param = this->get_parameter("signal_minus_idx").as_integer_array();
    const auto cutoff_frequency_param = this->get_parameter("cutoff_frequency_hz").as_double_array();
    const auto rated_load_param = this->get_parameter("rated_load_n").as_double_array();
    const auto rated_output_voltage_param =
      this->get_parameter("rated_output_voltage_v").as_double_array();
    const auto zero_balance_voltage_param =
      this->get_parameter("zero_balance_voltage_v").as_double_array();

    if (signal_plus_idx_param.empty()) {
      throw std::runtime_error("signal_plus_idx must not be empty");
    }
    if (signal_plus_idx_param.size() != signal_minus_idx_param.size()) {
      throw std::runtime_error("signal_plus_idx and signal_minus_idx must have the same length");
    }
    const size_t loadcell_count = signal_plus_idx_param.size();
    if (cutoff_frequency_param.size() != loadcell_count ||
      rated_load_param.size() != loadcell_count ||
      rated_output_voltage_param.size() != loadcell_count ||
      zero_balance_voltage_param.size() != loadcell_count)
    {
      throw std::runtime_error(
              "cutoff_frequency_hz, rated_load_n, rated_output_voltage_v, "
              "zero_balance_voltage_v must have the same length as signal_plus_idx");
    }

    signal_plus_indices_.reserve(loadcell_count);
    signal_minus_indices_.reserve(loadcell_count);
    rated_load_n_.reserve(loadcell_count);
    rated_output_voltage_v_.reserve(loadcell_count);
    zero_balance_voltage_v_.reserve(loadcell_count);
    lpf_.reserve(loadcell_count);

    for (size_t i = 0; i < loadcell_count; ++i) {
      if (signal_plus_idx_param[i] < 0 || signal_minus_idx_param[i] < 0) {
        throw std::runtime_error("signal_plus_idx and signal_minus_idx must be non-negative");
      }
      if (cutoff_frequency_param[i] < 0.0) {
        throw std::runtime_error("cutoff_frequency_hz must be non-negative");
      }
      if (rated_load_param[i] <= 0.0) {
        throw std::runtime_error("rated_load_n must be positive");
      }
      if (rated_output_voltage_param[i] <= 0.0) {
        throw std::runtime_error("rated_output_voltage_v must be positive");
      }

      signal_plus_indices_.push_back(static_cast<size_t>(signal_plus_idx_param[i]));
      signal_minus_indices_.push_back(static_cast<size_t>(signal_minus_idx_param[i]));
      rated_load_n_.push_back(rated_load_param[i]);
      rated_output_voltage_v_.push_back(rated_output_voltage_param[i]);
      zero_balance_voltage_v_.push_back(zero_balance_voltage_param[i]);
      lpf_.emplace_back(cutoff_frequency_param[i], 500.0);
    }

    publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
      publish_topic_name_, rclcpp::QoS(10));
    subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      subscribe_topic_name_, rclcpp::QoS(10),
      std::bind(&LoadcellNode::topic_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      this->get_logger(),
      "%zu loadcells processed, publishing %s on %s",
      signal_plus_indices_.size(),
      publish_raw_difference_ ? "raw differential voltage" : "converted load",
      publish_topic_name_.c_str());
  }

private:
  double convert_voltage_to_load(double differential_voltage, size_t loadcell_index) const
  {
    return
      (differential_voltage - zero_balance_voltage_v_[loadcell_index]) * 7.918593157;
  }

  void topic_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    const auto now = std::chrono::steady_clock::now();
    double dt_seconds = 0.0;
    if (has_previous_sample_time_) {
      dt_seconds = std::chrono::duration<double>(now - previous_sample_time_).count();
    }
    previous_sample_time_ = now;
    has_previous_sample_time_ = true;

    std_msgs::msg::Float32MultiArray out_msg;
    out_msg.data.reserve(signal_plus_indices_.size());

    for (size_t i = 0; i < signal_plus_indices_.size(); ++i) {
      const size_t plus_index = signal_plus_indices_[i];
      const size_t minus_index = signal_minus_indices_[i];
      if (plus_index >= msg->data.size() || minus_index >= msg->data.size()) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "loadcell[%zu] input indices (%zu, %zu) are out of range for input size %zu",
          i, plus_index, minus_index, msg->data.size());
        out_msg.data.push_back(std::numeric_limits<float>::quiet_NaN());
        continue;
      }

      const double differential_voltage =  msg->data[minus_index] - msg->data[plus_index];
      if (publish_raw_difference_) {
        out_msg.data.push_back(static_cast<float>(differential_voltage));
        continue;
      }

      const double raw_load_n = convert_voltage_to_load(differential_voltage, i);
      lpf_[i].set_sampling_period(dt_seconds);
      const double filtered_load_n = lpf_[i].update(raw_load_n);
      out_msg.data.push_back(static_cast<float>(filtered_load_n));
    }

    publisher_->publish(out_msg);
  }

  std::string subscribe_topic_name_;
  std::string publish_topic_name_;
  bool publish_raw_difference_ = false;
  std::vector<size_t> signal_plus_indices_;
  std::vector<size_t> signal_minus_indices_;
  std::vector<double> rated_load_n_;
  std::vector<double> rated_output_voltage_v_;
  std::vector<double> zero_balance_voltage_v_;
  bool has_previous_sample_time_ = false;
  std::vector<signal_utility::BilinearLowPassFilter> lpf_;
  std::chrono::steady_clock::time_point previous_sample_time_{};
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LoadcellNode>());
  rclcpp::shutdown();
  return 0;
}
