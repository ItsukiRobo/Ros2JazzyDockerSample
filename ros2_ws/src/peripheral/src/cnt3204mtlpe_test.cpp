#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int64_multi_array.hpp"
#include "std_msgs/msg/u_int32_multi_array.hpp"

#include <iostream>
#include <fcntl.h>
#include <memory.h>
#include <stdint.h>
#include <unistd.h>

char buf[128];
int fd = -1;
uint32_t counterData[4];

bool initialized = false;
uint32_t initial[4];

int cnt_indexes;
double mm_per_step;

int BoardOpen()
{
    std::cout << "CNT Board Open" << "\n";
    fd = open("/dev/CNT", O_RDWR);
    if(fd == -1) {
        std::cout << "fopen error: CNT-3204MT-LPE" << "\n";
        return -1;
    }
    return 0;
}

int BoardClose()
{
    std::cout << "CNT Board Close" << "\n";
    close(fd);
    return 0;
}

int BoardUpdate()
{
    char* bufptr = buf;
    
    if(read(fd, buf, sizeof(buf)) == -1)
    {
        std::cout << "Update error: CNT-3204MT-LPE" << "\n";
    }
    
    for(int i=0; i<4; i++) {
        counterData[i] = *(uint32_t*)bufptr;        
        bufptr += sizeof(uint32_t);
    }

    return 0;
}

class CNTNode : public rclcpp::Node
{
public:
  CNTNode()
  : Node("cnt3204mtlpe")
  {
    subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/pressure", 10, std::bind(&CNTNode::topic_callback, this, std::placeholders::_1));

    publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/pressure_and_length", 10);
    debug_publisher_ = this->create_publisher<std_msgs::msg::UInt32MultiArray>("/cnt3204mtlpe/counter", 10);
    debug_delta_publisher_ = this->create_publisher<std_msgs::msg::Int64MultiArray>("/cnt3204mtlpe/counter_delta", 10);

    this->declare_parameter<int>("~cnt_indexes", 4);
    this->get_parameter("~cnt_indexes", cnt_indexes);
    this->declare_parameter<double>("mm_per_step", 670.0 / 134400.0);
    this->get_parameter("mm_per_step", mm_per_step);
  }


private:
  void topic_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) const
  {
    auto msg1 = std_msgs::msg::Float32MultiArray();
    auto debug_msg = std_msgs::msg::UInt32MultiArray();
    auto debug_delta_msg = std_msgs::msg::Int64MultiArray();
    msg1.data = msg->data;

    BoardUpdate();

    for(int i = 0; i < cnt_indexes; i++)
    {
      uint32_t value = counterData[i];
      debug_msg.data.push_back(value);

      if (!initialized) {
        initial[i] = value ;
        if (i == cnt_indexes - 1) {
          initialized = true;
        }
      }

      uint32_t raw_delta = value - initial[i];
      int32_t signed_delta = static_cast<int32_t>(raw_delta);
      debug_delta_msg.data.push_back(static_cast<int64_t>(signed_delta));

      double length = static_cast<double>(signed_delta) * mm_per_step ;  // [mm]
      msg1.data.push_back(length);
    }

    publisher_->publish(msg1);

    // Debug
    // debug_publisher_->publish(debug_msg);
    // debug_delta_publisher_->publish(debug_delta_msg);
  }
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::UInt32MultiArray>::SharedPtr debug_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int64MultiArray>::SharedPtr debug_delta_publisher_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  BoardOpen();

  rclcpp::spin(std::make_shared<CNTNode>());

  BoardClose();

  rclcpp::shutdown();
  return 0;
}

//     // stsdata = inpd(address + 0x0c);   // ステータス読み出し

//     cnt3204mtlpe_pub->publish(msg);
//     rclcpp::spin_some(node);
//     loop_rate.sleep();
//   }

//   BoardClose();

//   return 0;
// }
