#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/multi_array_layout.hpp"
#include "std_msgs/msg/multi_array_dimension.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

// #include <stdio.h>
#include <fcntl.h>
#include <unistd.h>

#include <chrono>
#include <cerrno>
#include <cstring>
#include <iostream>
#include <memory>

int fd = -1;
char buf[32];
unsigned short int analogData[16];

std::int8_t update_rate;

int BoardOpen()
{
  fd = open("/dev/AI", O_RDWR);
  if(fd == -1)
  {
    // RCLCPP_INFO(node->get_logger(), "fopen error: AI-1616L-LPE");
    return -1;
  }
  return 0;
}

int BoardUpdate()
{
  char* bufptr = buf;

  if(fd == -1)
  {
    return -1;
  }

  const ssize_t bytes_read = read(fd, buf, sizeof(buf));
  if(bytes_read != static_cast<ssize_t>(sizeof(buf)))
  {
    if(bytes_read == -1)
    {
      std::cerr << "Update error: AI-1616L-LPE: " << std::strerror(errno) << "\n";
    }
    else
    {
      std::cerr << "Update error: AI-1616L-LPE: short read (" << bytes_read << " bytes)\n";
    }
    return -1;
  }

  bufptr = buf;

  for(int i = 0; i < 16; i++)
  {
    analogData[i] = *(unsigned short int*)bufptr;
    bufptr += sizeof(unsigned short int);	
  }
  return 0;
}

int BoardClose()
{
  close(fd);
  return 0;
}


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("ai1616llpe");

  // node->declare_parameter<int>("update_rate", 10);    // default : 10 Hz
  node->declare_parameter<std::uint16_t>("update_rate", 10);    // default : 10 Hz
  node->get_parameter("update_rate", update_rate);

  auto ai1616llpe_pub = node->create_publisher<std_msgs::msg::Float32MultiArray>("ai1616llpe/voltage", 10);
  rclcpp::Rate loop_rate(update_rate);   // Hz
  // rclcpp::Rate loop_rate(10);   // Hz

  if(BoardOpen() != 0)
  {
    rclcpp::shutdown();
    return -1;
  }

  std_msgs::msg::Float32MultiArray msg;


  while (rclcpp::ok())
  {
    msg.data.clear();
    if(BoardUpdate() != 0)
    {
      rclcpp::spin_some(node);
      loop_rate.sleep();
      continue;
    }

    for(int i = 0; i < 8; i++)
    // for(int i = 0; i < 16; i++)
		{
			double value = (analogData[i]-65535.0/2.0)/65535.0*20.0;
			msg.data.push_back(value);
		}

    // RCLCPP_INFO(node->get_logger(), "Publishing: '%f , %f'", msg.data[0], msg.data[1]);

    ai1616llpe_pub->publish(msg);
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  BoardClose();

  return 0;
}

