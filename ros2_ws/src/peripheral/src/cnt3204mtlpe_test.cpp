#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/multi_array_layout.hpp"
#include "std_msgs/msg/multi_array_dimension.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include <iostream>
#include <string>
// #include <conio. h>

// #include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <memory.h>
#include <stdint.h>

#include <chrono>

using std::placeholders::_1;

using namespace std_msgs::msg;
using namespace std;

char buf[128];
char obuf[1];
int fd = -1;
uint32_t counterData[4];

int flag = 0 ;
uint32_t initial[4];

int cnt_indexes;

// rclcpp::Node::SharedPtr node = nullptr;


int BoardOpen()    // CounterInput()
{
    std::cout << "CNT Board Open" << "\n";
    fd = open("/dev/CNT", O_RDWR);
    if(fd == -1) {
        std::cout << "fopen error: CNT-3204MT-LPE" << "\n";
        return -1;
    }
    return 0;
}

int BoardClose()    // ~CounterInput()
{
    std::cout << "CNT Board Close" << "\n";
    close(fd);
    return 0;
}

int BoardUpdate()    // UpdateIn()   カウンタ値を更新する。（制御ループで毎回呼び出す。）
{
    char* bufptr = buf;

    // char* testptr;
    
    if(read(fd, buf, sizeof(buf)) == -1)
    {
        std::cout << "Update error: CNT-3204MT-LPE" << "\n";
        // RCLCPP_INFO(this->get_logger(), "Update error: CNT-3204MT-LPE");
        // return -1;
    }
    
    bufptr = buf;

    for(int i=0; i<4; i++) {
        counterData[i] = *(uint32_t*)bufptr;        

        // if(i == 0) testptr = bufptr;

        bufptr += sizeof(uint32_t);
    }

    // return testptr;
    return 0;
}

int GetCounterValue(int Channel)    // カウンタ値を読む
{
    if(Channel<0) return -1;
    if(Channel>=4) return -1;
    return(counterData[Channel]);
}

void CounterReset(int Channel)    // カウンタを0にリセットする。
{
    if(Channel<0) return;
    if(Channel>=4) return;
    obuf[0] = 0x01 << Channel;
    write(fd, obuf, sizeof(obuf));
}

void CounterResetAll()    // 全チャンネルのカウンタを0にリセットする。
{
    obuf[0] = 0x0F;
    write(fd, obuf, sizeof(obuf));
}



class CNTNode : public rclcpp::Node
{
public:
  CNTNode()
  : Node("cnt3204mtlpe")
  {
    subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/pressure", 10, std::bind(&CNTNode::topic_callback, this, _1));

    publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/pressure_and_length", 10);

    this->declare_parameter<int>("~cnt_indexes", 4);
    this->get_parameter("~cnt_indexes", cnt_indexes);
  }


private:
  void topic_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) const
  {
    auto msg1 = std_msgs::msg::Float32MultiArray();
    msg1.data.clear();

    msg1.data = msg->data;

    // msg->data.clear();
    // msg.data.clear();
    BoardUpdate();

    // char* tes = BoardUpdate();

    // // address check
    // RCLCPP_INFO(node->get_logger(), "bufptr : '%x'", tes);
    // RCLCPP_INFO(node->get_logger(), "&bufptr : '%f'", *(uint32_t*)tes);

    // int stsdata = _inpd(tes + 0x0c);   // ステータス呼び出し
//    char* stptr = tes + 0x0c;   // ステータス呼び出し

    // RCLCPP_INFO(node->get_logger(), "status data : '%02x'", &stptr);

    for(int i = 0; i < cnt_indexes; i++)
    // for(int i = 0; i < 4; i++)
		{
			uint32_t value = counterData[i];

      if (flag == 0){
        initial[i] = value ;
        if (i == cnt_indexes-1){
          flag = 1 ;
        }
      }

      double length = (value - initial[i] + 40000) / 134400.0 * 670.0 - 40000 / 134400.0 * 670.0 ;  // [mm]
      //double length = (value - initial[i]) / 134400.0 * 670.0 ;  // [mm]
      /*if (length > 700.0){
        length = 0.0 ;
      }*/
			msg1.data.push_back(length);
			// msg1.data.push_back(value);
		}

    // RCLCPP_INFO(this->get_logger(), "cnt_indexes = %d", cnt_indexes);

    // RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", msg1.data[0]);
    // RCLCPP_INFO(node->get_logger(), "obuf : '%x'", fd);

    // stsdata = inpd(address + 0x0c);   // ステータス読み出し

    publisher_->publish(msg1);
  }
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  BoardOpen();

  rclcpp::spin(std::make_shared<CNTNode>());

  // CounterResetAll() ;

  BoardClose();

  rclcpp::shutdown();
  return 0;
}



// int main(int argc, char **argv)
// {
//   rclcpp::init(argc, argv);
//   auto node = rclcpp::Node::make_shared("cnt3204mtlpe");

//   auto cnt3204mtlpe_pub = node->create_publisher<std_msgs::msg::Float32MultiArray>("cnt3204mtlpe/count", 10);
//   rclcpp::Rate loop_rate(10);   // Hz

//   BoardOpen();

// //   CounterResetAll();

//   std_msgs::msg::Float32MultiArray msg;
  
//   uint32_t value;

//   while (rclcpp::ok())
//   {
//     msg.data.clear();
//     BoardUpdate();

//     // char* tes = BoardUpdate();

//     // // address check
//     // RCLCPP_INFO(node->get_logger(), "bufptr : '%x'", tes);

//     // RCLCPP_INFO(node->get_logger(), "&bufptr : '%f'", *(uint32_t*)tes);

//     // int stsdata = _inpd(tes + 0x0c);   // ステータス呼び出し
// //    char* stptr = tes + 0x0c;   // ステータス呼び出し

//     // RCLCPP_INFO(node->get_logger(), "status data : '%02x'", &stptr);

//     for(int i = 0; i < 4; i++)
// 		{
// 			value = counterData[i];
// 			// double value = (analogData[i]-65535.0/2.0)/65535.0*20.0;
// 			msg.data.push_back(value);
// 		}

//     // RCLCPP_INFO(node->get_logger(), "Publishing: '%f'", msg.data[0]);

//     // RCLCPP_INFO(node->get_logger(), "obuf : '%x'", fd);

//     // stsdata = inpd(address + 0x0c);   // ステータス読み出し

//     cnt3204mtlpe_pub->publish(msg);
//     rclcpp::spin_some(node);
//     loop_rate.sleep();
//   }

//   BoardClose();

//   return 0;
// }

