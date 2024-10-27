#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/int32.hpp>

#include <stdlib.h>
#include <string>
#include <memory>
#include <chrono>

#define HZ  1.0

using namespace std::chrono_literals;
using std::placeholders::_1;

class SimpleSub : public rclcpp::Node
{
public:
  SimpleSub(const std::string & node_name)
  : Node(node_name)
  {
    simpleSub_ = this->create_subscription<std_msgs::msg::Int32>(
      "sample_topic",
      rclcpp::QoS(1),
      std::bind(&SimpleSub::msgCallback, this, _1));
  }
  void step()
  {
    RCLCPP_INFO(this->get_logger(), "Message Received --> %d\n", recvMsg_.data);
  }
private:
  void msgCallback(const std_msgs::msg::Int32 & msg)
  {
    recvMsg_.data = msg.data;
  }

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr simpleSub_;
  std_msgs::msg::Int32 recvMsg_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  std::shared_ptr<SimpleSub> simpleSubNode = std::make_shared<SimpleSub>("simple_sub_iterative_node_cpp");

  rclcpp::Rate rate(HZ);

  while (rclcpp::ok()) {
    rclcpp::spin_some(simpleSubNode);
    simpleSubNode->step();
    rate.sleep();
  }

  rclcpp::shutdown();
  exit(EXIT_SUCCESS);
}