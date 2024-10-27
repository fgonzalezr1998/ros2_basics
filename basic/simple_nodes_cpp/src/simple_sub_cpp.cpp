#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/int32.hpp>

#include <stdlib.h>
#include <string>
#include <memory>
#include <chrono>

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
      rclcpp::QoS(4),
      std::bind(&SimpleSub::msgCallback, this, _1));
  }
private:
  void msgCallback(const std_msgs::msg::Int32 & msg)
  {
    RCLCPP_INFO(this->get_logger(), "Message Received --> %d\n", msg.data);
  }

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr simpleSub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  std::shared_ptr<SimpleSub> simpleSubNode = std::make_shared<SimpleSub>("simple_sub_node_cpp");

  rclcpp::spin(simpleSubNode);

  rclcpp::shutdown();
  exit(EXIT_SUCCESS);
}