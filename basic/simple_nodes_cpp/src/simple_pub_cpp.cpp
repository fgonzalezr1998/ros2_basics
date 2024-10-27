#include <rclcpp/rclcpp.hpp>
#include <stdlib.h>
#include <string>
#include <memory>

#include <std_msgs/msg/int32.hpp>

#define LOOP_RATE   2.0   // Hz

class SimplePub : public rclcpp::Node
{
public:
  SimplePub(const std::string & node_name)
  : Node(node_name), i_(0)
  {
    intPublisher_ = this->create_publisher<std_msgs::msg::Int32>(
      "sample_topic",
      rclcpp::QoS(4));
  }

  void step()
  {
    msg2Pub_.data = i_++;

    intPublisher_->publish(msg2Pub_);
  }
private:
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr intPublisher_;
  std_msgs::msg::Int32 msg2Pub_;
  int i_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto simple_pub_node = std::make_shared<SimplePub>("simple_pub_node_cpp");

  rclcpp::Rate loop_rate(LOOP_RATE);

  while (rclcpp::ok()) {
    simple_pub_node->step();
    loop_rate.sleep();
  }

  rclcpp::shutdown();
  exit(EXIT_SUCCESS);
}