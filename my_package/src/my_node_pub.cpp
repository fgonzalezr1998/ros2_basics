#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  //Init node
  auto node = rclcpp::Node::make_shared("my_node_pub");

  //Configure Hz:
  rclcpp::Rate loop_rate(1); //1Hz

  //Init publisher
  auto publisher = node->create_publisher<std_msgs::msg::String>("/talker", 1);

  std_msgs::msg::String msg;
  msg.data = "Hello World!";

  while(rclcpp::ok())
  {
      RCLCPP_INFO(node->get_logger(), "Publishing [%s]", msg.data.c_str());

      publisher->publish(msg);
      rclcpp::spin_some(node);
      loop_rate.sleep();
  }

  rclcpp::shutdown();

  exit(EXIT_SUCCESS);
}
