/*
 * Copyright (c) 2022, Fernando Gonzalez
 * All rights reserved.
 */

#include <rclcpp/rclcpp.hpp>
#include <kobuki_ros_interfaces/msg/bumper_event.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <string>
#include <memory>

using std::placeholders::_1;
using kobuki_ros_interfaces::msg::BumperEvent;
using geometry_msgs::msg::Twist;

enum StateType {
  STATE_FORWARD = 1,
  STATE_BACKWARD,
  STATE_TURNING,
};

class BumpAndGo : public rclcpp::Node
{
public:
  BumpAndGo(const std::string & node_name)
  : Node(node_name), state_(STATE_FORWARD)
  {
    bumper_sub_ = this->create_subscription<kobuki_ros_interfaces::msg::BumperEvent>(
      "/events/bumper_event", rclcpp::QoS(1).best_effort(),
      std::bind(&BumpAndGo::bumperCb, this, _1));
    vel_pub_ = this->create_publisher<Twist>(
      "/commands/velocity", rclcpp::QoS(1).best_effort());
  }

  void
  step()
  {
    if (bumper_state_ == nullptr)
      return;
    RCLCPP_INFO(
      this->get_logger(), "Bumper: %d, State: %d", bumper_state_->bumper, bumper_state_->state);

    run_state_action();
    evaluate_transition();
  }

private:
  void
  bumperCb(const kobuki_ros_interfaces::msg::BumperEvent::SharedPtr msg)
  {
    bumper_state_ = msg;
  }

  void
  run_state_action()
  {
    if (state_ == STATE_FORWARD) {

    }
  }

  void
  evaluate_transition()
  {
    ;
  }

  rclcpp::Subscription<BumperEvent>::SharedPtr bumper_sub_;
  rclcpp::Publisher<Twist>::SharedPtr vel_pub_;
  BumperEvent::SharedPtr bumper_state_;
  StateType state_;
};

int
main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<BumpAndGo>("bump_and_go_node");

  rclcpp::Rate loop_rate(5.0);
  while(rclcpp::ok()) {
    rclcpp::spin_some(node->get_node_base_interface());
    node->step();
    loop_rate.sleep();
  }

  exit(EXIT_SUCCESS);
}
