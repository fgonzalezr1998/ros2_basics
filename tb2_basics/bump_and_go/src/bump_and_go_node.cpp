/*
 * Copyright (c) 2022, Fernando Gonzalez
 * All rights reserved.
 */

#include <rclcpp/rclcpp.hpp>
#include <kobuki_ros_interfaces/msg/bumper_event.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <string>
#include <memory>
#include <math.h>

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
  explicit BumpAndGo(const std::string & node_name)
  : Node(node_name), clock_(RCL_SYSTEM_TIME), state_(STATE_FORWARD)
  {
    turning_vel_ = 0.0;
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
    Twist cmd_vel;
    switch (state_)
    {
      case STATE_FORWARD:
        cmd_vel.linear.x = 0.08;
        break;
      case STATE_BACKWARD:
        cmd_vel.linear.x = -0.08;
        break;
      case STATE_TURNING:
        cmd_vel.angular.z = turning_vel_;
      default:
        break;
    }
  }

  void
  evaluate_transition()
  {
    // If a bumper was pressed...

    if (bumper_state_->state == BumperEvent::PRESSED) {
      state_ = STATE_BACKWARD;
      ts_ = clock_.now();
      if (bumper_state_->bumper == BumperEvent::CENTER
        || bumper_state_->bumper == BumperEvent::RIGHT)
      {
        turning_vel_ = (float)(M_PI / 8.0);
        
      } else {
        turning_vel_ = -(float)(M_PI / 8.0);
      }
    } else {
      // If no bumpers pressed...

      if (state_ == STATE_BACKWARD) {
        if ((clock_.now() - ts_).seconds() >= 2.5) {
          ts_ = clock_.now();
          state_ = STATE_TURNING;
        }
      } else if (state_ == STATE_TURNING) {
        if ((clock_.now() - ts_).seconds() >= 4.0) {
          state_ = STATE_FORWARD;
        }
      }
    }
  }

  rclcpp::Subscription<BumperEvent>::SharedPtr bumper_sub_;
  rclcpp::Publisher<Twist>::SharedPtr vel_pub_;
  BumperEvent::SharedPtr bumper_state_;
  rclcpp::Time ts_;
  rclcpp::Clock clock_;
  float turning_vel_;
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
