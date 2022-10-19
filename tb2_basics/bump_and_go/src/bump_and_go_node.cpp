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
  explicit BumpAndGo(const std::string & node_name)
  : Node(node_name), clock_(RCL_SYSTEM_TIME), state_(STATE_FORWARD)
  {
    load_params();
    bumper_sub_ = this->create_subscription<kobuki_ros_interfaces::msg::BumperEvent>(
      "/events/bumper", rclcpp::QoS(1).best_effort(),
      std::bind(&BumpAndGo::bumper_cb, this, _1));
    vel_pub_ = this->create_publisher<Twist>(
      "/commands/velocity", rclcpp::QoS(1).reliable());
  }

  void
  step()
  {
    run_state_action();
    if (bumper_state_ != nullptr)
      evaluate_transition();
  }

private:
  void
  bumper_cb(const kobuki_ros_interfaces::msg::BumperEvent::SharedPtr msg)
  {
    bumper_state_ = msg;
  }

  void
  load_params()
  {
    fw_vel_ = (float) this->declare_parameter("forward_vel", 0.08);
    bw_vel_ = (float) this->declare_parameter("backward_vel", -0.08);
    turning_vel_ = (float) this->declare_parameter("turning_vel", 0.24);
    bw_time_ = (float) this->declare_parameter("backward_time", 2.5);
    turning_time_ = (float) this->declare_parameter("turning_time", 4.0);
  }

  void
  run_state_action()
  {
    Twist cmd_vel;

    switch (state_)
    {
      case STATE_FORWARD:
        cmd_vel.linear.x = fw_vel_;
        break;
      case STATE_BACKWARD:
        cmd_vel.linear.x = bw_vel_;
        break;
      case STATE_TURNING:
        cmd_vel.angular.z = current_angular_vel_;
      default:
        break;
    }
    vel_pub_->publish(cmd_vel);
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
        current_angular_vel_ = turning_vel_;
      } else {
        current_angular_vel_ = -turning_vel_;
      }
    } else {
      // If no bumpers pressed...

      if (state_ == STATE_BACKWARD) {
        if ((clock_.now() - ts_).seconds() >= bw_time_) {
          ts_ = clock_.now();
          state_ = STATE_TURNING;
        }
      } else if (state_ == STATE_TURNING) {
        if ((clock_.now() - ts_).seconds() >= turning_time_) {
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
  float fw_vel_, bw_vel_, turning_vel_, current_angular_vel_, bw_time_, turning_time_;
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
