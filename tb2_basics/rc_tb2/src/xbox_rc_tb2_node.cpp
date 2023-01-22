/**
 * @file xbox_rc_tb2_node.cpp
 * @author Fernando Gonzalez (fergonzaramos@yahoo.es)
 * @brief Node that subscribes to Joystick data and command
 * the velocity messages to the Turtlebot 2
 * @version 0.1
 * @date 2022-10-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <memory>
#include <math.h>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "rc_tb2/RcTb2.hpp"

using sensor_msgs::msg::Joy;
using std::placeholders::_1;
using geometry_msgs::msg::Twist;

#define HZ 5

/**
 * @mainpage ROS2 package to be able to move a Turtlebot 2 robot
 * ROS2 package to be able to move a Turtlebot 2 robot using an Xbox
 * or PS4 remote controller
 * 
 */

class Tb2Controller : public rclcpp::Node
{
public:
  /**
   * @brief Constructor
   * @details It is the constructor of the class Tb2Controller.
   * It inherits from rclcpp::Node
   * 
   * @param node_name: const std::string &. Name of the node
   */
  Tb2Controller(const std::string & node_name)
  : Node(node_name)
  {
    rc_tb2_ = std::make_unique<rc_tb2::RcTb2>(rc_tb2::RcTypes::XBOX);
    joy_sub_ = this->create_subscription<Joy>("/joy",
      rclcpp::QoS(1).best_effort(),
      std::bind(&Tb2Controller::joyCallback_, this, _1)
    );

    rc_actions::Trigger_t trigger;

    trigger.button = rc_actions::Buttons_e::BUTTON_LT;
    trigger.value = (float)-1.0;

    auto action = std::make_shared<rc_actions::RunCmd_t>("echo Hello World");
    rc_tb2_->setActionCmd("process1", trigger, action);

    // rc_tb2_->execActions();
  }

  void
  update()
  {
    rc_tb2::RcType rc_data;
    Twist vel_msg;

    try {
      rc_data = rc_tb2_->getRcData();
    } catch(rc_tb2::RcTb2Exception & e) {
      e.what();
      return;
    }

    composeVelMsg(rc_data, &vel_msg);

    RCLCPP_INFO(this->get_logger(), "Linear: %f | Angular: %f\n",
      vel_msg.linear.x, vel_msg.angular.z);
  }
private:
  void
  composeVelMsg(rc_tb2::RcType rc, Twist * msg)
  {
    // Linear vel

    msg->linear.x = rc.joystick_l.x * MAX_LINEAR_VEL;

    // Angular vel

    msg->angular.z = rc.joystick_r.y * MAX_ANGULAR_VEL;
  }

  void
  joyCallback_(const Joy::SharedPtr msg)
  {
    rc_tb2_->setRcData(*msg);
  }

  std::unique_ptr<rc_tb2::RcTb2> rc_tb2_;         // Member for handling the remote
  rclcpp::Subscription<Joy>::SharedPtr joy_sub_;  // Joy Subscriber

  const float MAX_LINEAR_VEL = 0.5;
  const float MAX_ANGULAR_VEL = M_PI / 3.0;
};

int
main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Tb2Controller>("xbox_rc_tb2_node");

  rclcpp::Rate loop_rate(HZ);
  while(rclcpp::ok()) {
    rclcpp::spin_some(node->get_node_base_interface());
    node->update();
    loop_rate.sleep();
  }
}