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
#include <sensor_msgs/msg/joy.hpp>
#include "rc_tb2/RcTb2.hpp"

using sensor_msgs::msg::Joy;
using std::placeholders::_1;

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
    rc_tb2_ = std::make_unique<rc_tb2::RcTb2>(rc_tb2::XBOX);
    joy_sub_ = this->create_subscription<Joy>("/joy",
      rclcpp::QoS(1).best_effort(),
      std::bind(&Tb2Controller::joyCallback_, this, _1)
    );
  }
private:
  void
  joyCallback_(const Joy::SharedPtr msg)
  {
    ;
  }

  std::unique_ptr<rc_tb2::RcTb2> rc_tb2_; // Member for handling the remote
  rclcpp::Subscription<Joy>::SharedPtr joy_sub_;     // Joy Subscriber
};

int
main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Tb2Controller>("xbox_rc_tb2_node");
}