/**
 * @file RcTb2.hpp
 * @author Fernando Gonzalez (fergonzaramos@yahoo.es)
 * @brief Headers for RcTb2 class and associated data
 * @details It provides an interface to gather the data provided by
 * the joystick package (remote controller) taking into account different
 * types of remote controllers (Xbox, Ps4, etc...)
 * @version 0.1
 * @date 2022-10-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <string>
#include <exception>
#include <sensor_msgs/msg/joy.hpp>
#include <stdio.h>
#include "rc_actions/RcActions.hpp"

#ifndef RC_TB2__RCTB2_HPP_
#define RC_TB2__RCTB2_HPP_

using sensor_msgs::msg::Joy;
using rc_actions::Action_t;
using rc_actions::Actions_e;
using rc_actions::Trigger_t;
using rc_actions::RunCmd_t;

namespace rc_tb2
{

/*!
 * @brief Remote Controller Type
 */
typedef enum RcTypes {
  XBOX = 1,
  PS4,
} RcTypes;

typedef struct RcType RcType;
typedef struct JoystickType JoystickType;
typedef struct ButtonsType ButtonsType;

/**
 * @brief Represents the four buttons located
 * in the right side of the remote
 */
struct ButtonsType {
  bool up;            /**< bool: true if pressed, false if released */
  bool right;         /**< bool: true if pressed, false if released */
  bool left;          /**< bool: true if pressed, false if released */
  bool bottom;        /**< bool: true if pressed, false if released */
};

/**
 * @brief Represents the position of a Joy Stick
 * based on a coordinates system (x, y)
 */
struct JoystickType {
  float x;            /**< float: Between -1.0 and 1.0 */
  float y;            /**< float: Between -1.0 and 1.0 */
};

/**
 * @brief Represents all the buttons and controls of a remote control
 * 
 */
struct RcType {
  JoystickType joystick_l;  /**< Joystick left */
  JoystickType joystick_r;  /**< Joystick right */
  float lt;                 /**< Trigger left. 1.0: fully released. -1.0: fully pressed*/
  float rt;                 /**< Trigger left. 1.0: fully released. -1.0: fully pressed*/
  bool lb;                  /**< Left Button: true if pressed, false if released */
  bool rb;                  /**< Right Button: true if pressed, false if released */
  bool h_arrow;             /**< Horizontal arrow. true if pressed, false if released */
  bool v_arrow;             /**< Horizontal arrow. true if pressed, false if released */
  ButtonsType buttons;      /**< Buttons */
};

/*!
 * @brief Exception to manage errors in RcTb2 class
 */
class RcTb2Exception : public std::exception 
{
public:
  /*!
   * @brief Constructor
   *
   * @param text: std::string. message to be printed when exception is raised
   */
  RcTb2Exception(const std::string & text);
  virtual const char* what();
private:
  std::string error_msg_;
};

/*!
 * @brief Class to manage the Turtlebot 2 using the remote control data
 */
class RcTb2
{
public:
  /*!
   * @brief Constructor
   *
   * @param rc_type: Type of remote controller (XBOX or PS4)
   * 
   * @throw RcTb2Exception
   */
  RcTb2(const RcTypes & rc_type);

  /**
   * @brief Set the last remote controller state with the received data
   * 
   * @param joy_msg Joy message
   */
  void setRcData(const Joy & joy_msg);

  template <typename T> void setAction(Trigger_t trigger, T action);

  /**
   * @brief Get the last RC Data
   * 
   * @return RcType: The last Remote Controller data
   * 
   * @throw RcTb2Exception in case there's no data yet
   */
  RcType getRcData();
private:
  RcTypes rc_type_;
  std::unique_ptr<RcType> last_rc_data_;

  void setXboxRcData(const Joy & joy_msg, RcType * rc_data);
};
}

#endif  // RC_TB2__RCTB2_HPP_
