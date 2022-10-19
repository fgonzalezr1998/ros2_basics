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

#ifndef RC_TB2__RCTB2_HPP_
#define RC_TB2__RCTB2_HPP_

namespace rc_tb2
{

/*!
 * @brief Remote Controller Type
 */
typedef enum RCType {
  XBOX = 1,
  PS4,
} RCType;

typedef struct RcType RcType;
typedef struct JoystickType JoystickType;
typedef struct ButtonsType ButtonsType;
struct ButtonsType {
  bool up;
  bool right;
  bool left;
  bool bottom;
};
struct JoystickType {
  float x;
  float y;
};
struct RcType {
  JoystickType joystick_l;
  JoystickType joystick_r;
  bool lt;
  bool rt;
  bool h_arrow;
  bool v_arrow;
  ButtonsType buttons;
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
   * @param rc_type: const RCType &. Type of remote controller (XBOX or PS4)
   * 
   * @throw cTb2Exception::RcTb2Exception
   */
  RcTb2(const RCType & rc_type);
private:
  RCType rc_type_;
};
}

#endif  // RC_TB2__RCTB2_HPP_
