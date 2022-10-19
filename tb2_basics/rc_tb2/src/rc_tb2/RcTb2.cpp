/**
 * @file RcTb2.cpp
 * @author Fernando Gonzalez (fergonzaramos@yahoo.es)
 * @brief Implementation for RcTb2 class and associated data
 * @details It provides an interface to gather the data provided by
 * the joystick package (remote controller) taking into account different
 * types of remote controllers (Xbox, Ps4, etc...) 
 * @version 0.1
 * @date 2022-10-05
 *
 * @copyright Copyright (c) 2022
 * 
 */
#include "rc_tb2/RcTb2.hpp"

namespace rc_tb2
{
  RcTb2Exception::RcTb2Exception(const std::string & text)
  {
    error_msg_ = text;
  }
  const char *
  RcTb2Exception::what()
  {
    return error_msg_.c_str();
  }

  RcTb2::RcTb2(const RCType & rc_type)
  {
    if (rc_type < XBOX) {
      throw RcTb2Exception("Bad RC Type: " + std::to_string(rc_type));
    }
  }
}