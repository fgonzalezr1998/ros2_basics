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

  RcTb2::RcTb2(const RcTypes & rc_type)
  {
    if (rc_type < XBOX) {
      throw RcTb2Exception("Bad RC Type: " + std::to_string(rc_type));
    }

    rc_type_ = rc_type;
    actions_list_.clear();
    last_rc_data_ = nullptr;
  }

  void
  RcTb2::setRcData(const Joy & joy_msg)
  {
    RcType rc_data;
    // Fill the 'last_rc_data_' struct
    if (rc_type_ == XBOX) {
      setXboxRcData(joy_msg, &rc_data);
    }

    last_rc_data_ = std::make_unique<RcType>(rc_data);
  }

  /*
   * Private Member. It has to be here forn no specialization after instantiation
   */

  template<> void
  RcTb2::setAction(const std::string & name, Trigger_t trigger,
    std::shared_ptr<RunCmd_t> action)
  {
    rc_actions::Action_t act;

    act.name = name;
    act.trigger = trigger;
    act.action_type = rc_actions::Actions_e::RUN_CMD;
    act.action = action;

    actions_list_.push_back(act);
  }

  void
  RcTb2::setActionCmd(const std::string & name, Trigger_t trigger,
    std::shared_ptr<RunCmd_t> action)
  {
    this->setAction<std::shared_ptr<rc_actions::RunCmd_t>>(name, trigger, action);
  }

  /*
   * For debugging
   */

  void
  RcTb2::execActions()
  {
    rc_actions::RunCmd_t *cmd;
    for (auto action : actions_list_) {
      if (action.action_type == rc_actions::Actions_e::RUN_CMD) {
        cmd = (RunCmd_t *)action.action.get();
        processesHandler_.runCmd(cmd->cmd, action.name);
      }
    }
  }

  RcType
  RcTb2::getRcData()
  {
    if (last_rc_data_ == nullptr) {
      throw RcTb2Exception("Unknown data");
    }

    return *last_rc_data_.get();
  }

  /*
   * PRIVATE MEMBERS
   */

  void
  RcTb2::setXboxRcData(const Joy & joy_msg, RcType * rc_data)
  {
    rc_data->joystick_l.y = joy_msg.axes[0];
    rc_data->joystick_l.x = joy_msg.axes[1];

    rc_data->joystick_r.y = joy_msg.axes[3];
    rc_data->joystick_r.x = joy_msg.axes[4];

    rc_data->lt = joy_msg.axes[2];
    rc_data->rt = joy_msg.axes[5];

    rc_data->lb = joy_msg.buttons[4];
    rc_data->rb = joy_msg.buttons[5];

    rc_data->h_arrow = joy_msg.buttons[6];
    rc_data->v_arrow = joy_msg.buttons[7];

    rc_data->buttons.up = joy_msg.buttons[3];
    rc_data->buttons.right = joy_msg.buttons[1];
    rc_data->buttons.bottom = joy_msg.buttons[0];
    rc_data->buttons.left = joy_msg.buttons[2];
  }
}