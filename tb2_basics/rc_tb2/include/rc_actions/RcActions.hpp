#include <string>
#include <vector>

#ifndef RC_ACTIONS__RCACTIONS_HPP_
#define RC_ACTIONS__RCACTIONS_HPP_

namespace rc_actions
{

enum Actions_e {
  RUN_CMD = 1,
};

typedef struct RunCmd_t RunCmd_t;
typedef struct Trigger_t Trigger_t;
typedef struct Action_t Action_t;

struct RunCmd_t {
  std::string cmd;
};

struct Trigger_t {
  std::string button;
  bool value;
};

struct Action_t {
  Trigger_t trigger;
  Actions_e action_type;
  uint8_t *action;
};

} // namespace rc_actions

#endif  // RC_ACTIONS__RCACTIONS_HPP_