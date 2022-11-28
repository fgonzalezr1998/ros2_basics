#include <string>
#include <vector>
#include <memory>
#include <map>
#include <thread>

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
  RunCmd_t(const char * s)
  {
    cmd = s;
  }
  std::string cmd;
};

struct Trigger_t {
  std::string button;
  bool value;
};

struct Action_t {
  std::string name;
  Trigger_t trigger;
  Actions_e action_type;
  std::shared_ptr<void> action;
};

class ProcessesHandler
{
public:
  ProcessesHandler();
  void runCmd(const std::string & cmd, const std::string & name);
private:
  void execProcess(char *args[], const std::string & name);
  void waitForProcesses();
  std::map<std::string, int> processes_table_;
  std::thread *processes_th_;
};

} // namespace rc_actions

#endif  // RC_ACTIONS__RCACTIONS_HPP_