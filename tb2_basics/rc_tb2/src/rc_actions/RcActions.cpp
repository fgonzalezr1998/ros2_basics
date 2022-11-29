#include <sys/types.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <fcntl.h>
#include <string.h>
#include <vector>
#include "rc_actions/RcActions.hpp"

#define MAXSTR 128

namespace rc_actions
{
  ProcessesHandler::ProcessesHandler()
  {
    processes_th_ = new std::thread(&ProcessesHandler::waitForProcesses, this);
  }

  void
  ProcessesHandler::runCmd(const std::string & cmd, const std::string & name)
  {
    // Split the command by ' ' separator

    std::vector<std::string> args;
    std::string::size_type prev_pos, pos;

    prev_pos = pos = 0;
    while ((pos = cmd.find(' ', pos)) != std::string::npos) {
      std::string substring(cmd.substr(prev_pos, pos - prev_pos));
      args.push_back(substring.c_str());
      prev_pos = ++pos;
    }
    args.push_back(cmd.substr(prev_pos, pos - prev_pos).c_str());

    // Compose the char* array with the arguments

    char** args_arr = (char**)malloc((args.size() * MAXSTR) + 1);
    uint i = 0;
    for (auto argument : args) {
      char* arg = (char*)malloc(MAXSTR);
      strncpy(arg, argument.c_str(), MAXSTR);
      args_arr[i] = arg;
      i++;
    }
    args_arr[i] = NULL;

    // Execute the process

    execProcess(args_arr, name);

    // Free memory

    for(i = 0; i < args.size(); i++) {
      free(args_arr[i]);
    }
    free(args_arr);
  }

  void
  ProcessesHandler::execProcess(char *args[], const std::string & name)
  {
    int pid = fork();

    switch (pid) {
    case 0:
      /* I'm the child */

      if (execvp(args[0], args) == -1) {
        fprintf(stdout, "Error in the child\n");
        exit(-1);
      }
      int status;
      while (wait(&status) > 0);
      break;

    case -1:
      /* Failure */
      break;
    
    default:
      /* I'm the parent */

      (void)setpgid(pid, 0);
      printf("Interting %s with pid: %d\n", name.c_str(), pid);
      processes_table_.insert({name, pid});
      break;
    }
  }

  void
  ProcessesHandler::waitForProcesses()
  {
    int status, pid;
    while (true) {
      pid = wait(&status);
      if (pid == -1 || pid == 0)
        continue;

      // When a process finish, delete it from map table

      for (auto it = processes_table_.begin(); it != processes_table_.end(); it++) {
        if (it->second == pid) {
          printf("Process [%s] with pid %d Died!\n", it->first.c_str(), it->second);
          processes_table_.erase(it);
          break;
        }
      }
    }
  }
}