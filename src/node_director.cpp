/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Benjamin Pelletier
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Robert Bosch nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/
#include <node_director/node_director.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <array>
#include <vector>
#include <string>
#include <signal.h>
#include <sys/wait.h>

#include <ros/node_handle.h>

namespace node_director {

std::string waitStatusString(int status) {
  std::stringstream ss;
  bool hasContent = false;
  if (WIFEXITED(status)) {
    int exitStatus = WEXITSTATUS(status);
    ss << "Exited(";
    if (exitStatus == EXIT_SUCCESS) ss << "SUCCESS";
    else if (exitStatus == EXIT_FAILURE) ss << "FAILURE";
    else ss << status;
    ss << ")";
    hasContent = true;
  }
  if (WIFSIGNALED(status)) {
    if (hasContent) ss << " ";
    int sig = WTERMSIG(status);
    ss << "Signaled(";
    if (sig == SIGINT) ss << "INT";
    else if (sig == SIGTERM) ss << "TERM";
    else if (sig == SIGKILL) ss << "KILL";
    else if (sig == SIGQUIT) ss << "QUIT";
    else ss << sig;
    if (WCOREDUMP(status)) {
      if (hasContent) ss << " ";
      ss << "CoreDump";
    }
    ss << ")";
    hasContent = true;
  }
  if (WIFSTOPPED(status)) {
    if (hasContent) ss << " ";
    int sig = WSTOPSIG(status);
    ss << "StoppedBySignal(";
    if (sig == SIGINT) ss << "INT";
    else if (sig == SIGTERM) ss << "TERM";
    else if (sig == SIGKILL) ss << "KILL";
    else if (sig == SIGQUIT) ss << "QUIT";
    else ss << sig;
    ss << ")";
    hasContent = true;
  }
  if (WIFCONTINUED(status)) {
    if (hasContent) ss << " ";
    ss << "ResumedSigCont";
  }
  return ss.str();
}

NodeDirector::NodeDirector() {

}

pid_t NodeDirector::spawnNode(const std::string& package, const std::string& executable, const std::vector<std::string>& args) {
  int nArgs = args.size() + 3;
  const char* charargs[nArgs + 1];
  int i = 0;
  charargs[i++] = "rosrun";
  charargs[i++] = package.c_str();
  charargs[i++] = executable.c_str();
  for (auto const &arg : args) {
    charargs[i++] = arg.c_str();
  }
  charargs[i] = 0;

  std::stringstream cmd;
  for (i = 0; i < nArgs; i++) {
    if (i > 0) cmd << " ";
    cmd << charargs[i];
  }

  pid_t pid = fork();

  if (pid == 0)
  {
    // child process
    //ROS_INFO("This is the child process %s::%s", package.c_str(), executable.c_str());

    //ROS_INFO("Execing %s", cmd.str().c_str());
    int result = execvp("rosrun", (char **)charargs);
    ROS_ERROR("Continued past execvp point when executing %s::%s! result = %d, errno = %s",
              package.c_str(), executable.c_str(), result, strerror(errno));
    exit(EXIT_FAILURE);
  }
  else if (pid > 0)
  {
    // parent process
    nodes_.emplace(pid, std::make_shared<ManagedNode>(pid, package, executable, args, cmd.str()));
    return pid;
  }
  else
  {
    // fork failed
    ROS_ERROR("Unable to fork process to start %s::%s", package.c_str(), executable.c_str());
    return 0;
  }
}

void NodeDirector::shutdown() {
  if (ros::ok()) {
    ROS_INFO("Shut down requested; terminating %d child nodes\n", (int)nodes_.size());
  } else {
    printf("[NodeDirector] Shut down requested; terminating %d child nodes\n", (int)nodes_.size());
  }

  // Create a local list of all nodes
  std::vector< std::shared_ptr<ManagedNode> > nodes;
  for (auto const& kvp : nodes_) {
    nodes.push_back(kvp.second);
  }

  // Issue stop requests to all nodes
  std::vector< std::shared_ptr<ManagedNode> > stopRequests;
  int result;
  for (const std::shared_ptr<ManagedNode> node : nodes) {
    stopNode(node, &result);
    if (result == 0) {
      stopRequests.push_back(node);
    }
  }

  // Wait for all processes that were successfully sent kill requests
  if (ros::ok()) {
    ROS_INFO("Waiting for %d child nodes to complete\n", (int)stopRequests.size());
  } else {
    printf("[NodeDirector] Shutdown waiting for %d child nodes to complete\n", (int)stopRequests.size());
  }
  int status;
  for (const std::shared_ptr<ManagedNode> node : stopRequests) {
    pid_t pid = node->pid;
    std::stringstream ss;
    ss << node->package << "::" << node->executable << " (" << pid << ")";
    std::string node_name = ss.str();
    printf("[NodeDirector] Waiting for %s to complete...\n", node_name.c_str());
    int result = waitpid(pid, &status, 0);
    if (result == -1) {
      printf("[NodeDirector] Error waiting for child node %d to complete, result %d\n", pid, result);
    } else {
      printf("[NodeDirector] Child node %d completed (%d), status %s\n", pid, status, node_director::waitStatusString(status).c_str());
    }
  }

  nodes_.clear();

  printf("[NodeDirector] Shutdown complete\n");
}

bool NodeDirector::stopNode(std::shared_ptr<ManagedNode> node, int* result) {
  std::stringstream ss;
  ss << node->package << "::" << node->executable << " (" << node->pid << ")";
  std::string node_name = ss.str();

  int status;
  *result = waitpid(node->pid, &status, WNOHANG);
  if (*result == 0) {
    if (ros::ok()) {
      ROS_INFO("Killing child node %s", node_name.c_str());
    } else {
      printf("[NodeDirector] Killing child node %s\n", node_name.c_str());
    }
    if (kill(node->pid, SIGINT) == 0) {
      nodes_.erase(node->pid);
      return true;
    } else {
      if (ros::ok()) {
        ROS_ERROR("Error killing child node %s: %s\n", node_name.c_str(), strerror(errno));
      } else {
        fprintf(stderr, "[NodeDirector] Error killing child node %s: %s\n", node_name.c_str(), strerror(errno));
      }
      return false;
    }
  } else if (*result == -1) {
    if (ros::ok()) {
      ROS_ERROR("Error checking child node %s: %s\n", node_name.c_str(), strerror(errno));
    } else {
      fprintf(stderr, "[NodeDirector] Error checking child node %s: %s\n", node_name.c_str(), strerror(errno));
    }
    return false;
  } else {
    if (ros::ok()) {
      //Do nothing
    } else {
      printf("[NodeDirector] Child node %s already terminated\n", node_name.c_str());
    }
    return false;
  }
}

NodeDirector::~NodeDirector() {
  if (nodes_.size() > 0)
    shutdown();
}

}
