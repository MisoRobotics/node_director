/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Robert Bosch LLC.
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
#ifndef NODE_DIRECTOR_NODE_DIRECTOR_H
#define NODE_DIRECTOR_NODE_DIRECTOR_H

#include <node_director/node_director.h>

#include <string>
#include <sstream>
#include <map>

#include <sensor_msgs/Image.h>

namespace node_director {

std::string waitStatusString(int status);

struct ManagedNode {
  pid_t pid;
  const std::string package;
  const std::string executable;
  const std::vector<std::string> args;
  const std::string command;

  ManagedNode(pid_t pid, const std::string& package, const std::string& executable, const std::vector<std::string>& args, const std::string& command)
    : pid(pid), package(package), executable(executable), args(args), command(command)
  { }
};

// Encapsulates the management of multiple processes
class NodeDirector {
 public:
  NodeDirector();
  ~NodeDirector();

  pid_t spawnNode(const std::string& package, const std::string& executable, const std::vector<std::string>& args);

  bool stopNode(std::shared_ptr<ManagedNode> node, int* result);

  void shutdown();

 //private:
  std::map< pid_t, std::shared_ptr<ManagedNode> > nodes_;

};

}

#endif

