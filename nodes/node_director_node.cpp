#include <node_director/node_director.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sstream>
#include <std_srvs/Empty.h>
#include <libudev.h>
#include <vector>
#include <map>
#include <utility>
#include <thread>
#include <unistd.h>
#include <ros/package.h>
#include <sstream>
#include <signal.h>
#include <sys/wait.h>
#include <sstream>
#include <iostream>
#include <sys/wait.h>
#include <errno.h>

#include "node_director/SpawnNode.h"
#include "node_director/ListNodes.h"
#include "node_director/StopNode.h"
#include "node_director/RestartNode.h"

node_director::NodeDirector director;

bool handle_spawn_node_request(node_director::SpawnNodeRequest& request, node_director::SpawnNodeResponse& response) {
  std::stringstream ss;
  for (const std::string& arg : request.args) {
    ss << " " << arg;
  }
  pid_t pid = director.spawnNode(request.ros_package, request.executable_name, request.args);
  if (pid == 0) {
    ROS_ERROR("Unable to spawn %s::%s", request.ros_package.c_str(), request.executable_name.c_str());
    return false;
  } else {
    ROS_INFO("Spawned %s::%s%s", request.ros_package.c_str(), request.executable_name.c_str(), ss.str().c_str());
    response.id = pid;
    return true;
  }
}

bool handle_list_nodes_request(node_director::ListNodesRequest& request, node_director::ListNodesResponse& response) {
  std::vector<node_director::SpawnedNodeInfo> nodes;
  int status;
  std::vector<pid_t> nodesToRemove;
  for (const auto& node : director.nodes_) {
    node_director::SpawnedNodeInfo info;
    info.id = node.first;
    info.ros_package = std::string(node.second->package);
    info.executable_name = std::string(node.second->executable);
    info.args = std::vector<std::string>(node.second->args);

    int result = waitpid(node.first, &status, WNOHANG);
    if (result == 0) {
      info.state = "Ok";
    } else if (result == -1) {
      info.state = "Error checking state";
    } else {
      info.state = "Terminated [" + node_director::waitStatusString(status) + "]";
      nodesToRemove.push_back(node.first);
    }

    nodes.push_back(info);
  }

  for (const auto& pid : nodesToRemove) {
    director.nodes_.erase(pid);
  }

  response.nodes = nodes;
  return true;
}

bool handle_stop_node_request(node_director::StopNodeRequest& request, node_director::StopNodeResponse& response) {
  pid_t pid = request.id;
  if (director.nodes_.count(pid) == 0) {
    response.result_code = node_director::StopNodeResponse::RESULT_NODE_NOT_FOUND;
    return true;
  }

  int result;
  bool success = director.stopNode(director.nodes_[pid], &result);

  if (result == 0) {
    if (success) {
      response.result_code = node_director::StopNodeResponse::RESULT_SUCCESS;
    } else {
      response.result_code = node_director::StopNodeResponse::RESULT_ERROR_KILLING_NODE;
    }
  } else if (result == -1) {
    response.result_code = node_director::StopNodeResponse::RESULT_ERROR_CHECKING_NODE;
  } else {
    response.result_code = node_director::StopNodeResponse::RESULT_SUCCESS; // Node already stopped
  }

  return true;
}

bool handle_restart_node_request(node_director::RestartNodeRequest& request, node_director::RestartNodeResponse& response) {
  pid_t pid = request.id;
  if (director.nodes_.count(pid) == 0) {
    response.result_code = node_director::RestartNodeResponse::RESULT_NODE_NOT_FOUND;
    return true;
  }

  std::shared_ptr<node_director::ManagedNode> node = director.nodes_[pid];

  int result;
  bool success = director.stopNode(node, &result);

  if (result == 0) {
    if (success) {
      response.result_code = node_director::StopNodeResponse::RESULT_SUCCESS;
    } else {
      response.result_code = node_director::StopNodeResponse::RESULT_ERROR_KILLING_NODE;
    }
  } else if (result == -1) {
    response.result_code = node_director::StopNodeResponse::RESULT_ERROR_CHECKING_NODE;
  } else {
    response.result_code = node_director::StopNodeResponse::RESULT_SUCCESS; // Node already stopped
  }

  ROS_INFO("Restarting %s::%s after waiting for %d to complete", node->package.c_str(), node->executable.c_str(), pid);
  int status;
  waitpid(pid, &status, 0);
  response.exit_message = node_director::waitStatusString(status);

  response.new_id = director.spawnNode(node->package, node->executable, node->args);
  if (pid == 0) {
    ROS_ERROR("Unable to restart %s::%s", node->package.c_str(), node->executable.c_str());
    return false;
  } else {
    ROS_INFO("Restarted %s::%s", node->package.c_str(), node->executable.c_str());
    return true;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "node_director");
  ros::NodeHandle node("~");
  const std::string director_namespace = node.getNamespace();

  ros::ServiceServer listNodesService = node.advertiseService("list_nodes", handle_list_nodes_request);
  ros::ServiceServer spawnNodeService = node.advertiseService("spawn_node", handle_spawn_node_request);
  ros::ServiceServer stopNodeService = node.advertiseService("stop_node", handle_stop_node_request);
  ros::ServiceServer restartNodeService = node.advertiseService("restart_node", handle_restart_node_request);

  ROS_INFO("NodeDirector started at %s", director_namespace.c_str());

  ros::spin();

  director.shutdown();

  return EXIT_SUCCESS;
}

