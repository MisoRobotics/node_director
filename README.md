# node_director
A ROS package to spawn nodes at runtime and manage their lifecycles.

## Introduction
ROS does not provide a good way to spawn nodes at runtime because general best practice is for nodes to be loaded at start and persist throughout the mission.  However, it is sometimes necessary to spawn nodes programmatically.  This project was prompted by the desire to spawn usb_cam nodes to automatically serve images from all applicable cameras connected to a given computer regardless of which /dev/videoN they were connected to.

## Usage
Each node_director instance controls the lifecycle of all nodes it spawns.  When the controlling node_director dies, it shuts down all of its spawned child nodes as well.

## Services
```list_nodes```: Lists all spawned child nodes.  If a node has died since the last time list_nodes was called, that node will be listed as terminated for this call, and then removed from the node list so that it does not appear in future list_nodes calls.

```spawn_node```: Spawns a specified node by rosrun'ing the specified executable from the specified package with the specified arguments, and then keeps track of the resulting forked child process via a process ID which is returned from the call.

```stop_node```: Kills the specified node by sending SIGINT, but does not wait for the node to terminate (verification of node termination can be achieved by using list_nodes after a stop_node call)

```restart_node```: Kills the specified node by sending SIGINT and then waits for the process to complete, at which point it spawns a new node with the same inputs used to start the node that was just killed.  Equivalent to stop_node, then wait for the node to complete, then spawn_node.
