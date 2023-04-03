# harness_assembly
Repository of ROS packages for wire harness assembly task
## task_control
Package containing the "master" node, i.e., the one reading the sequence of operations from a file and sending the corresponding goal to the servers
## trajectory_control
Package containing the nodes (action servers) for executing the wires routing through the clips or the pick-and-place of the connector
## trajectory_control_msgs
This package contains the definition for custom messages/actions used in trajectory_control
## twist_control
Package with the node (again an action server) for publishing the twist command based on the tactile indicator
## twist_control_msgs
This package contains the definition for custom messages/actions used in twist_control