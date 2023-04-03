#! /usr/bin/env python

import rospy
import actionlib
from pilz_robot_programming.robot import Robot
from pilz_robot_programming.commands import Lin
from geometry_msgs.msg import Pose, Point, Quaternion
from math import pi
import copy
import yaml
import io
import rospkg

from trajectory_control_msgs.msg import ConnectorPickPlaceAction, ConnectorPickPlaceGoal
from trajectory_control_msgs.msg import RoutingAction, RoutingGoal
from trajectory_control_msgs.msg import WiresGraspAction, WiresGraspGoal
from trajectory_control_msgs.msg import PushDownAction, PushDownGoal
from trajectory_control_msgs.msg import SeparateWiresAction, SeparateWiresGoal
from twist_control_msgs.msg import IndicatorAction, IndicatorGoal

class TaskMaster(object):

    def __init__(self):
        # self._robot = Robot("1")
        # self._planning_group = "manipulator"
        # self._target_link = "tool0"
        # self._base = "base"
        self._connector_place_client = actionlib.SimpleActionClient("connector_pick_place", ConnectorPickPlaceAction)
        self._tend_client = actionlib.SimpleActionClient("indicator", IndicatorAction)
        self._routing_client = actionlib.SimpleActionClient("routing", RoutingAction)
        self._wires_grasp_client = actionlib.SimpleActionClient("grasp_wires", WiresGraspAction)
        self._push_down_client = actionlib.SimpleActionClient("push_down", PushDownAction)
        self._separate_wires_client = actionlib.SimpleActionClient("separate_wires", SeparateWiresAction)
        # components_spec_filepath = rospy.get_param("components_spec_filepath")
        rospack = rospkg.RosPack()
        pkg_path = rospack.get_path("scene_creation")
        components_spec_filepath = pkg_path+"/cfg/components.yaml"
        with io.open(components_spec_filepath) as stream:
            self._components_spec = yaml.safe_load(stream)
        
        self._connector_place_client.wait_for_server()
        self._routing_client.wait_for_server()
        self._wires_grasp_client.wait_for_server()
        self._push_down_client.wait_for_server()
        self._separate_wires_client.wait_for_server()

        self._vel_scale = 0.075
        self._offset = 0.015
        self._direction = 1
      
    def execute_operation(self, operation):
        op_code = operation["type"]
        if operation.has_key("pick_from"):
            self._direction = -1 if ("MO6" in operation["pick_from"]) else 1
        if op_code == "PC":
            goal = ConnectorPickPlaceGoal
            goal.holder_length = self._components_spec["components"]["holder_"+operation["connector"]]["length"]
            goal.connector_width = self._components_spec["components"][operation["connector"]]["width"]*0.001 # from mm to m
            goal.offset = self._offset
            goal.pick_frame = operation["connector"]
            goal.place_frame = "holder_" + operation["connector"] + "_tf"
            goal.velocity_scale = self._vel_scale
            self._connector_place_client.send_goal_and_wait(goal)
            result = self._connector_place_client.get_result()
            if not result.success:
                raise Exception("Connector " + operation["connector"] + " pick error.")
            #tend wires
            goal = IndicatorGoal
            goal.tc = 1.0
            goal.t_stop = 0.1
            goal.vel_max = operation["vel_max"]
            goal.indicator_threshold = operation["indicator_threshold"]
            goal.distance = self._components_spec["components"]["holder_"+operation["connector"]]["length"] - \
                            self._components_spec["components"][operation["connector"]]["width"]*0.001
            self._tend_client.send_goal_and_wait(goal)
            result = self._tend_client.get_result()
            if not result.success:
                raise Exception("Connector " + operation["connector"] + " tend error.")
        elif ( op_code == "R"):
            current_frame = copy.deepcopy(operation["pick_from"])
            if not "clip" in current_frame:
                current_frame = "holder_" + current_frame + "_tf"
            goal = RoutingGoal
            goal.z_offset = 0.03
            goal.direction = self._direction
            goal.up_hole = operation["up_hole"]
            for clip,offset,rotation,lateral in zip(operation["clips"],operation["offset"],operation["rotation"],operation["lateral"]):
                goal.current_frame = current_frame
                goal.next_frame = clip
                goal.x_offset = offset
                goal.z_rotation = rotation
                goal.lateral = lateral
                goal.velocity_scale = self._vel_scale
                self._routing_client.send_goal_and_wait(goal)
                result = self._routing_client.get_result()
                if not result.success:
                    raise Exception("Routing from " + current_frame + " to " + clip + " does not succeded.")
                current_frame = clip
        elif ( op_code == "G" or op_code == "S" ): # in case of grasp or separation operation
            # first we need to GRASP the wires
            goal = WiresGraspGoal
            goal.connector_name = operation["pick_from"]
            goal.num_wires = operation["wires"]
            goal.offset = self._offset
            goal.velocity_scale = self._vel_scale
            self._wires_grasp_client.send_goal_and_wait(goal)
            result = self._wires_grasp_client.get_result()
            if not result.success:
                raise Exception("Wires grasp from " + operation["pick_from"] + " does not succeded.")
            if (op_code == "S" ): # in case of separation operation
                goal = SeparateWiresGoal
                goal.direction = self._direction
                for clip in operation["clips"]:
                    goal.next_frame = clip
                    goal.velocity_scale = self._vel_scale
                    self._separate_wires_client.send_goal_and_wait(goal)
                    result = self._separate_wires_client.get_result()
                    if not result.success:
                        raise Exception("Separation in " + clip + " does not succeded.")
        elif (op_code == "PD"):
            goal = PushDownGoal
            goal.direction = self._direction
            for clip,side in zip(operation["clips"],operation["side"]):
                goal.frame = clip
                goal.z_offset = 0.0
                goal.velocity_scale = self._vel_scale
                goal.clip_side = side
                self._push_down_client.send_goal_and_wait(goal)
                result = self._push_down_client.get_result()
                if not result.success:
                    raise Exception("Push down operation for " + clip + " does not succeded.")
        
if __name__ == '__main__':
    rospy.init_node('assembly_task_master')
    master = TaskMaster()
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path("task_control")
    assembly_seq_filepath = pkg_path + "/cfg/assembly_seq.yaml"
    with io.open(assembly_seq_filepath) as stream:
        yaml_node = yaml.safe_load(stream)
    for operation in yaml_node["sequence"]:
        try:
            master.execute_operation(operation)
        except Exception as e:
            print(e)
            break
    rospy.signal_shutdown("")
