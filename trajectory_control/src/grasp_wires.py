#!/usr/bin/env python

import rospy
from pilz_robot_programming.robot import Robot
from pilz_robot_programming.commands import Sequence, Lin, Circ, from_euler
import actionlib
from trajectory_control_msgs.msg import WiresGraspAction, WiresGraspGoal, WiresGraspResult, WiresGraspFeedback
from geometry_msgs.msg import Pose, Point, TransformStamped, Quaternion
from hand_e_pkg.msg import gripper_command, finger_position
import yaml
import rospkg
from threading import Thread
class GraspWiresActionServer(object):
    def __init__(self,name,connectors):
        self._planning_group = "manipulator"
        self._target_link = "fingernails"
        self._base = "base"
        self._action_name = name
        self._connectors = connectors
        self._as = actionlib.SimpleActionServer(self._action_name, WiresGraspAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        self._gripper_pub = rospy.Publisher("gripper_command",gripper_command,queue_size=1)
        rospy.Subscriber("fingers_position",finger_position,self.gripper_cb,queue_size=1)
        self._fingers_moving = False
        self._finger_pos = 0

    """ Callback for gripper subscriber """
    def gripper_cb(self,msg):
        self._finger_pos = msg.position
        self._fingers_moving = msg.moving

    """ Method to move the gripper """
    def move_gripper(self,width):
        if self._finger_pos != width:
            gripper_cmd_msg = gripper_command(position=width,speed=255,force=100)
            gripper_cmd_msg.header.stamp = rospy.Time.now()
            self._gripper_pub.publish(gripper_cmd_msg)
            while not self._fingers_moving:
                continue
            while self._fingers_moving:
                continue

    def check_execution(self,robot):
        r = rospy.Rate(200)
        while (self._as.is_active() and not rospy.is_shutdown()):
            if self._as.is_preempt_requested():
                robot.stop()
                break
            r.sleep()

    def execute_cb(self, action_goal):
        robot = Robot("1")

        thread = Thread(target=self.check_execution, args=(robot,)) # spin the thread for stopping the motion in case of preemption
        thread.start()

        result = WiresGraspResult()

        num_wires = action_goal.num_wires
        if(action_goal.connector_name == "MO6" or num_wires == -1):
            x_ = -action_goal.offset
            y_ = 0
            z_ = 0
            self._target_link = "fingerpad"
        else:
            self._target_link = "fingernails"
            length = self._connectors["components"][action_goal.connector_name]["length"]
            length = length/1000
            pitch = self._connectors["components"][action_goal.connector_name]["pitch"]
            pitch = pitch/1000
            ending_gap = self._connectors["components"][action_goal.connector_name]["ending_gap"]
            ending_gap = ending_gap/1000
            holes_diameter = self._connectors["components"][action_goal.connector_name]["holes_diameter"]
            holes_diameter = holes_diameter/1000
            correction = 0.0010
            x_ = -action_goal.offset
            y_ = 0
            z_ = -length/2 + ending_gap + (num_wires-1)*pitch + holes_diameter - correction
        
        reference_frame = "holder_" + action_goal.connector_name + "_tf"
        aborted = False
        
        try:
            #move up to be sure that there are no obstacles in the next move
            if not self._as.is_preempt_requested():
                self.move_gripper(100)
                robot.move(Lin(goal = Pose(position = Point(0, 0, -0.1), orientation = Quaternion(0,0,0,1)),
                        target_link = self._target_link,
                        reference_frame = self._target_link,
                        planning_group = self._planning_group,
                        vel_scale = action_goal.velocity_scale,
                        acc_scale= 0.01,
                        relative = False
                        ))
            #move above target position, align orientation
            if not self._as.is_preempt_requested():
                robot.move(Lin(goal = Pose(position = Point(x_, y_, z_ -0.1), orientation = Quaternion(0,0,0,1)),
                        target_link = self._target_link,
                        reference_frame = reference_frame,
                        planning_group = self._planning_group,
                        vel_scale = action_goal.velocity_scale,
                        acc_scale= 0.01,
                        relative = False
                        ))

            #move to target position
            if not self._as.is_preempt_requested():
                robot.move(Lin(goal = Pose(position = Point(x_, y_, z_), orientation = Quaternion(0,0,0,1)),
                        target_link = self._target_link,
                        reference_frame = reference_frame,
                        planning_group = self._planning_group,
                        vel_scale = action_goal.velocity_scale,
                        acc_scale= 0.01,
                        relative = False
                        ))
                self.move_gripper(210)
        except:
            aborted = True
            print("something wrong happened")
            
        if self._as.is_preempt_requested() or aborted:
            result.success = False
            self._as.set_preempted(result)
        else:
            result.success = True
            self._as.set_succeeded(result)
            print("Success")
        
        robot._release()

if __name__ == "__main__":
    rospy.init_node("graspwiresserver")

    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path("scene_creation")
    with open(pkg_path + "/cfg/components.yaml") as file:
        connectors = yaml.load(file, Loader=yaml.FullLoader)

    server = GraspWiresActionServer('grasp_wires', connectors)
    rospy.spin()
