#!/usr/bin/env python

import rospy
import actionlib
import tf2_ros
from pilz_robot_programming.robot import Robot
from pilz_robot_programming.commands import Sequence, Lin, Circ, from_euler
import math
import numpy as np
from trajectory_control_msgs.msg import PushDownAction, PushDownGoal, PushDownResult, PushDownFeedback
from geometry_msgs.msg import Pose, TransformStamped, Point, Quaternion 
from hand_e_pkg.msg import gripper_command, finger_position
from threading import Thread
class PushDownActionServer(object):
        
    def __init__(self, name):
        self._planning_group = "manipulator"
        self._target_link = "fingernails"
        self._base = "base"
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, PushDownAction, execute_cb=self.execute_cb, auto_start = False)
        self._direction = 1
        self._x_offset = 0.02
        self._z_offset = 0.005
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
    
        try:
            tcp_pose = robot.get_current_pose(target_link=self._target_link,
                                              base=self._base)
        except:
            aborted = True

        result = PushDownResult()

        if(action_goal.direction == -1):
            self._direction = -1
            quaternion = from_euler(0, 0, math.radians(180))
        else:
            self._direction = 1
            quaternion = from_euler(0, 0, 0)

        aborted = False

        clip_side = - action_goal.clip_side * action_goal.direction
        try:
            #linear movement
            if not self._as.is_preempt_requested():
                self.move_gripper(145)
                # go above the clips (absolute position of fingernails wrt base frame)
                tcp_pose.position.z = 0.14
                robot.move(Lin(goal = tcp_pose,
                               reference_frame = self._base,
                               target_link = self._target_link,
                               planning_group = self._planning_group,
                               vel_scale = action_goal.velocity_scale,
                               acc_scale= 0.1,
                               relative = False))
                # go above the point where to push
                robot.move(Lin(goal = Pose(position = Point(self._x_offset * clip_side,0.0,-0.079),
                                           orientation = quaternion),
                               reference_frame = action_goal.frame,
                               target_link = self._target_link,
                               planning_group = self._planning_group,
                               vel_scale = action_goal.velocity_scale,
                               acc_scale= 0.1,
                               relative = False))
                # push down
                self.move_gripper(230)
                robot.move(Lin(goal = Pose(position = Point(self._x_offset * clip_side,0.0,-self._z_offset),
                                           orientation = quaternion),
                               reference_frame = action_goal.frame,
                               target_link = self._target_link,
                               planning_group = self._planning_group,
                               vel_scale = action_goal.velocity_scale,
                               acc_scale= 0.1,
                               relative = False))
                # go up
                try:
                    tcp_pose = robot.get_current_pose(target_link=self._target_link,
                                                    base=self._base)
                except:
                    aborted = True
                tcp_pose.position.z = 0.14
                robot.move(Lin(goal = tcp_pose,
                               reference_frame = self._base,
                               target_link = self._target_link,
                               planning_group = self._planning_group,
                               vel_scale = action_goal.velocity_scale,
                               acc_scale= 0.1,
                               relative = False))
        except:
            print("something wrong happened")
            aborted = True

        if self._as.is_preempt_requested() or aborted:
            result.success = False
            self._as.set_preempted(result)
        else:
            result.success = True
            self._as.set_succeeded(result)
            print("Success")
        
        robot._release()


if __name__ == "__main__":
    rospy.init_node("pushdown_server")
    server = PushDownActionServer('push_down')
    rospy.spin()



