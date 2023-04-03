#! /usr/bin/env python

import rospy
import actionlib
from pilz_robot_programming.robot import Robot
from pilz_robot_programming.commands import Ptp, Lin, Sequence, Gripper
from geometry_msgs.msg import Pose, Point, Quaternion
from hand_e_pkg.msg import gripper_command, finger_position
from threading import Thread
from math import pi

from trajectory_control_msgs.msg import ConnectorPickPlaceAction, ConnectorPickPlaceGoal, ConnectorPickPlaceResult, ConnectorPickPlaceFeedback

class ConnectorPickPlaceActionServer(object):
    # create messages that are used to publish feedback/result

    def __init__(self, name):
        self._planning_group = "manipulator"
        self._target_link = "fingerpad"
        self._base = "base"
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, ConnectorPickPlaceAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        self._grasp_pos = [-0.4771660009967249, -1.812564035455221, 2.091193501149313, -3.422630926171774, -1.0954487959491175, 0.00807556311552915 ]
        self._home_pos = [ 1.4511696100234985, -1.752373834649557, 1.8177664915667933, -1.63480867962026, -1.5647791067706507, -0.11663006543717991 ]
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

    """ Function to stop the robot motion in case of goal preemption"""
    def check_execution(self,robot):
        r = rospy.Rate(200)
        while (self._as.is_active() and not rospy.is_shutdown()):
            if self._as.is_preempt_requested():
                robot.stop()
                break
            r.sleep()

    """ Function for goal execution"""
    def execute_cb(self, action_goal):
        robot = Robot("1")
        
        pick_frame = action_goal.pick_frame
        place_frame = action_goal.place_frame
        holder_length = action_goal.holder_length
        connector_width = action_goal.connector_width
        offset = action_goal.offset
        v_scale = action_goal.velocity_scale

        
        thread = Thread(target=self.check_execution, args=(robot,)) # spin the thread for stopping the motion in case of preemption
        thread.start()

        aborted = False
        try:        
            if not self._as.is_preempt_requested():
                self.move_gripper(100)
                # go to grasp position (joints)
                robot.move(Ptp(goal=self._grasp_pos,
                            relative=False, vel_scale=v_scale, acc_scale=0.1,
                            reference_frame=self._base, planning_group=self._planning_group, target_link=self._target_link
                            ))

            if not self._as.is_preempt_requested():
                # go near the connector (before the approach)
                robot.move(Lin(goal=Pose(position=Point(-(holder_length+offset-connector_width),0.0,-0.05)),
                            relative=False, vel_scale=v_scale, acc_scale=0.1, 
                            reference_frame=pick_frame, planning_group=self._planning_group, target_link=self._target_link
                            ))
            
            if not self._as.is_preempt_requested():
                ### approach the connector (5 cm currently)
                robot.move(Lin(goal=Pose(position=Point(-(holder_length+offset-connector_width),0.0,0.0)),
                            relative=False, vel_scale=v_scale, acc_scale=0.1, 
                            reference_frame=pick_frame, planning_group=self._planning_group, target_link=self._target_link
                            ))
            
                """ Here we need to close the gripper before to continue """
                self.move_gripper(240)

            if not self._as.is_preempt_requested():
                ### go up to remove the connector from the warehouse
                robot.move(Lin(goal=Pose(position=Point(0.01,0.0,0.0)),
                                relative=True, vel_scale=v_scale, acc_scale=0.1, 
                                reference_frame=pick_frame, planning_group=self._planning_group, target_link=self._target_link
                                ))
            
            if not self._as.is_preempt_requested():
                ### retract the gripper
                robot.move(Lin(goal=Pose(position=Point(0.0,0.0,-0.07)),
                                relative=True, vel_scale=v_scale, acc_scale=0.1, 
                                reference_frame=pick_frame, planning_group=self._planning_group, target_link=self._target_link
                                ))
            
            if not self._as.is_preempt_requested():
                # go home position (joints)
                robot.move(Ptp(goal=self._home_pos,
                            relative=False, vel_scale=v_scale, acc_scale=0.1,
                            reference_frame=self._base, planning_group=self._planning_group, target_link=self._target_link
                            ))

            if not self._as.is_preempt_requested():
                ### go above the place point
                robot.move(Lin(goal=Pose(position=Point(-offset,0.0,-0.07)),
                                relative=False, vel_scale=v_scale, acc_scale=0.1, 
                                reference_frame=place_frame, planning_group=self._planning_group, target_link=self._target_link
                                ))
            
            if not self._as.is_preempt_requested():
                ### insert the connector
                robot.move(Lin(goal=Pose(position=Point(-offset,0.0,0.0)),
                                relative=False, vel_scale=v_scale, acc_scale=0.1, 
                                reference_frame=place_frame, planning_group=self._planning_group, target_link=self._target_link
                                ))

            """ The gripper must stay closed because now the connector has to be locked by using the tactile indicator """
        except:
            print("something wrong happened")
            aborted = True

        result = ConnectorPickPlaceResult()
        if self._as.is_preempt_requested() or aborted:
            result.success = False
            self._as.set_preempted(result)
        else:
            result.success = True
            self._as.set_succeeded(result)
        thread.join()
        robot._release()
        
if __name__ == '__main__':
    try:
        rospy.init_node('pickplaceserver')
        server = ConnectorPickPlaceActionServer('connector_pick_place')
        rospy.spin()
    except rospy.ROSInterruptException:
        pass