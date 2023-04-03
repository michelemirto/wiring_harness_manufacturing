#!/usr/bin/env python

import rospy
import actionlib
import tf2_ros
from pilz_robot_programming.robot import Robot
from pilz_robot_programming.commands import Sequence, Lin, Circ, from_euler
import math
import numpy as np
from trajectory_control_msgs.msg import RoutingAction, RoutingGoal, RoutingResult, RoutingFeedback
from geometry_msgs.msg import Pose, TransformStamped, Point, Quaternion 
from hand_e_pkg.msg import gripper_command, finger_position
from threading import Thread
class RoutingActionServer(object):
        
    def __init__(self, name):
        self._planning_group = "manipulator"
        self._target_link = "fingerpad"
        self._base = "base"
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, RoutingAction, execute_cb=self.execute_cb, auto_start = False)
        self.mult = 1 #default multiplier. Set to -1 in action goal in case of MO6 routing
        self.height_clip = 0.06204 #distance between frame origin and clip top part
        self.pad_offset = 0.01565 #distance between fingerpad frame origin and finger bottom part
        self.finger_width = 0.013 
        self.hole_distance = 0.03197 #distance between the two holes of the clip
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

        x_offset_before = self.finger_width #offset between clip top part and the end of first linear trajectory
        x_offset_after = action_goal.x_offset #offset between clip top part and the end of second linear trajectory

        result = RoutingResult()

        #In case of MO6 routing, direction in action goal is -1.
        #orientation is rotated 180 degrees
        if(action_goal.direction == -1):
            self.mult = -1
            quaternion = from_euler(0, 0, math.radians(180))
            quaternion_circ = from_euler(0, 0, math.radians(180) + math.radians(action_goal.z_rotation))
        else:
            self.mult = 1
            quaternion = from_euler(0, 0, 0)
            quaternion_circ = from_euler(0, 0, math.radians(action_goal.z_rotation))

        #If up_hole in action goal is True, routing is stopped at the first hole of the clip
        #hole_offset will be added to the z coordinate
        if(action_goal.up_hole):
            hole_offset = self.hole_distance
        else:
            hole_offset = 0.0
        
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)

        #listen the relative position between current frame (e.g. clip1) and next frame (e.g. clip2)
        try:
            transform_n_c = tfBuffer.lookup_transform(action_goal.next_frame, action_goal.current_frame, rospy.Time(0), rospy.Duration(1));
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.loginfo("exception in lookupTransform3")
            result.success = False
            self._as.set_succeeded(result)
        
        #array p_nc contains the position of current_frame wrt next_frame
        p_nc = np.array([transform_n_c.transform.translation.x,
                          transform_n_c.transform.translation.y,
                          transform_n_c.transform.translation.z])     
        quat_nc = transform_n_c.transform.rotation

        #The objective is to move the robot (i.e. fingerpad frame) of a "radius" distance from current_frame to a position
        #above next_frame. The orientation must be aligned with next_frame 
        #After reaching this position with two linear movement, the robot makes a circular movement and reachs the final position.
        #The circular movement has the center in current_frame and radius equal to "radius"

        #The final position is at z = -hole_offset, y = y_offset and x = -Dx-x_offset_after from next_frame
        #hole_offset is 0 if routing must stop at the clip second hole
        # y_offset is a quantity that depends on the fingers rotation during routing
        # Dx is computed from tcp pose above next_frame and "radius" 

        #the position above next_frame Dz is computed as an offset plus the height of the clip from 
        # next_frame plus the offset between the fingerpad frame (i.e. tcp frame) and the end of the finger

        #if action goal field "lateral" is not 0, the position above next_frame is reached in two steps. 
        #First: a linear movement is executed wrt next_frame, to a position x = frame_distance/3 - x_offset_after
        #y computed to reach a distance from next_frame equal to "lateral_radius" and z = 0
        #lateral_ radius is equal to Dz
        #Second: a circular movement with center in next_frame and radius equal to "lateral_radius"

        aborted = False
        try:
            Dz = action_goal.z_offset + self.height_clip + self.pad_offset
            y_offset = 0.0 #(0.5 * (self.finger_width+0.015) * math.tan(math.radians(action_goal.z_rotation)))
            
            if(action_goal.lateral == 0):
                #no lateral movement required
                x_ = (x_offset_before)*self.mult
                y_ = - y_offset
                z_ = - Dz
                
                #first linear movement
                if not self._as.is_preempt_requested():
                    self.move_gripper(210)
                    robot.move(Lin(goal = Pose(position = Point(x_,y_,z_),orientation = quaternion_circ),
                            reference_frame = action_goal.next_frame,
                            target_link = self._target_link,
                            planning_group = self._planning_group,
                            vel_scale = action_goal.velocity_scale,
                            acc_scale= 0.1,
                            relative = False))
                    
                    x_ = (-x_offset_after)*self.mult

                    #second linear movement
                    robot.move(Lin(goal = Pose(position = Point(x_,y_,z_),orientation = quaternion_circ),
                            reference_frame = action_goal.next_frame,
                            target_link = self._target_link,
                            planning_group = self._planning_group,
                            vel_scale = action_goal.velocity_scale,
                            acc_scale= 0.1,
                            relative = False))
                
            else:
                #lateral linear movement required
                
                lateral_radius = -Dz
                frame_distance = math.sqrt(p_nc[0]**2 + p_nc[1]**2)

                x_ = frame_distance/3 - x_offset_after
                y_ = math.sqrt(lateral_radius**2 - (x_)**2) * action_goal.lateral
                z_ = 0

                if not self._as.is_preempt_requested():
                    self.move_gripper(210)
                    robot.move(Lin(goal = Pose(position = Point(x_,y_,z_),orientation = quaternion_circ),
                            reference_frame = action_goal.next_frame,
                            target_link = self._target_link,
                            planning_group = self._planning_group,
                            vel_scale = action_goal.velocity_scale,
                            acc_scale= 0.1,
                            relative = False))

                    #lateral circular movement   
                    x_ = (-x_offset_after)*self.mult         
                    y_ = - y_offset
                    z_ = - Dz
                    # self.move_gripper(230)
                    robot.move(Circ(goal = Pose(position = Point(x_,y_,z_),orientation = quaternion_circ),
                            center = Point(0, 0, 0),
                            reference_frame = action_goal.next_frame,
                            target_link = self._target_link,
                            planning_group = self._planning_group,
                            vel_scale = action_goal.velocity_scale,
                            acc_scale= 0.1,
                            relative = False
                            ))
                
            #the circ center is in current_frame
            #radius is computed as the distance between the current tcp position and current_frame
            
            tcp_pose = robot.get_current_pose(target_link=self._target_link,
                                              base=action_goal.current_frame)
            x_tcp = tcp_pose.position.x
            y_tcp = tcp_pose.position.y
            z_tcp = tcp_pose.position.z

            radius = math.sqrt(x_tcp **2 + y_tcp **2 + z_tcp **2)
            Dx = radius - math.sqrt(x_tcp **2 + y_tcp **2)

            #circular movement
            z_ = -hole_offset
            y_ = - y_offset
            x_ = (- Dx - x_offset_after)*self.mult
            if not self._as.is_preempt_requested():
                self.move_gripper(240)
                robot.move(Circ(goal = Pose(position = Point(x_,y_,z_), orientation = quaternion_circ), 
                            center = Point(p_nc[0], p_nc[1], p_nc[2]), 
                            reference_frame = action_goal.next_frame, 
                            target_link = self._target_link,
                            vel_scale = action_goal.velocity_scale,
                            acc_scale= 0.1,
                            relative = False
                            ))
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
    rospy.init_node("routingserver")
    server = RoutingActionServer('routing')
    rospy.spin()