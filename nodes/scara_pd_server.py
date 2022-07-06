#!/usr/bin/env python
from rbe500GroupProject.srv import EffectorGoToPosition, EffectorGoToPositionResponse
import math
import numpy as np
import tf
from gazebo_msgs.msg import *
from gazebo_msgs.srv import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *
import rospy

class scaraPD:
    def __init__(self):
        """
        Class constructor
        """
        self.d3_current_value = 0
        self.d3_current_pos = 1
        self.kp = .5

        ### Initialize node, name it 'HW2'
        #This needs to have the scara_fk_node running in order to get current EE position
        rospy.init_node('scara_pd_server')
        rospy.loginfo(rospy.get_caller_id() + " started PD Control node")
        rospy.Subscriber("/rrbot/joint_states", JointState, self.update_d3_value)
        rospy.Subscriber("/scara_fk", Pose, self.update_d3_pos)
        s = rospy.Service('scara_pd_control_server',EffectorGoToPosition, self.pd_callback)
    
    def update_d3_value(self,jointData):
        self.d3_current_value = jointData.position[2]
    
    def update_d3_pos(self,jointData):
        self.d3_current_pos = jointData.position.z

    def pd_callback(self,req):
        rospy.loginfo(rospy.get_caller_id() +
                    " PD callback. Initializing PID Controller")

        start = rospy.get_time()
        dt = .1
        desiredPos = req.z
        error = desiredPos - self.d3_current_pos
        rospy.loginfo(error)
        while(abs(error) > .1):
            rospy.loginfo(error)
            error = desiredPos - self.d3_current_pos
            effort = error*self.kp

            if(effort > 0 and effort < 1):
                effort = -1
            if(effort < 0 and effort > -1):
                effort = 1

            time = Time(0,0)
            duration = Duration(1,0)
            apply_Joint_Effort = rospy.ServiceProxy('/gazebo/apply_joint_effort', ApplyJointEffort)
            #effort must be > 1 or < -1 for some reason. An effort of .5 or -.5 doesnt move anything
            aplyJointEffort = apply_Joint_Effort('prismatic',effort,time,duration)
            rospy.sleep(dt)

        return EffectorGoToPositionResponse("Reached desired position")


    def run(self):
        rospy.spin()
class Time:
    def __init__(self,secs,nsecs):
        self.secs = secs
        self.nsecs = nsecs
class Duration:
    def __init__(self,secs,nsecs):
        self.secs = secs
        self.nsecs = nsecs


if __name__ == '__main__':
    scaraPD().run()