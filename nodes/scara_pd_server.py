#!/usr/bin/env python
from ntpath import join
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
        self.current_t0 = 0
        self.current_t2 = 0

        self.kp = 2
        self.kd = 1.5

        self.t0_kp = .75
        self.t0_kd = 1.25

        self.t2_kp = 2
        self.t2_kd = .5

        self.apply_Joint_Effort = rospy.ServiceProxy('/gazebo/apply_joint_effort', ApplyJointEffort)
        self.clear_Joint_Forces = rospy.ServiceProxy('/gazebo/clear_joint_forces', JointRequest)

        ### Initialize node, name it 'HW2'
        #This needs to have the scara_fk_node running in order to get current EE position
        rospy.init_node('scara_pd_server')
        rospy.loginfo(rospy.get_caller_id() + " started PD Control node")
        rospy.Subscriber("/rrbot/joint_states", JointState, self.updateJointValues)
        rospy.Subscriber("/scara_fk", Pose, self.update_d3_pos)
        s = rospy.Service('scara_pd_control_server',EffectorGoToPosition, self.pd_callback)
    
    def updateJointValues(self,jointData):
        self.current_t0 = jointData.position[0]
        self.current_t2 = jointData.position[1]
        self.d3_current_value = jointData.position[2]
    
    def update_d3_pos(self,jointData):
        self.d3_current_pos = jointData.position.z

    def pd_callback(self,req):
        rospy.loginfo(rospy.get_caller_id() +
                    " PD callback. Initializing PID Controller")

        trigger = 0
        dt = .1

        desiredPosPris = req.z
        errorPris = desiredPosPris - self.d3_current_pos
        lastPosPris = self.d3_current_pos

        desiredT0 = req.x #need to update dynamically
        errorT0 = desiredT0 - self.current_t0
        lastT0 = self.current_t0

        desiredT2 = req.y #need to update dynamically
        errorT2 = desiredT2 - self.current_t2
        lastT2 = self.current_t2

        rospy.loginfo(errorPris)
        while(trigger < 5):
            
            errorPris = desiredPosPris - self.d3_current_pos
            effortPris = (-errorPris*self.kp) + (self.kd*((self.d3_current_pos-lastPosPris)/dt))
            lastPosPris = self.d3_current_pos

            errorT0 = desiredT0 - self.current_t0
            effortT0 = errorT0*self.t0_kp + (self.t0_kd*((lastT0-self.current_t0)/dt))
            lastT0 = self.current_t0

            errorT2 = desiredT2 - self.current_t2
            effortT2 = errorT2*self.t2_kp + (self.t2_kd*((lastT2-self.current_t2)/dt))
            lastT2 = self.current_t2
                
            rospy.loginfo(effortT2)
            rospy.loginfo(self.current_t2)
            self.sendEffort(effortT2,'joint2')
            self.sendEffort(effortT0,'joint0')
            #self.sendEffort(effortPris,'prismatic')

            rospy.sleep(dt)

            #Make sure the EE is in the desired position for a few cycles
            if abs(errorPris) < .01 and abs(errorT0) < .05 and abs(errorT2) < .01:
                trigger = trigger + 1
            else:
                trigger = 0
                

        self.stopAll()    
        rospy.loginfo("D3 Pos")
        rospy.loginfo(self.d3_current_pos)
        return EffectorGoToPositionResponse("Reached desired position")

    def sendEffort(self, effort, jointName):
        time = Time(0,0)
        duration = Duration(1,0)
        clearJointForces = self.clear_Joint_Forces(jointName)
        applyJointEffort = self.apply_Joint_Effort(jointName,effort,time,duration)
    def stopAll(self):
        self.sendEffort(0,'prismatic')
        self.sendEffort(0,'joint_0')
        self.sendEffort(0,'joint_2')

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