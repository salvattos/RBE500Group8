#!/usr/bin/env python
import sys
import time
from rbe500GroupProject.srv import EffectorGoToPosition, EffectorGoToPositionResponse
import math
import numpy as np
import tf
from gazebo_msgs import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *
import rospy
from weakref import ref
from subprocess import call

sys.path.append('/home/munir/catkin_ws/src/rbe500GroupProject/nodes')
from pid import *


global start_time 
global last_time
    
def pd_callback(req):
    rospy.loginfo(rospy.get_caller_id() +
                  " PD callback. Initializing PID Controller")

    
    start_time = time.time()
    last_time = start_time
    
    pid_joint3.setpoint = req.x

    return EffectorGoToPositionResponse("Running Controller")


def pd_FeedbackCallback(jointData):

    d3_current_pos = jointData.position[2]
    current_time = time.time()
    dt = current_time - last_time

    joint3_effort = pid_joint3(d3_current_pos)

    apply_joint_effort = rospy.ServiceProxy(
        'gazebo/apply_joint_effort', ApplyJointEffort)
    apply_joint_effort(joint3_effort)
    last_time = current_time

    # Keep Track of data
    t += [current_time - start_time]
    y += [d3_current_pos]
    setpoint += [pid_joint3.setpoint]

    #rospy.loginfo(rospy.get_caller_id() + " Published ")


def scara_pd_control_node():
    rospy.loginfo(rospy.get_caller_id() + " started PD Control node")
    rospy.init_node('scara_pd_control_node', anonymous=True)
    rospy.Subscriber("/rrbot/joint_states", JointState, pd_FeedbackCallback)
    s = rospy.Service('scara_pd_control_server',
                      EffectorGoToPosition, pd_callback)

    rospy.spin()


if __name__ == '__main__':
    pid_joint3 = PID(5, 0.01, 0.1, setpoint=0)
    pid_joint3.output_limits = (0, 1)

    # Keep track of values for plotting
    setpoint, t, y = [], [], []

    scara_pd_control_node()
