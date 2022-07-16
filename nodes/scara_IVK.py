#!/usr/bin/env python
import rospy
import numpy as np
import math
from gazebo_msgs.msg import *
from gazebo_msgs.srv import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from rbe500GroupProject.srv import forwardVK, forwardVKResponse
from rbe500GroupProject.srv import inverseVK, inverseVKResponse
curr_t1 = 0
curr_t2 = 0
curr_d3 = 0

def calc_inverseVK(req):
    pDot = np.array([[req.xdot],[req.ydot],[req.zdot]])
    jac = getJacobian()
    invJ = np.linalg.pinv(jac[0:3][0:3])

    qDot = np.matmul(invJ,pDot)

    t1Dot = float(qDot[0])
    t2Dot = float(qDot[1])
    d3Dot = float(qDot[2])

    return inverseVKResponse([t1Dot,t2Dot,d3Dot])

def calc_forwardVK(req):
    qDot = np.array([[req.t1dot],[req.t2dot],[req.d3dot]])
    jac = getJacobian()
    eeDot = np.matmul(jac,qDot)

    rospy.loginfo(eeDot)
    xDot = float(eeDot[0])
    yDot = float(eeDot[1])
    zDot = float(eeDot[2])

    return forwardVKResponse([xDot, yDot,zDot])


def getJacobian():
    J = np.array([[ - math.cos(curr_t1 + curr_t2) - math.cos(curr_t1), -math.cos(curr_t1 + curr_t2),  0],
[ - math.sin(curr_t1 + curr_t2) - math.sin(curr_t1), -math.sin(curr_t1 + curr_t2),  0],
[                        0,             0, -1],
[                        0,             0,  0],
[                        0,             0,  0],
[                        1,             1, -1]])

    return J


def updateJointValues(jointData):
    global curr_t1
    global curr_t2
    global curr_d3
    curr_t1 = jointData.position[0]
    curr_t2 = jointData.position[1]
    curr_d3 = jointData.position[2]

def scara_VK_node():
    rospy.loginfo(rospy.get_caller_id() + "started VK node")
    rospy.init_node('scara_VK_node', anonymous=True)
    rospy.Subscriber("/rrbot/joint_states", JointState, updateJointValues)
    s = rospy.Service('scara_FVK_server',forwardVK, calc_forwardVK)
    d = rospy.Service('scara_IVK_server',inverseVK, calc_inverseVK)

    rospy.spin()


if __name__ == '__main__':
    scara_VK_node()