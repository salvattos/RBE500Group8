#!/usr/bin/env python
##############################################
#          RBE500  GROUP 8 - Part 1
#      Created/Modified on: June 21, 2022
#           Authors: Munir Jojo-Verge
#                    Shanw Salvatto
##############################################

import rospy
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from tf import *
from rbe500GroupProject.srv import scaraIK, scaraIKResponse
import numpy as np
import math


def handle_scara_ik(req):
    rospy.loginfo(rospy.get_caller_id() + " Begining IK")
    y = req.x
    x = req.y
    z = req.z
    # need to update these
    l1 = 1
    l2 = 1
    l3 = 1
    # This is weird: We are hoping that D < 1, otherwise sqrt below will fail!
    D = -(pow(l1, 2)+pow(l2, 2)-pow(x, 2)-pow(y, 2))/(2*l1*l2)
    if (D > 1):
        rospy.loginfo(rospy.get_caller_id() + " IK failed due to D > 1")
        return scaraIKResponse([0, 0, 0])

    C = math.sqrt(1-pow(D, 2))
    beta = math.atan2(C, D)
    alpha = math.atan2(y, x)

    t1 = alpha-beta
    t2 = math.atan2(C, D)
    t3 = l3-z

    return scaraIKResponse([t1, t2, t3])


def scara_ik_server():
    rospy.init_node('scara_ik_server')
    s = rospy.Service('scara_ik', scaraIK, handle_scara_ik)
    rospy.loginfo(rospy.get_caller_id() + "Ready to perform IK.")
    rospy.spin()


if __name__ == '__main__':
    scara_ik_server()
