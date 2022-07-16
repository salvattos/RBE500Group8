#!/usr/bin/env python
from rbe500GroupProject.srv import setVelocity, setVelocityResponse
from rbe500GroupProject.srv import inverseVK, inverseVKResponse
import math
import numpy as np
import tf
from gazebo_msgs.msg import *
from gazebo_msgs.srv import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from std_msgs.msg import *
import rospy


def velocityController(req):
    t0Pub = rospy.Publisher('/rrbot/joint0_velocity_controller/command', Float64, queue_size = 10)
    t2Pub = rospy.Publisher('/rrbot/joint2_velocity_controller/command', Float64, queue_size = 10)
    d3Pub = rospy.Publisher('/rrbot/jointPris_velocity_controller/command', Float64, queue_size = 10)

    get_qdots = rospy.ServiceProxy('scara_IVK_server', inverseVK)
    
    startTime = rospy.get_time()
    dt = .1

    while rospy.get_time() - startTime < 5:
        #get current qDots
        qDots = get_qdots(req.xDot,req.yDot,req.zDot)
        #send them to each joint topic
        t0Pub.publish(qDots.qDot[0])
        t2Pub.publish(qDots.qDot[1])
        d3Pub.publish(qDots.qDot[2])

        rospy.sleep(dt)

    #stop all
    t0Pub.publish(0.0)
    t2Pub.publish(0.0)
    d3Pub.publish(0.0)
    rospy.sleep(dt)




def scara_VK_node():
    rospy.init_node('scara_IVK_PD_node', anonymous=True)
    rospy.loginfo(rospy.get_caller_id() + "started IVK PD node")
    s = rospy.Service('scara_IVK_PD_server',setVelocity, velocityController)

    rospy.spin()


if __name__ == '__main__':
    scara_VK_node()