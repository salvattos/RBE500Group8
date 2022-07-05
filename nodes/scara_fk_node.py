#!/usr/bin/env python
import rospy
from sensor_msgs.msg import *
from geometry_msgs.msg import *
import tf
import numpy as np
import math


def callback(fkData):
    rospy.loginfo(rospy.get_caller_id() + " Calculating Forward Kinematics")
    #print("made it")
    # Initialize publisher
    # Note: if this doesnt work moving this line to the main function might fix it
    fk_pub = rospy.Publisher('scara_fk', Pose, queue_size=10)

    
    # Not sure what the actual lengths are, you will need to update them
    l1 = 1
    l2 = 1
    l3 = 1
    # Not sure if this is the correct reference, but it is probably close
    t1 = fkData.position[0]
    t2 = fkData.position[1]
    d3 = fkData.position[2]

    rate = rospy.Rate(10) # 10hz
    if not rospy.is_shutdown():

        # Might need to double check this, very similar tf matrix can be found on page 27 of lecture 2 notes
        zeroTothree = np.array([[math.cos(t1 + t2), -math.sin(t1 + t2), 0, (-l3*math.sin(t1+t2))-(l2*math.sin(t1))],
                                [math.sin(t1+t2), math.cos(t1+t2), 0,
                                (l3*math.cos(t1+t2))+(l2*math.cos(t1))],
                                [0, 0, 1, l1-d3],
                                [0, 0, 0, 1]])
        # Not sure how to import this correctly, probably needs to be fixed
        q = tf.transformations.quaternion_from_matrix(zeroTothree)
        # Create pose object and populate it
        p = Pose()
        p.position.x = zeroTothree[0][3]
        p.position.y = zeroTothree[1][3]
        p.position.z = zeroTothree[2][3]
        p.orientation.x = q[0]
        p.orientation.y = q[1]
        p.orientation.z = q[2]
        p.orientation.w = q[3]
        # publish pose object to scara_fk topic
        fk_pub.publish(p)

        rospy.loginfo(rospy.get_caller_id() + "Published Pose")



def scara_fk_node():
    rospy.loginfo(rospy.get_caller_id() + "started FK node")
    rospy.init_node('scara_fk_node', anonymous=True)
    rospy.Subscriber("/rrbot/joint_states", JointState, callback)
    
    rospy.spin()


if __name__ == '__main__':
    scara_fk_node()
