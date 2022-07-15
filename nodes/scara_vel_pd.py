#!/usr/bin/env python
from ntpath import join
from rbe500GroupProject.srv import setVelocity, setVelocityResponse
import math
import numpy as np
import tf
from gazebo_msgs.msg import *
from gazebo_msgs.srv import *
from geometry_msgs.msg import *
from sensor_msgs.msg import *
import rospy