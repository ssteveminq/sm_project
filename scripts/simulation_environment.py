#!/usr/bin/env python

import roslib
import sys
import rospy
import random
import numpy as np
import select, termios, tty

from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Bool
from sm_project.msg import Slug_state
import std_msgs.msg

from os import listdir
import os

class Environment_Manager(object):
    def __init__(self, wait=0.0):


        # self.states_pub=rospy.Publisher("/sm/sm_states",Sm_states,queue_size=50)
        self.states_pub=rospy.Publisher("SM/current_state",Int32MultiArray,queue_size=50)
        self.fake_sm=Int32MultiArray()
        self.Initial_time = rospy.get_time()



    def publish_fake_SM_state(self):
        cur_time = rospy.get_time()
        self.fake_sm.data.append(int(cur_time))
        self.states_pub.publish(self.fake_sm)



    def listener(self,wait=0.0):
        # rospy.spin()
        while not rospy.is_shutdown():
            self.publish_fake_SM_state()
            rospy.Rate(0.1).sleep()


if __name__ == '__main__':
        rospy.init_node('simulation_time')
        manager = Environment_Manager(sys.argv[1] if len(sys.argv) >1 else 0.0)
	manager.listener()	

