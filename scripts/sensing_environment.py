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
from std_msgs.msg import Bool
from sm_msgs.msg import Sm_states
from sm_project.msg import Slug_state
import std_msgs.msg

from os import listdir
import os

class Environment_Manager(object):
    def __init__(self, wait=0.0):

        obstacle2_topic="/obstacle2_Is_Occupied"
        rospy.Subscriber(obstacle2_topic, Bool, self.obstacle2_Callback)
        obstacle3_topic="/obstacle3_Is_Occupied"
        rospy.Subscriber(obstacle3_topic, Bool, self.obstacle3_Callback)
        robot_pose_topic="global_pose"
        rospy.Subscriber(robot_pose_topic, PoseStamped, self.pose_callback)

        # self.states_pub=rospy.Publisher("/sm/sm_states",Sm_states,queue_size=50)
        self.states_pub=rospy.Publisher("/sm/sm_states",Slug_state,queue_size=50)
        self.slug_msg = Slug_state()
        self.Initial_time = rospy.get_time()

    def publish_slug_state(self):
        curr_time=rospy.get_time()
        duration = int(curr_time -self.Initial_time)
        self.calculate_statesvariables(duration)
        self.states_pub.publish(self.slug_msg)

    def pose_callback(self,msg):
        # rospy.loginfo('global_pose_callback')
        robot_pos = msg.pose
        #Todo: change w.r.t robot poses
        self.slug_msg.r_state = 2
        
    def obstacle2_Callback(self,msg):
        # rospy.loginfo('obstacle2_callback')
        self.slug_msg.obstacle2=msg.data
        # msg.data

    def obstacle3_Callback(self,msg):
        # rospy.loginfo('obstacle3_callback')
        self.slug_msg.obstacle3=msg.data

    def calculate_statesvariables(self, time_duration):
        #calulate other variables in slug_states
        #obstacles are automatically updated from others
        #workload
        #complete_work_at_workstation
        #complete_dropoff_success
        #complete_dropoff_tries
        #workload_add
        #next_state_is_workstation
        #complete_work_with_robot
        #arriving_at_0
        # rospy.loginfo('duration %d',int(time_duration))
        
        self.slug_msg.wait=False
        self.slug_msg.workload=time_duration*0.3
        self.slug_msg.complete_work_at_workstation=False
        self.slug_msg.complete_dropoff_tries=0
        self.slug_msg.workload_add=0
        self.slug_msg.next_state_is_workstation=False
        self.slug_msg.complete_work_with_robot=False
        self.slug_msg.arriving_at_0=False
        
    

    def listener(self,wait=0.0):
        # rospy.spin()
        while not rospy.is_shutdown():
            self.publish_slug_state()
            rospy.Rate(3).sleep()


if __name__ == '__main__':
        rospy.init_node('environment_sensing')
        manager = Environment_Manager(sys.argv[1] if len(sys.argv) >1 else 0.0)
	manager.listener()	

