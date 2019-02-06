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
        self.states_pub=rospy.Publisher("/sm/sm_states",Slug_state,queue_size=50)
        self.slug_msg = Slug_state()
        self.Initial_time = rospy.get_time()


        obstacle2_topic="/obstacle2_Is_Occupied"
        rospy.Subscriber(obstacle2_topic, Bool, self.obstacle2_Callback)
        obstacle3_topic="/obstacle3_Is_Occupied"
        rospy.Subscriber(obstacle3_topic, Bool, self.obstacle3_Callback)
        robot_pose_topic="global_pose"
        rospy.Subscriber(robot_pose_topic, PoseStamped, self.pose_callback)
        task_sm_topic='SM/current_state'
        rospy.Subscriber(task_sm_topic,Int32MultiArray,self.Task_SM_Callback)

        # initial workload
        self.slug_msg.workload=16

    def publish_slug_state(self):
        curr_time=rospy.get_time()
        duration = int(curr_time -self.Initial_time)
        self.calculate_statesvariables(duration)
        self.states_pub.publish(self.slug_msg)

    def pose_callback(self,msg):
        # rospy.loginfo('global_pose_callback')
        robot_pos = msg.pose
        #Todo: change w.r.t robot poses
        if robot_pos.position.y <0.2:
            self.slug_msg.r_state = 1
        elif robot_pos.position.y <-0.49:
            self.slug_msg.r_state = 2
        elif robot_pos.position.y <-0.99:
            self.slug_msg.r_state = 3
        elif robot_pos.position.y <-2.5: 
            self.slug_msg.r_state = 4
        
    def obstacle2_Callback(self,msg):
        # rospy.loginfo('obstacle2_callback')
        # self.slug_msg.obstacle2=int(msg.data)
        if msg.data==True:
            self.slug_msg.obstacle2=1
        else:
            self.slug_msg.obstacle2=0
        # msg.data

    def obstacle3_Callback(self,msg):
        # rospy.loginfo('obstacle3_callback')
        # self.slug_msg.obstacle3=int(msg.data)
        if msg.data==True:
            self.slug_msg.obstacle3=1
        else:
            self.slug_msg.obstacle3=0

    def Task_SM_Callback(self,msg):
         self.Initial_time = rospy.get_time()


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
        
        #calculate workload
        previous_workload = self.slug_msg.workload
        self.slug_msg.workload=self.slug_msg.workload-int(time_duration%10)

        if previous_workload == self.slug_msg.workload:
            self.slug_msg.workload_stays_constant = 1
        else: 
            self.slug_msg.workload_stays_constant = 0


        # calculate if waiting
        if self.slug_msg.workload > 0:
            self.slug_msg.wait=0
        else: 
            self.slug_msg.wait=1

        # todo: complete dropoff tries
        self.slug_msg.complete_dropoff_tries=0


        # todo: complete dropoff success
        self.slug_msg.complete_dropoff_tries=0

        self.slug_msg.workload_stays_constant=0
        
        # placeholders for controllable variables. Not used
        self.slug_msg.complete_work_at_workstation=0
        self.slug_msg.workload_add=0
        self.slug_msg.next_state_is_workstation=0
        self.slug_msg.complete_work_with_robot=0
        self.slug_msg.arriving_at_0=0

        if self.slug_msg.r_state == 4:
            self.slug_msg.complete_work_with_robot=1
        elif self.slug_msg.complete_work_with_robot==1 and self.slug_msg.r_state != 1:
            self.slug_msg.complete_work_with_robot=1
        else: 
            self.slug_msg.complete_work_with_robot=0
    
        
        if self.slug_msg.complete_work_with_robot==1 and self.slug_msg.r_state == 1:
            self.slug_msg.r_state=0

    def listener(self,wait=0.0):
        # rospy.spin()
        while not rospy.is_shutdown():
            self.publish_slug_state()
            rospy.Rate(3).sleep()


if __name__ == '__main__':
        rospy.init_node('environment_sensing')
        manager = Environment_Manager(sys.argv[1] if len(sys.argv) >1 else 0.0)
	manager.listener()	

