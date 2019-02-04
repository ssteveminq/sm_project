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
import std_msgs.msg

from os import listdir
import os


class Environment_Manager(object):
    def __init__(self, wait=0.0):

        obstacle_topic="/Is_Occupied"
        rospy.Subscriber(obstacle_topic, Bool, self.obstacle_Callback)

        self.states_pub=rospy.Publisher("/sm/sm_states",Sm_states,queue_size=50)
        self.Initial_time = rospy.Time.now()


    def Update_Measurement_Filter(self,x_center, y_center):
        #Update the filter with the last measurements
        
        self.my_particle.update(x_center, y_center)

        #Resample the particles
        self.my_particle.resample('stratified')
        current_entropy=self.my_particle.get_entropy()
        ParticlesContribution=self.my_particle.returnParticlesContribution()
        rospy.loginfo("resampledparticle : "+str(current_entropy))
        rospy.loginfo("contribution : "+str(ParticlesContribution))

    def obstacle_Callback(self,msg):
        msg.data


    def PositionMeasurementCb(self,msg):
        #recieve poses array from measurement

        poses_array=msg.people
        detected_people=len(poses_array)
        if(detected_people==0):
            return
        else:
            # rospy.loginfo("person detected")
            detected_=True

        x_center=poses_array[0].pos.x
        y_center=poses_array[0].pos.y
        # z_center=poses_array[0].position.z
        #data association

        #Let's say we are tracking som ar tags tracking chairs
        #update center of 3d objects center

        #add noises
        coin = np.random.uniform()
        if(coin >= 1.0-self.noise_probability): 
            x_noise = float(np.random.uniform(-0.15, 0.15))
            y_noise = float(np.random.uniform(-0.15, 0.15))
                # z_noise = int(np.random.uniform(-300, 300))
        else: 
            x_noise = 0
            y_noise = 0
                # z_noise = 0
        x_center += x_noise
        y_center += y_noise
        # z_center += z_noise
        
        self.Estimate_Filter()
        self.Update_Measurement_Filter(x_center,y_center)



    def listener(self,wait=0.0):
        # rospy.spin()
        while not rospy.is_shutdown():
            self.Estimate_Filter()
            rospy.Rate(3).sleep()


if __name__ == '__main__':
        rospy.init_node('Sensing Environment Node')
        manager = Environment_Manager(sys.argv[1] if len(sys.argv) >1 else 0.0)
	manager.listener()	
        

