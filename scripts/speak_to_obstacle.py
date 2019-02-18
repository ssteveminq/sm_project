#!/usr/bin/env python
import actionlib
from sm_project.msg import Sm_StateAction, Sm_StateGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Int8MultiArray
from tmc_msgs.msg import BatteryState

from hsrb_interface import Robot
from hsrb_interface import exceptions
import roslib
import rospy
import smach
import smach_ros
from smach import State
import tf.transformations

import sm_project.msg
from sm_project.msg import Slug_state
from sm_project.msg import Sm_StateFeedback,Sm_StateResult, Sm_StateAction
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PoseStamped


class Talk_to_Obstacle():

	def __init__(self):
		print "subscribers initialized"
		obstacle2_topic="/obstacle2_Is_Occupied"
		rospy.Subscriber(obstacle2_topic, Bool, self.obstacle2_Callback)
		obstacle3_topic="/obstacle3_Is_Occupied"
		rospy.Subscriber(obstacle3_topic, Bool, self.obstacle3_Callback)

		self.Initial_time = rospy.get_time()
		self.curr_time = self.Initial_time
		self.dur = 0

	def obstacle2_Callback(self,msg):
		self.curr_time = rospy.get_time()
		self.dur = int(self.curr_time-self.Initial_time)
		print "self.dur", self.dur
		if msg.data==True and self.dur > 10.0:
			self.Initial_time = self.curr_time
			tts.say("Could you please not block this work area?")
			rospy.sleep(0.1)			
		else:
			pass 

	def obstacle3_Callback(self,msg):
		self.curr_time = rospy.get_time()
		self.dur = int(self.curr_time-self.Initial_time)
		print "self.dur", self.dur
		if msg.data==True and self.dur > 10.0:
			self.Initial_time = self.curr_time
			tts.say("Could you please not block this work area?")
			rospy.sleep(0.1)			
		else:
			pass 


tts=whole_body = None

while not rospy.is_shutdown():
	try:
		robot = Robot()
		tts = robot.try_get('default_tts')
		# delay caused below
		whole_body = robot.try_get('whole_body')
		tts.language = tts.ENGLISH
		print "slug manager initialized"
		break
	except (exceptions.ResourceNotFoundError,
		   exceptions.RobotConnectionError) as e:
		rospy.logerr("Failed to obtain resource: {}\nRetrying...".format(e))


if __name__ == '__main__':

	obstacle_communicator= Talk_to_Obstacle()
	rospy.spin()
