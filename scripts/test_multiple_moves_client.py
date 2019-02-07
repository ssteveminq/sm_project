#!/usr/bin/env python
from __future__ import print_function
import rospy
import sys
# Brings in the SimpleActionClient
import actionlib
# Brings in the messages used by the fibonacci action, including the
from sm_project.msg import Sm_StateAction, Sm_StateGoal
from std_msgs.msg import Int32MultiArray


class Test_Transitions(object):
	def __init__(self, wait=0.0):

		self.client = actionlib.SimpleActionClient('slug_controller', Sm_StateAction)
		# self.states_pub=rospy.Publisher("/sm/sm_states",Sm_states,queue_size=50)
		self.states_pub=rospy.Publisher("SM/current_state",Int32MultiArray,queue_size=50)
		self.fake_sm=Int32MultiArray()
		self.Initial_time = rospy.get_time()



	def publish_fake_SM_state(self):
		cur_time = rospy.get_time()
		self.fake_sm.data.append(int(cur_time))
		self.states_pub.publish(self.fake_sm)
		self.client.wait_for_server()
		# print("wait for server")
		# Creates a goal to send to the action server.
		goal = Sm_StateGoal(start=True)
		# Sends the goal to the action server.
		self.client.send_goal(goal)
		# Waits for the server to finish performing the action.
		self.client.wait_for_result()
			
		rospy.loginfo("Start action")


	def listener(self,wait=0.0):
		# rospy.spin()
		while not rospy.is_shutdown():
			self.publish_fake_SM_state()
			rospy.Rate(0.14).sleep()


if __name__ == '__main__':
	rospy.init_node('test_steps')
	manager = Test_Transitions(sys.argv[1] if len(sys.argv) >1 else 0.0)
	manager.listener()  

