#!/usr/bin/env python
from __future__ import print_function
import rospy
# Brings in the SimpleActionClient
import actionlib
# Brings in the messages used by the fibonacci action, including the
from sm_project.msg import Sm_StateAction, Sm_StateGoal

def controller_action_client():
	# Creates the SimpleActionClient, passing the type of the action
	client = actionlib.SimpleActionClient('slug_controller', Sm_StateAction)
	# Waits until the action server has started up and started
	# listening for goals.
		# rospy.sleep(1.0)
	# print("wait for server")
	client.wait_for_server()
	# print("wait for server")
	# Creates a goal to send to the action server.
	goal = Sm_StateGoal(start=True)
	# Sends the goal to the action server.
	client.send_goal(goal)
	# Waits for the server to finish performing the action.
	client.wait_for_result()
		
	rospy.loginfo("Start action")

	# Prints out the result of executing the action
	return client.get_result()  # A FibonacciResult

if __name__ == '__main__':
	try:
			# Initializes a rospy node so that the SimpleActionClient can
			# publish and subscribe over ROS.
			rospy.init_node('slug_controller_client')
			result = controller_action_client()
			print(result)
			# print("Result:", ', '.join([str(n) for n in result.sequence]))
	except rospy.ROSInterruptException:
		print("program interrupted before completion", file=sys.stderr)
