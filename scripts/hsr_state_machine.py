#!/usr/bin/env python

import actionlib
import actionlib
from sm_project.msg import Sm_StateAction, Sm_StateGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Int8MultiArray
from tmc_msgs.msg import BatteryState
from hsrb_interface import Robot
from hsrb_interface import exceptions
from villa_manipulation.msg import *
# import roslib
import rospy
import smach
import smach_ros
from smach import State
import tf.transformations

import csv
import numpy as np
import time



def get_action(cmd_idx):

	# if cmd_idx==len(desired_states):
	if cmd_idx==limit_moves: 
		print "end of demo"
		desired_state = 0 
		output_state = 'end_demo'
		return desired_state, output_state

	desired_state=get_policy()
	
	if desired_state == 0:
		output_state = 'Go_S0'
	elif desired_state == 1:
		output_state = 'Go_S1'
	elif desired_state == 2:
		output_state = 'Go_S2'
	elif desired_state == 3:
		output_state = 'Go_S3'
	elif desired_state == 4:
		output_state = 'Go_S4'
	else:  
		print "desired state out of bounds"
		output_state = 'end_demo'


	return desired_state, output_state


def get_policy():


	#call slug action server to get policy
	slug_goal = Sm_StateGoal(start=True)
	# Sends the goal to the action server.
	slug_cli.send_goal(slug_goal)
	# Waits for the server to finish performing the action.
	slug_cli.wait_for_result()
	slug_result = slug_cli.get_result()    
	cmd_state = slug_result.policy

	print "cmd_state", cmd_state


	return cmd_state
def navigation_action(goal_x,goal_y,goal_yaw):
	pose = PoseStamped()
	pose.header.stamp = rospy.Time.now()
	pose.header.frame_id = "map"
	pose.pose.position = Point(goal_x, goal_y, 0)
	quat = tf.transformations.quaternion_from_euler(0, 0, goal_yaw)
	pose.pose.orientation = Quaternion(*quat)

	goal = MoveBaseGoal()
	goal.target_pose = pose

	# send message to the action server
	cli.send_goal(goal)

	# wait for the action server to complete the order
	cli.wait_for_result()

	# print result of navigation
	result_action_state = cli.get_state()

	return result_action_state 

def givepose_action():
	goal = villa_manipulation.msg.HandoverGoal()
	givepose_client.send_goal(goal)
	givepose_client.wait_for_result()
	result_action_state = givepose_client.get_state()
	return result_action_state 


def receivepose_action():
	goal = villa_manipulation.msg.HandoverGoal()
	receivepose_client.send_goal(goal)
	receivepose_client.wait_for_result()
	result_action_state = receivepose_client.get_state()
	return result_action_state 

def putdown_action():
    goal = villa_manipulation.msg.ForcePutDownGoal()
    goal.place_pose =PoseStamped()
    goal.place_pose.pose.position.x=0.57
    goal.place_pose.pose.position.y=-1.3
    goal.place_pose.pose.position.z=0.70
    goal.place_pose.header.frame_id='map'
 
    put_down_client.send_goal(goal)
    put_down_client.wait_for_result()
    result_action_state = put_down_client.get_state()
    return result_action_state 
	
def generate_send_goal(cmd_idx, cmd_state, prev_state):

	if cmd_idx ==-1:
		return GoalStatus.SUCCEEDED

	goal_x = -0.0
	goal_y = -0.0
	goal_yaw = 0.0  
		
	Move_Base = True

	# cmd_state = desired_states[cmd_idx]

	if cmd_state == 0:
			goal_y = -0.0
			move_action_state=navigation_action(goal_x,goal_y,goal_yaw)
			if move_action_state==GoalStatus.SUCCEEDED:
			   g_action_state=givepose_action()
			   r_action_state=g_action_state
			if prev_state==0:
			   Move_Base = False
	elif cmd_state == 1:
			goal_y = -0.0
	elif cmd_state == 2:
			goal_y = -0.5
	elif cmd_state == 3:
			goal_y = -1.0
	elif cmd_state == 4:
			
			goal_y = -2.0

			#move base + receiveepose
			move_action_state=navigation_action(goal_x,goal_y,goal_yaw)
			if move_action_state==GoalStatus.SUCCEEDED and prev_state!=4:
				receive_action_state=receivepose_action()
			#the case should wait? 
			if prev_state==4:
				receive_action_state=GoalStatus.SUCCEEDED
				 
	else:  
			goal_y = -2.0

	if Move_Base:
		r_action_state=navigation_action(goal_x,goal_y,goal_yaw)
		# print 'action_state',action_state
	
		# fill ROS message
		# pose = PoseStamped()
		# pose.header.stamp = rospy.Time.now()
		# pose.header.frame_id = "map"
		# pose.pose.position = Point(goal_x, goal_y, 0)
		# quat = tf.transformations.quaternion_from_euler(0, 0, goal_yaw)
		# pose.pose.orientation = Quaternion(*quat)

		# goal = MoveBaseGoal()
		# goal.target_pose = pose

	# send message to the action server
		# cli.send_goal(goal)

	# wait for the action server to complete the order
		# cli.wait_for_result()

	# print result of navigation
		# action_state = cli.get_state()
	# else:
		#handover or putdown

# print 'action_state',action_state
	return r_action_state


def track_motion_during_duration(counter_in, cmd_state, prev_state):
	
	State_pub=rospy.Publisher('SM/current_state', Int8MultiArray, queue_size=10)
	State_msg = Int8MultiArray()

	cmd_idx=counter_in
	
	print "cmd_idx", cmd_idx

	start_time = rospy.get_time()

	print "start_time", start_time
	duration=0
	iterator=0

	print "cmd_state", cmd_state
	print "prev_state", prev_state

	if cmd_state == 3:
		max_dur = 10.0
	elif cmd_state == prev_state:
		max_dur = 10.0
	else:
		max_dur = 5.0

	print "max_dur", max_dur


	while (duration<max_dur or cmd_idx != -1):

		iterator=iterator+1
		action_state = generate_send_goal(cmd_idx, cmd_state, prev_state)
		
		curr_time =rospy.get_time()
		# print "curr_time", curr_time
		duration = curr_time - start_time
		# print duration
		
		# if iterator%10000000==1:
			# print "duration time: %s, action_state %s," % (duration, action_state)

		if action_state == GoalStatus.SUCCEEDED:
			"duration time: %s, action_state %s," % (duration, action_state)
			cmd_idx= -1


		iterator=0

	battery_msg = rospy.wait_for_message('/hsrb/battery_state', BatteryState)
	finished_time = rospy.get_time()
	State_msg.data.append(finished_time)
	State_msg.data.append(battery_msg.power)
	State_msg.data.append(cmd_state)
	State_pub.publish(State_msg)


	return action_state

class S_0(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['Go_S0', 'Go_S1','end_demo'],
							input_keys=['S0_counter_in','S0_counter_out','S0_desired_state_in','S0_desired_state_out','S0_previous_state_in','S0_previous_state_out'],
							output_keys=['S0_counter_out','S0_desired_state_out','S0_previous_state_out'])

	def execute(self, userdata):
		rospy.loginfo('Executing S_0')

		tts.say("Executing State 0")
		action_state = track_motion_during_duration(userdata.S0_counter_in, userdata.S0_desired_state_in, userdata.S0_previous_state_in)

		if action_state == GoalStatus.SUCCEEDED:
			userdata.S0_counter_out=userdata.S0_counter_in+1
			
			print "userdata.S0_counter out", userdata.S0_counter_out

			userdata.S0_previous_state_out = userdata.S0_desired_state_in

			userdata.S0_desired_state_out, output_state = get_action(userdata.S0_counter_out)
			#previous_action=0

			return output_state

		else:
			"goal was not achieved"
			return 'end_demo'
			
		# if state_index == length_state_array
						# return 'end_demo'



class S_1(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['Go_S1', 'Go_S2', 'end_demo'],
							input_keys=['S1_counter_in','S1_counter_out','S1_desired_state_in','S1_desired_state_out','S1_previous_state_in','S1_previous_state_out'],
							output_keys=['S1_counter_out','S1_desired_state_out','S1_previous_state_out' ])

	def execute(self, userdata):
		rospy.loginfo('Executing state S_1')
		# tts.say("Executing State 1")
		rospy.sleep(0.5)

		action_state = track_motion_during_duration(userdata.S1_counter_in, userdata.S1_desired_state_in, userdata.S1_previous_state_in)

		if action_state == GoalStatus.SUCCEEDED:
			userdata.S1_counter_out=userdata.S1_counter_in+1
			print "userdata.S1_counter out", userdata.S1_counter_out

			userdata.S1_previous_state_out = userdata.S1_desired_state_in

			userdata.S1_desired_state_out, output_state = get_action(userdata.S1_counter_out)
			#previous_action=1

			return output_state

		else:
			"goal was not achieved"
			return 'end_demo'
			 

class S_2(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['Go_S0', 'Go_S1', 'Go_S2', 'Go_S3', 'end_demo'],
							input_keys=['S2_counter_in','S2_counter_out','S2_desired_state_in','S2_desired_state_out','S2_previous_state_in','S2_previous_state_out'],
							output_keys=['S2_counter_out','S2_desired_state_out','S2_previous_state_out' ])

	def execute(self, userdata):
		rospy.loginfo('Executing state S_2')
		#tts.say("Executing State 2")

		rospy.sleep(0.5)


		action_state = track_motion_during_duration(userdata.S2_counter_in, userdata.S2_desired_state_in, userdata.S2_previous_state_in)

		if action_state == GoalStatus.SUCCEEDED:
			userdata.S2_counter_out=userdata.S2_counter_in+1
			print "userdata.S2_counter out", userdata.S2_counter_out

			userdata.S2_previous_state_out = userdata.S2_desired_state_in

			userdata.S2_desired_state_out, output_state = get_action(userdata.S2_counter_out)

			#previous_action=2
			return output_state

		else:
			"goal was not achieved"
			return 'end_demo'


class S_3(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['Go_S2', 'Go_S3', 'Go_S4', 'end_demo'],
							input_keys=['S3_counter_in','S3_counter_out','S3_desired_state_in','S3_desired_state_out','S3_previous_state_in','S3_previous_state_out'],
							output_keys=['S3_counter_out','S3_desired_state_out','S3_previous_state_out' ])

	def execute(self, userdata):
		rospy.loginfo('Executing state S_3')
		# tts.say("Executing State 3")
		rospy.sleep(0.5)

		action_state = track_motion_during_duration(userdata.S3_counter_in, userdata.S3_desired_state_in, userdata.S3_previous_state_in)

		if action_state == GoalStatus.SUCCEEDED:
			userdata.S3_counter_out=userdata.S3_counter_in+1
			print "userdata.S3_counter out", userdata.S3_counter_out

			userdata.S3_previous_state_out = userdata.S3_desired_state_in

			#previous_action=3
			userdata.S3_desired_state_out, output_state = get_action(userdata.S3_counter_out)

			return output_state

		else:
			"goal was not achieved"
			return 'end_demo'


class S_4(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['Go_S3', 'Go_S4', 'end_demo'],
							input_keys=['S4_counter_in','S4_counter_out','S4_desired_state_in','S4_desired_state_out','S4_previous_state_in','S4_previous_state_out'],
							output_keys=['S4_counter_out','S4_desired_state_out','S4_previous_state_out'])

	def execute(self, userdata):
		rospy.loginfo('Executing state S_4')
		# tts.say("Executing State 4")
		rospy.sleep(0.5)

		action_state = track_motion_during_duration(userdata.S4_counter_in, userdata.S4_desired_state_in, userdata.S4_previous_state_in)

		if action_state == GoalStatus.SUCCEEDED:
			userdata.S4_counter_out=userdata.S4_counter_in+1
			print "userdata.S4_counter out", userdata.S4_counter_out

			userdata.S4_previous_state_out = userdata.S4_desired_state_in
			userdata.S4_desired_state_out, output_state = get_action(userdata.S4_counter_out)

			#previous_action=4
			return output_state
		else:
			"goal was not achieved"
			return 'end_demo'




tts=whole_body = None

while not rospy.is_shutdown():
	try:
		robot = Robot()
		tts = robot.try_get('default_tts')
		whole_body = robot.try_get('whole_body')
		tts.language = tts.ENGLISH
		break
	except (exceptions.ResourceNotFoundError,
		   exceptions.RobotConnectionError) as e:
		rospy.logerr("Failed to obtain resource: {}\nRetrying...".format(e))

limit_moves = 100

# tts.say("Hello! I'm ready'")
rospy.sleep(1)

# initialize action client

cli = actionlib.SimpleActionClient('/move_base/move', MoveBaseAction)
slug_cli = actionlib.SimpleActionClient('slug_controller', Sm_StateAction)
givepose_client = actionlib.SimpleActionClient('givepose_action',villa_manipulation.msg.HandoverAction)
receivepose_client= actionlib.SimpleActionClient('receivepose_action',villa_manipulation.msg.HandoverAction)
put_down_client = actionlib.SimpleActionClient('putdown_action',villa_manipulation.msg.ForcePutDownAction)
 

# wait for the action server to establish connection
cli.wait_for_server()
slug_cli.wait_for_server()
givepose_client.wait_for_server()
receivepose_client.wait_for_server()
put_down_client.wait_for_server()


if __name__=='__main__':
	# create SMACH state machine
	sm = smach.StateMachine(outcomes=['stop'])
	# sm.userdata.desired_states =desired_states
	sm.userdata.state_index=0
	sm.userdata.current_desired_state=get_policy()
	sm.userdata.previous_desired_state=sm.userdata.current_desired_state
	#previous_action=sm.userdata.current_desired_state

	with sm:
		smach.StateMachine.add('S_0', S_0(),
			transitions = {'Go_S0': 'S_0', 'Go_S1' : 'S_1', 'end_demo' : 'stop'},
					remapping = {'S0_counter_in':'state_index',
								 'S0_counter_out':'state_index',
								 'S0_desired_state_in' : 'current_desired_state',
								 'S0_desired_state_out' : 'current_desired_state',
								 'S0_previous_state_in' : 'previous_desired_state',
								 'S0_previous_state_out' : 'previous_desired_state'})
		smach.StateMachine.add('S_1', S_1(),
			transitions = {'Go_S1':'S_1', 'Go_S2':'S_2', 'end_demo':'stop'},
					remapping = {'S1_counter_in':'state_index',
								 'S1_counter_out':'state_index',
								 'S1_desired_state_in' : 'current_desired_state',
								 'S1_desired_state_out' : 'current_desired_state',
								 'S1_previous_state_in' : 'previous_desired_state',
								 'S1_previous_state_out' : 'previous_desired_state'})
		smach.StateMachine.add('S_2', S_2(),
			transitions = {'Go_S0': 'S_0', 'Go_S1': 'S_1', 'Go_S2':'S_2', 'Go_S3':'S_3', 'end_demo':'stop'},
					remapping = {'S2_counter_in':'state_index',
								 'S2_counter_out':'state_index',
								 'S2_desired_state_in' : 'current_desired_state',
								 'S2_desired_state_out' : 'current_desired_state',
								 'S2_previous_state_in' : 'previous_desired_state',
								 'S2_previous_state_out' : 'previous_desired_state'})
		smach.StateMachine.add('S_3', S_3(),
			transitions = {'Go_S2': 'S_2', 'Go_S3':'S_3', 'Go_S4':'S_4', 'end_demo':'stop'},
					remapping = {'S3_counter_in':'state_index',
								 'S3_counter_out':'state_index',
								 'S3_desired_state_in' : 'current_desired_state',
								 'S3_desired_state_out' : 'current_desired_state',
								 'S3_previous_state_in' : 'previous_desired_state',
								 'S3_previous_state_out' : 'previous_desired_state'})	
		smach.StateMachine.add('S_4', S_4(),
			transitions = {'Go_S3':'S_3', 'Go_S4':'S_4', 'end_demo':'stop'},
					remapping = {'S4_counter_in':'state_index',
								 'S4_counter_out':'state_index',
								 'S4_desired_state_in' : 'current_desired_state',
								 'S4_desired_state_out' : 'current_desired_state',
								 'S4_previous_state_in' : 'previous_desired_state',
								 'S4_previous_state_out' : 'previous_desired_state'})	

	if sm.userdata.current_desired_state == 0:
		sm.set_initial_state(['S_0'])
	elif sm.userdata.current_desired_state == 1:
		sm.set_initial_state(['S_1'])
	elif sm.userdata.current_desired_state == 2:
		sm.set_initial_state(['S_2'])
	elif sm.userdata.current_desired_state == 3:
		sm.set_initial_state(['S_3'])
	else: 
		sm.set_initial_state(['S_4'])

	
	outcome = sm.execute()


# if __name__ == '__main__':
  # main()
