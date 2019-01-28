#!/usr/bin/env python

import actionlib
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Int8MultiArray
from tmc_msgs.msg import BatteryState
from hsrb_interface import Robot
from hsrb_interface import exceptions
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

	if cmd_idx==len(desired_states):
		print "index exceeds the lenght of array"
		output_state = 'end_demo'
		return output_state

	desired_state=desired_states[cmd_idx]
	handover_bool=handover[cmd_idx]
	
	if desired_state == '0' & handover_bool == 'TRUE':
		output_state = 'Go_S0_drop_off_completed_work'
	if desired_state == '0' & handover_bool == 'FALSE':
		output_state = 'Go_S0'
	elif desired_state == '1':
		output_state = 'Go_S1'
	elif desired_state == '2':
		output_state = 'Go_S2'
	elif desired_state == '3' & handover_bool == 'TRUE'::
		output_state = 'Go_S3_pick_up_completed_work'
	elif desired_state == '3' & handover_bool == 'FALSE'::
		output_state = 'Go_S3'
	# elif desired_state == '4':
	# 	output_state = 'Go_S4'
	else:  
		print "desired state out of bounds"
		output_state = 'end_demo'

	return output_state

#from cmd_idx with desired_states_array, set the goal position 
def generate_send_goal(cmd_idx):

	if cmd_idx ==-1:
		return GoalStatus.SUCCEEDED

	goal_y = 0.17
	goal_yaw = 0.0  

	cmd_state = desired_states[cmd_idx]

	if cmd_state == '0':
		goal_x = 5.0
	elif cmd_state == '1':
		goal_x = 5.5
	elif cmd_state == '2':
		goal_x = 6.0
	elif cmd_state == '3':
		goal_x = 6.2
	else:  
		goal_x = 4.5

	# fill ROS message
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
	action_state = cli.get_state()
	print 'action_state',action_state

	return action_state


def track_motion_during_duration(counter_in):
	
	State_pub=rospy.Publisher('SM/current_state', Int8MultiArray, queue_size=10)
	State_msg = Int8MultiArray()

	cmd_idx=counter_in
	original_cmd_idx=cmd_idx
	print "cmd_idx", cmd_idx

	start_time = rospy.get_time()

	print "start_time", start_time
	duration=0
        iterator=0

	while duration<10.0:

                iterator=iterator+1
		action_state = generate_send_goal(cmd_idx)
		
		curr_time =rospy.get_time()
		# print "curr_time", curr_time
		duration = curr_time - start_time
		# print duration
		
                if iterator%100000==1:
                    print "duration time: %s, action_state %s," % (duration, action_state)

		if action_state == GoalStatus.SUCCEEDED:
			cmd_idx= -1


        iterator=0
 	battery_msg = rospy.wait_for_message('/hsrb/battery_state', BatteryState)
	State_msg.data.append(battery_msg.power)
	State_msg.data.append(int(desired_states[original_cmd_idx]))
	State_pub.publish(State_msg)


	return action_state

class S_0(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['Go_S0', 'Go_S1','end_demo'],
							input_keys=['S0_counter_in','S0_counter_out'],
							output_keys=['S0_counter_out'])

	def execute(self, userdata):
		rospy.loginfo('Executing S_0')

		action_state = track_motion_during_duration(userdata.S0_counter_in)

		if action_state == GoalStatus.SUCCEEDED:
			userdata.S0_counter_out=userdata.S0_counter_in+1
			
			print "userdata.S0_counter out", userdata.S0_counter_out

			return get_action(userdata.S0_counter_out)

		else:
			"goal was not achieved"
			return 'end_demo'
			
		# if state_index == length_state_array
						# return 'end_demo'

class S_0_handover(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['Go_S0', 'Go_S1','end_demo'],
							input_keys=['S0_handover_counter_in','S0_handover_counter_out'],
							output_keys=['S0_handover_counter_out'])

	def execute(self, userdata):

		action_state = track_motion_during_duration(userdata.S0_handover_counter_in)

		if action_state == GoalStatus.SUCCEEDED:
			userdata.S0_handover_counter_out=userdata.S0_handover_counter_in+1
			
			print "userdata.S0_handover_counter out", userdata.S0_handover_counter_out

			return get_action(userdata.S0_handover_counter_out)

		else:
			"goal was not achieved"
			return 'end_demo'
			
		# if state_index == length_state_array
						# return 'end_demo'


class S_1(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['Go_S0', 'Go_S0_drop_off_completed_work' 'Go_S1', 'Go_S2', 'end_demo'],
							input_keys=['S1_counter_in','S1_counter_out'],
							output_keys=['S1_counter_out'])

	def execute(self, userdata):
		rospy.loginfo('Executing state S_1')
                tts.say("Executing State 1")
                rospy.sleep(0.5)

		action_state = track_motion_during_duration(userdata.S1_counter_in)

		if action_state == GoalStatus.SUCCEEDED:
			userdata.S1_counter_out=userdata.S1_counter_in+1
			print "userdata.S1_counter out", userdata.S1_counter_out

			return get_action(userdata.S1_counter_out)

		else:
			"goal was not achieved"
			return 'end_demo'
			 

class S_2(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['Go_S1', 'Go_S2', 'Go_S3', 'Go_S3_pick_up_completed_work', 'end_demo'],
							input_keys=['S2_counter_in','S2_counter_out'],
							output_keys=['S2_counter_out'])

	def execute(self, userdata):
		rospy.loginfo('Executing state S_2')
                tts.say("Executing State 2")

                rospy.sleep(0.5)

		action_state = track_motion_during_duration(userdata.S2_counter_in)

		if action_state == GoalStatus.SUCCEEDED:
			userdata.S2_counter_out=userdata.S2_counter_in+1
			print "userdata.S2_counter out", userdata.S2_counter_out

			return get_action(userdata.S2_counter_out)
			# return 'end_demo'
		else:
			"goal was not achieved"
			return 'end_demo'


class S_3(smach.State):
	def __init__(self):
		#  'Go_S4' deleted
		smach.State.__init__(self, outcomes=['Go_S2', 'Go_S3', 'end_demo'],
							input_keys=['S3_counter_in','S3_counter_out'],
							output_keys=['S3_counter_out'])

	def execute(self, userdata):
		rospy.loginfo('Executing state S_3')
                tts.say("Executing State 3")
                rospy.sleep(0.5)

		action_state = track_motion_during_duration(userdata.S3_counter_in)

		if action_state == GoalStatus.SUCCEEDED:
			userdata.S3_counter_out=userdata.S3_counter_in+1
			print "userdata.S3_counter out", userdata.S3_counter_out

			return get_action(userdata.S3_counter_out)
		else:
			"goal was not achieved"
			return 'end_demo'



class S_3_handover(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['Go_S2', 'Go_S3', 'end_demo'],
							input_keys=['S3_handover_counter_in','S3_handover_counter_out'],
							output_keys=['S3_handover_counter_out'])

	def execute(self, userdata):
		rospy.loginfo('Executing state S_3')
                tts.say("Executing State 3")
                rospy.sleep(0.5)

		action_state = track_motion_during_duration(userdata.S3_handover_counter_in)

		if action_state == GoalStatus.SUCCEEDED:
			userdata.S3_handover_counter_out=userdata.S3_handover_counter_in+1
			print "userdata.S3_handover_counter out", userdata.S3_handover_counter_out

			return get_action(userdata.S3_handover_counter_out)
		else:
			"goal was not achieved"
			return 'end_demo'

# class S_4(smach.State):
# 	def __init__(self):
# 		smach.State.__init__(self, outcomes=['Go_S3', 'Go_S4', 'end_demo'],
# 							input_keys=['S4_counter_in','S4_counter_out'],
# 							output_keys=['S4_counter_out'])

# 	def execute(self, userdata):
# 		rospy.loginfo('Executing state S_4')
#                 tts.say("Executing State 4")
#                 rospy.sleep(0.5)

# 		action_state = track_motion_during_duration(userdata.S4_counter_in)

# 		if action_state == GoalStatus.SUCCEEDED:
# 			userdata.S4_counter_out=userdata.S4_counter_in+1
# 			print "userdata.S4_counter out", userdata.S4_counter_out

# 			return get_action(userdata.S4_counter_out)
# 		else:
# 			"goal was not achieved"
# 			return 'end_demo'


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



# rospy.init_node('test_move')

#===========Read csv==============
f = open('../csv_parser/states_and_vals.csv', 'rt')
reader = csv.reader(f)
row_count = sum(1 for row in reader)
f.seek(0)

col_count = len(next(reader))
# print "columns"
# print col_count

f.seek(0)
data = [row for row in reader] # list comprehension 
f.close()
string_col = data[0]
desired_states = []
handover = []

for col in range(0, col_count):
	if (string_col[col] == 'robot_state'):
		robot_state_col = col 
	if (string_col[col]) == 'handover'
		handover_col = col 

for row1 in range(1,row_count):
	this_row = data[row1]
	desired_states.extend([this_row[robot_state_col]])
	handover.extend([this_row[handover_col]])

desired_states = np.array(desired_states)
desired_states.resize(desired_states.shape[0])
handover = np.array(handover)
handover.resize(handover.shape[0])

print desired_states

# exit()

#==================================

tts.say("Hello operator! I finished reading csv file")
rospy.sleep(2)

# initialize action client
cli = actionlib.SimpleActionClient('/move_base/move', MoveBaseAction)

# wait for the action server to establish connection
cli.wait_for_server()


if __name__=='__main__':
    # create SMACH state machine
    sm = smach.StateMachine(outcomes=['stop'])
    # sm.userdata.desired_states =desired_states
    sm.userdata.state_index=0

    with sm:
	smach.StateMachine.add('S_0', S_0(),
		transitions = {'Go_S0': 'S_0', 'Go_S1' : 'S_1', 'end_demo' : 'stop'},
				remapping = {'S0_counter_in':'state_index',
							 'S0_counter_out':'state_index'})
	smach.StateMachine.add('S_0_handover', S_0_handover(),
		transitions = {'Go_S0': 'S_0', 'Go_S1' : 'S_1', 'end_demo' : 'stop'},
				remapping = {'S0_handover_counter_in':'state_index',
							 'S0_handover_counter_out':'state_index'})
	smach.StateMachine.add('S_1', S_1(),
		transitions = {'Go_S0': 'S_0', 'Go_S0_drop_off_completed_work': 'S_0_handover','Go_S1':'S_1', 'Go_S2':'S_2', 'end_demo':'stop'},
				remapping = {'S1_counter_in':'state_index',
							 'S1_counter_out':'state_index'})
	smach.StateMachine.add('S_2', S_2(),
		transitions = {'Go_S1': 'S_1', 'Go_S2':'S_2', 'Go_S3':'S_3', 'Go_S3_pick_up_completed_work': 'S_3_handover', 'end_demo':'stop'},
				remapping = {'S2_counter_in':'state_index',
							 'S2_counter_out':'state_index'})
	smach.StateMachine.add('S_3', S_3(),
		transitions = {'Go_S2': 'S_2', 'Go_S3':'S_3', 'end_demo':'stop'},
				remapping = {'S3_counter_in':'state_index',
							 'S3_counter_out':'state_index'})	
	smach.StateMachine.add('S_3_handover', S_3_handover(),
		transitions = {'Go_S2': 'S_2', 'Go_S3':'S_3', 'end_demo':'stop'},
				remapping = {'S3_handover_counter_in':'state_index',
							 'S3_handover_counter_out':'state_index'})	
	# smach.StateMachine.add('S_3', S_3(),
	# 	transitions = {'Go_S2': 'S_2', 'Go_S3':'S_3', 'Go_S4':'S_4', 'end_demo':'stop'},
	# 			remapping = {'S3_counter_in':'state_index',
	# 						 'S3_counter_out':'state_index'})	
	# smach.StateMachine.add('S_4', S_4(),
	# 	transitions = {'Go_S3':'S_3', 'Go_S4':'S_4', 'end_demo':'stop'},
	# 			remapping = {'S4_counter_in':'state_index',
	# 						 'S4_counter_out':'state_index'})	

    outcome = sm.execute()


# if __name__ == '__main__':
  # main()
