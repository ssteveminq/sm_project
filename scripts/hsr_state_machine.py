import actionlib
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import roslib
import rospy
import smach
import smach_ros
from smach import State
import tf.transformations


def generate_commanded_state(userdata):

	goal_y = 0.4
	goal_yaw = 0.0	

	if userdata == '0':
		goal_x = 4.0
	elif userdata == '1':
		goal_x = 3.5
	elif userdata == '2':
		goal_x = 3.0
	elif userdata == '3':
		goal_x = 2.5
	else:  
		goal_x = 2.0

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


	return action_state



class POS_0(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes = ['left', 'right', 'stay', 'end_demo'])


	def execute(self, userdata):
		rospy.loginfo('Moving to state 0')


		action_state = generate_commanded_state(goal_x, goal_y, goal_yaw)

		if (less than 10 s have passed):
			if action_state == GoalStatus.SUCCEEDED:
				return 'stay'
			else:
				return 'left'
		# 10 seconds have passed
		else: 
			state_index+=1
			if state_index == length_state_array
				return 'end_demo'
			else:
				goal_state = state_array[state_index]
				if goal_state == '0'
					return 'stay'
				else:
					return 'right'


class POS_1(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes = ['left', 'right', 'stay', 'end_demo'])


class POS_2(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes = ['left', 'right', 'stay', 'end_demo'])


class POS_3(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes = ['left', 'right', 'stay', 'end_demo'])


class POS_4(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes = ['left', 'right', 'stay', 'end_demo'])



# def main():


rospy.init_node('test_move')


#===========Read csv==============

state_array = []
state_index = 0

#==================================


# initialize action client
cli = actionlib.SimpleActionClient('/move_base/move', MoveBaseAction)

# wait for the action server to establish connection
cli.wait_for_server()

# create SMACH state machine
sm = smach.StateMachine(outcomes=['stop'])

with sm:
	smach.StateMachine.add('POS_0', POS_0(),
		transitions = {'left': 'POS_0', 'right' : 'POS_1', 'stay': 'POS_0', 'end_demo' : 'stop'})

outcome = sm.execute()


# if __name__ == '__main__':
# 	main()