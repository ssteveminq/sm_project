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


def get_action(cmd_idx):

    if cmd_idx==len(desired_staes):
        print "index exceeds the lenght of array"
        output_state = 'end_demo'
        return output_state

    desired_state=desired_states[cmd_idx]
    
    if desired_state == '0':
        output_state = 'Go_S0'
    elif desired_state == '1':
        output_state = 'Go_S1'
    elif desired_state == '2':
        output_state = 'Go_S2'
    elif desired_state == '3':
        output_state = 'Go_S3'
    elif desired_state == '4':
        output_state = 'Go_S4'
    else:  
        print "desired state out of bounds"
        output_state = 'end_demo'

    return output_state

#from cmd_idx with desired_states_array, set the goal position 
def generate_send_goal(cmd_idx):

       if cmd_idx ==-1:
           return GoalStatus.SUCCEEDED

	goal_y = 0.4
	goal_yaw = 0.0	

        cmd_state = desired_staes[cmd_idx]

	if cmd_state == '0':
		goal_x = 4.0
	elif cmd_state == '1':
		goal_x = 3.5
	elif cmd_state == '2':
		goal_x = 3.0
	elif cmd_state == '3':
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



class S_0(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes = ['Go_S0', 'GO_S1' , 'end_demo'])
                                    input_keys=['S0_counter_in'],
                                    output_keys=['S0_counter_out'])


	def execute(self, userdata):
		rospy.loginfo('Moving to state 0')
                
                cmd_idx=userdata.S0_counter_in

                start_time = rospy.Time()
                duration=0

                while duration<10.0:

                    action_state = generate_send_goal(cmd_idx)

                    curr_time =rospy.Time()
                    duration = curr_time - start_time
                    print "duration time: ", duration + "action_state", action_state

                    if action_state == GoalStatus.SUCCEEDED:
                        cmd_idx= -1


                if action_state == GoalStatus.SUCCEEDED:
                    userdata.S0_counter_out=userdata.S0_counter_in+1
                    return get_action(userdata.S0_counter_out)
                else
                    print action_state

                    return 'end_demo'
                    
                # if state_index == length_state_array
                                # return 'end_demo'




class S_1(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes = ['Go_S0','Go_S1', 'GO_S2', 'end_demo'],
                                           input_keys=['S1_counter_in'],
                                           output_keys=['S1_counter_out'])

        def execute(self, userdata):
            rospy.loginfo('Executing state S_1')
            rospy.loginfo('Counter = %f'%userdata.S1_counter_in)
            return 'end_demo'
     

class S_2(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes = ['Go_S0', 'GO_S2', 'end_demo'])


class S_3(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes = ['left', 'right', 'stay', 'end_demo'])


class POS_4(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes = ['left', 'right', 'stay', 'end_demo'])



# def main():


rospy.init_node('test_move')


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

for col in range(0, col_count):
	if (string_col[col] == 'robot_state'):
		robot_state_col = col 

for row1 in range(1,row_count):
	this_row = data[row1]
	desired_states.extend([this_row[robot_state_col]])

desired_states = np.array(desired_states)
desired_states.resize(desired_states.shape[0])

print desired_states

#==================================

# initialize action client
cli = actionlib.SimpleActionClient('/move_base/move', MoveBaseAction)

# wait for the action server to establish connection
cli.wait_for_server()

# create SMACH state machine
sm = smach.StateMachine(outcomes=['stop'])
# sm.userdata.desired_states =desired_states
sm.userdata.state_index=0

with sm:
	smach.StateMachine.add('S_0', S_0(),
		transitions = {'Go_S0': 'S_0', 'Go_S1' : 'S_1', 'end_demo' : 'stop'},
                remapping = {'S0_counter_in':'state_index',
                             'S0_counter_out':'state_index'})
	smach.StateMachine.add('S_1', S_1(),
		transitions = {'Go_S0': 'S_0', 'Go_S1':'S_1', 'Go_S2':'S_2', 'end_demo':'stop'},
                remapping = {'S1_counter_in':'state_index',
                             'S1_counter_out':'state_index'})



outcome = sm.execute()


# if __name__ == '__main__':
# 	main()
