#!/usr/bin/env python

import actionlib
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import rospy
import tf.transformations

import csv
import numpy as np

################# csv parser ################

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
desired_position = []

for col in range(0, col_count):
	if (string_col[col] == 'robot_state'):
		robot_state_col = col 

for row1 in range(1,row_count):
	this_row = data[row1]
	desired_position.extend([this_row[robot_state_col]])

desired_position = np.array(desired_position)
desired_position.resize(desired_position.shape[0])

print desired_position
##############################################



rospy.init_node('test_move')

# initialize action client
cli = actionlib.SimpleActionClient('/move_base/move', MoveBaseAction)

# wait for the action server to establish connection
cli.wait_for_server()

#inputcmd=[0,1,2,3,4,4,4,3,2,1,0,0,1,2,2,3,4,3,3]

# for i in desired_position:
    # navigoal_pose=get_pose(i);



# input goal pose
goal_x = 5.8
goal_y = 4.8
goal_yaw = 0.0

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
#cli.send_goal(goal)

# wait for the action server to complete the order
#cli.wait_for_result()

# print result of navigation
action_state = cli.get_state()
if action_state == GoalStatus.SUCCEEDED:
        rospy.loginfo("Navigation Succeeded.")
