#!/usr/bin/env python
import json 
import random 
import os
import sys
import roslib
import actionlib
import rospy
import sm_project.msg
from sm_project.msg import Slug_state
from sm_project.msg import Sm_StateFeedback,Sm_StateResult, Sm_StateAction
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import PoseStamped
from hsrb_interface import Robot


class Controller():

	def __init__(self, name,wait=0.0):

		self.file_name=''
		self.trans_name=''
		self.path_location=''

		self.num_nodes = 0 
		self.loadjsonfiles()
		self.name_and_bits = self.get_lookup()

		# self.states_pub=rospy.Publisher("/sm/sm_states",Slug_state,queue_size=50)
		self.SlugState = Slug_state()
		

		#ros subscriber
		# sm_state_topic='sm/sm_states'
		# rospy.Subscriber(sm_state_topic,Slug_state,self.sm_state_callback)

		obstacle2_topic="/obstacle2_Is_Occupied"
		rospy.Subscriber(obstacle2_topic, Bool, self.obstacle2_Callback)
		obstacle3_topic="/obstacle3_Is_Occupied"
		rospy.Subscriber(obstacle3_topic, Bool, self.obstacle3_Callback)
		robot_pose_topic="global_pose"
		rospy.Subscriber(robot_pose_topic, PoseStamped, self.pose_callback)
		task_sm_topic='SM/current_state'
		#rospy.Subscriber(task_sm_topic,Int32MultiArray,self.Task_SM_Callback)


		# initial workload
		self.previous_workload=13
		self.SlugState.workload=12
		self.policy_workload_add_previous=0
		self.first_move=True  
		self.remainder=0


		#rosaction_server
		self._feedback = Sm_StateFeedback()
		self._result = Sm_StateResult()
		self._action_name = name
		self._as = actionlib.SimpleActionServer(self._action_name, Sm_StateAction, execute_cb=self.execute_cb, auto_start = False)
		self._as.start()
		rospy.loginfo('action_server_started:%s', self._action_name)


		self.node_num='0'
		self.prev_node_num=self.node_num

		self.next_state=None

	def execute_cb(self, goal):
		print "execute_cb"
		# helper variables
		if self._as.is_preempt_requested():
			rospy.loginfo('%s: Preempted')
			self._as.set_preempted()

		r=rospy.Rate(1)

		success = False
		# print('save picture')

		if self.first_move==True : 
			self.Initial_time = rospy.get_time()

		curr_time=rospy.get_time()
		duration = int(curr_time -self.Initial_time)
		self.calculate_statesvariables(duration)
		self.Initial_time = curr_time

		rospy.loginfo('action_server_executed') 
		self._result.policy= self.get_policy()
		self._transitions=self.transition_options
		self._feedback.is_feasible= True
		rospy.loginfo('action_server_executed') 
		rospy.loginfo('policy:%d', self._result.policy) 
		self._as.set_succeeded(self._result)
		self._as.publish_feedback(self._feedback)

		r.sleep()
		# rospy.loginfo('policy:%d , self._result.policy') 

	def get_policy(self):
		print "get_policy"
		
		environment = ['wait', 'obstacle2', 'obstacle3', 'workload', 'complete_work_at_workstation', 'complete_dropoff_success', 'complete_dropoff_tries', 'workload_stays_constant']

		controllable = ['r_state', 'complete_work_with_robot']

		#, 'workload_add', 'next_state_is_workstation', 'arriving_at_0'

		self.Slug_state_to_Dictionary()

		environment_states = {}
		for states in environment:
			environment_states[states] = self.cur_dictionary[states]

		print "environment", environment_states

		controllable_states = {}
		for states in controllable:
			controllable_states[states] = self.cur_dictionary[states]

		print "controllable", controllable_states

		self.transition_options = [str(i) for i in self.transitions_dict[self.prev_node_num]]

		print "transition options"
		print self.transition_options

		if self.first_move==False: 

			success = False 
			while success == False: 

				for node_options in self.transition_options:					
					if all(self.nodes_dict[node_options][key] == environment_states[key] for key in environment):
						self.node_num = node_options
						success = True 
						print "this node_num ", self.node_num 

				match = False  
				if success == False: 
		
					for key, node in self.nodes_dict.items():
						# there is a bug here ?? need to index properly
						for key1, node1 in node.items():
							if all(self.nodes_dict[key][key3] == environment_states[key3] for key3 in environment):

								if all(self.nodes_dict[key][key3] == controllable_states[key3] for key3 in controllable):
								
								# if self.nodes_dict[key] == self.cur_dictionary:
									print "NO MATCH"
									      
									print "selected key", key 

									
									self.node_num = key 

									success = True 

									match = True 

								if match == True:
									break 


		else: 
			self.first_move = False  

		self.policy_workload_add_previous=self.nodes_dict[self.node_num]['workload_add']

		# keep track of where robot was
		self.prev_node_num=self.node_num
		# where to go next
		self.transition_options = [str(i) for i in self.transitions_dict[self.node_num]]
		print "next transition_options", self.transition_options
		self.node_num=(random.choice(self.transition_options)) 
		print "next node_num ", self.node_num 

		self.next_state=self.nodes_dict[self.node_num]['r_state']

		# command robot state 
		print "next state", self.next_state
		return self.nodes_dict[self.node_num]['r_state']


	def Slug_state_to_Dictionary(self):

		print "Slug_state_to_Dictionary"

		self.cur_dictionary = {}
		self.cur_dictionary['wait'] = self.SlugState.wait
		
		if self.first_move==True:
			self.cur_dictionary['obstacle2'] = self.nodes_dict[self.node_num]['obstacle2']
			self.cur_dictionary['obstacle3'] = self.nodes_dict[self.node_num]['obstacle3']

		else: 
			if self.next_state == 0 or self.next_state == 1:
				self.cur_dictionary['obstacle2'] = self.SlugState.obstacle_right 
				self.cur_dictionary['obstacle3'] = 0
			elif self.next_state == 2:
				self.cur_dictionary['obstacle2'] = 0
				self.cur_dictionary['obstacle3'] = self.SlugState.obstacle_right 				
			elif self.next_state == 3:
				self.cur_dictionary['obstacle2'] = self.SlugState.obstacle_left
				self.cur_dictionary['obstacle3'] = 0
			elif self.next_state == 4:
				self.cur_dictionary['obstacle2'] = 0 
				self.cur_dictionary['obstacle3'] = self.SlugState.obstacle_left

		self.cur_dictionary['workload'] = self.SlugState.workload
		self.cur_dictionary['complete_work_at_workstation'] = self.SlugState.complete_work_at_workstation

		# TODO: tries and success - sense
		self.cur_dictionary['complete_dropoff_success'] = self.nodes_dict[self.node_num]['complete_dropoff_success']
		self.cur_dictionary['complete_dropoff_tries'] = self.nodes_dict[self.node_num]['complete_dropoff_tries']

		# todo: check r_state properly DO WE NEED TO DO THIS?
		# self.cur_dictionary['r_state'] = self.SlugState.r_state
		self.cur_dictionary['r_state'] = self.nodes_dict[self.node_num]['r_state']


		if self.nodes_dict[self.node_num]['next_state_is_workstation'] >= 12:
			self.cur_dictionary['next_state_is_workstation'] = 0
			self.cur_dictionary['workload_add'] = 0
		else: 
			self.cur_dictionary['next_state_is_workstation'] = self.nodes_dict[self.node_num]['next_state_is_workstation']
			self.cur_dictionary['workload_add'] = self.nodes_dict[self.node_num]['workload_add']	
		self.cur_dictionary['complete_work_with_robot'] = self.nodes_dict[self.node_num]['complete_work_with_robot']

		if self.policy_complete_work_with_robot != self.SlugState.complete_work_with_robot:
			print "work with robot tracking error"
			exit()

		self.cur_dictionary['next_arriving_at_0'] = self.nodes_dict[self.node_num]['next_arriving_at_0']
		self.cur_dictionary['workload_stays_constant']=self.SlugState.workload_stays_constant


	def sm_state_callback(self, msg):
		# rospy.loginfo('sm_states_updated')
		self.SlugState=msg


	def loadjsonfiles(self):

		self.path_location = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
		self.file_name = os.path.join(self.path_location, 'config', 'node_dictionary.json')
		self.trans_file_name = os.path.join(self.path_location, 'config', 'transition_dictionary.json')

		with open(self.file_name, 'r') as f :
			self.nodes_dict=json.load(f)

		with open(self.trans_file_name, 'r') as f :
			self.transitions_dict=json.load(f)

		# self.file_name = os.path.join(self.path_location, 'config', 'ctrl.json')
		# print(f"path_location:{self.path_location}")
		# 'with' is context block
		# 'r' is read mode
		# 'open' is a file handle and is being stored as f 

		# with open(self.file_name, 'r') as f :
		#     self.file_content = json.load(f)
		#     self.file_content = clean(self.file_content)

		#self.num_nodes = len(self.file_content['nodes'])


	def get_lookup(self):


		lookup = [
			{'name': 'wait', 'bits': 1},
			{'name': 'obstacle2', 'bits': 1},
			{'name': 'obstacle3', 'bits': 1},
			{'name': 'workload', 'bits': 5},
			{'name': 'complete_work_at_workstation', 'bits': 1},
			{'name': 'complete_dropoff_success', 'bits': 1},
			{'name': 'complete_dropoff_tries', 'bits': 2},
			{'name': 'r_state', 'bits': 3},
			{'name': 'workload_add', 'bits': 4},
			{'name': 'next_state_is_workstation', 'bits': 1},
			{'name': 'complete_work_with_robot', 'bits': 1},
			{'name': 'workload_stays_constant','bits': 1},
			{'name': 'next_arriving_at_0', 'bits': 1},
			]

		return lookup



	def publish_slug_state(self):

		print "publish_slug_state"
	  
		self.states_pub.publish(self.SlugState)



	def pose_callback(self,msg):
		# rospy.loginfo('global_pose_callback')
		robot_pos = msg.pose
		#Todo: change w.r.t robot poses
		if robot_pos.position.y <0.2:
			self.SlugState.r_state = 1
		elif robot_pos.position.y <-0.49:
			self.SlugState.r_state = 2
		elif robot_pos.position.y <-0.99:
			self.SlugState.r_state = 3
		elif robot_pos.position.y <-2.5: 
			self.SlugState.r_state = 4
		
	def obstacle2_Callback(self,msg):
		# rospy.loginfo('obstacle2_callback')
		# self.SlugState.obstacle2=int(msg.data)
		if msg.data==True:
			self.SlugState.obstacle_left=1
		else:
			self.SlugState.obstacle_left=0
		# msg.data

	def obstacle3_Callback(self,msg):
		# rospy.loginfo('obstacle3_callback')
		# self.SlugState.obstacle3=int(msg.data)
		if msg.data==True:
			self.SlugState.obstacle_right=1
		else:
			self.SlugState.obstacle_right=0

	#def Task_SM_Callback(self,msg):
		# self.Initial_time = msg.data[0]
		#self.Initial_time = rospy.get_time()


	def calculate_statesvariables(self, time_duration):
		print "calculate_statesvariables"
		# rospy.loginfo('duration %d',int(time_duration))

		self.policy_r_state= self.nodes_dict[self.node_num]['r_state']
		self.policy_workload_add= self.nodes_dict[self.node_num]['workload_add']
		self.policy_next_state_is_workstation= self.nodes_dict[self.node_num]['next_state_is_workstation']
		self.policy_complete_work_with_robot=self.nodes_dict[self.node_num]['complete_work_with_robot']
		self.policy_arriving_at_0= self.nodes_dict[self.node_num]['next_arriving_at_0']

		# todo: sense the robot state
		self.SlugState.r_state=self.policy_r_state

		print "time_duration",int(time_duration)
		print "remainder", self.remainder
		print "int(subtraction_amount)", int( (time_duration+self.remainder) / 10)


		print "self.SlugState.workload", self.SlugState.workload

		#calculate workload
		if self.SlugState.r_state != 4:
			self.SlugState.workload=self.SlugState.workload-int( (time_duration+self.remainder) / 10)
			self.remainder = int( (time_duration+self.remainder) % 10)
		else: 
			self.SlugState.workload=self.SlugState.workload+self.policy_workload_add_previous
			self.remainder=0

		print "self.SlugState.workload", self.SlugState.workload

		# check if workload changes
		if self.previous_workload == self.SlugState.workload:
			self.SlugState.workload_stays_constant = 1
		else: 
			self.SlugState.workload_stays_constant = 0

		if self.SlugState.workload < self.previous_workload and self.SlugState.r_state != 4:
			self.SlugState.complete_work_at_workstation = 1
		elif self.SlugState.complete_work_at_workstation == 1 and self.SlugState.r_state != 4:
			self.SlugState.complete_work_at_workstation = 1
		else: 
			self.SlugState.complete_work_at_workstation = 0


		# calculate if waiting
		if self.SlugState.workload > 0:
			self.SlugState.wait=0
		else: 
			self.SlugState.wait=1

		# todo: complete dropoff tries
		self.SlugState.complete_dropoff_tries=0


		# todo: complete dropoff success
		self.SlugState.complete_dropoff_success = 0


		if self.SlugState.r_state == 4:
			self.SlugState.complete_work_with_robot=1
		elif self.SlugState.complete_work_with_robot==1 and self.SlugState.r_state != 1:
			self.SlugState.complete_work_with_robot=1
		else: 
			self.SlugState.complete_work_with_robot=0
	
		
		if self.SlugState.complete_work_with_robot==1 and self.SlugState.r_state == 1:
			self.SlugState.r_state=0

		self.previous_workload=self.SlugState.workload




# tts=whole_body = None

# while not rospy.is_shutdown():
# 	try:
# 		robot = Robot()
# 		tts = robot.try_get('default_tts')
# 		whole_body = robot.try_get('whole_body')
# 		tts.language = tts.ENGLISH
# 		print "slug manager initialized"
# 		break
# 	except (exceptions.ResourceNotFoundError,
# 		   exceptions.RobotConnectionError) as e:
# 		rospy.logerr("Failed to obtain resource: {}\nRetrying...".format(e))

if __name__ == '__main__':

	# if I use robot, comment this out
	rospy.init_node('slug_action')

	if len(sys.argv)>1:
		node_init=sys.argv[1]
		sim_length=int(sys.argv[2])
	else:
		node_init ='0'
		sim_length=int(10)

	slug_controller= Controller("slug_controller")
	# rospy.spin()
	
	# var_list = delivery_sim.simulate(node_init, sim_length)
 
