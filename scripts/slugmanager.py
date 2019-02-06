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

# def clean(js):
#     """Eliminate all terminal nodes and return."""

#     while True:

#             # accumulate list of terminal nodes and remove them from `js`
#         terminal_nodes = []
#         num_nodes = len(js['nodes'])


#         for key, node in js['nodes'].items():
#             if not node['trans']:
#                 terminal_nodes.append(int(key))

#         # remove references to terminal nodes
#         for key, node in js['nodes'].items():
#             node['trans'] = [t for t in node['trans'] if t not in terminal_nodes]

#         for t in terminal_nodes:
#             del js['nodes'][str(t)]

		
#         if not terminal_nodes:
#             return js

# def variables_to_base10(node, name_and_bits):

#     state_binary = node['state']
#     count = 0 

#     list_local = []
#     variable_dictionary = {}


#     for j in name_and_bits:

#         # j is equal to a dictionary
#         name = j['name']
#         bits = j['bits']

#         # create array slice from an array of numbers
#         bin_string = state_binary[count:count+bits] # grab the section 
#         bin_string = bin_string[::-1] # reverse the section
#         # make an array of strings 
#         bin_string = [str(str_var) for str_var in bin_string] 
#         # make into string
#         bin_string = ''.join(bin_string)

#         val_base_10 = int(bin_string,2)

#         # print(f"{name}:{val_base_10}")

		 

#         list_local.append(val_base_10)
		
#         variable_dictionary[name] = val_base_10
#         transitions = node['trans']
#         count+=bits 


#     return list_local, variable_dictionary, transitions 



class Controller():

	def __init__(self, name,wait=0.0):

		# 'with' is context block
		# 'r' is read mode
		# 'open' is a file handle and is being stored as f 
		# with open(file_name, 'r') as f :
			# self.file_content = json.load(f)
			# self.file_content = clean(self.file_content)

		self.file_name=''
		self.trans_name=''
		self.path_location=''

		self.num_nodes = 0 
		self.loadjsonfiles()
		self.name_and_bits = self.get_lookup()

		# self.states_pub=rospy.Publisher("/sm/sm_states",Slug_state,queue_size=50)
		self.SlugState = Slug_state()
		self.Initial_time = rospy.get_time()

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
		rospy.Subscriber(task_sm_topic,Int32MultiArray,self.Task_SM_Callback)

		# initial workload
		self.previous_workload=17
		self.SlugState.workload=16
		self.policy_workload_add_previous=0


		#rosaction_server
		self._feedback = Sm_StateFeedback()
		self._result = Sm_StateResult()
		self._action_name = name
		self._as = actionlib.SimpleActionServer(self._action_name, Sm_StateAction, execute_cb=self.execute_cb, auto_start = False)
		self._as.start()
		rospy.loginfo('action_server_started:%s', self._action_name)

		self.node_num='0'

	def execute_cb(self, goal):
		print "execute_cb"
		# helper variables
		if self._as.is_preempt_requested():
			rospy.loginfo('%s: Preempted')
			self._as.set_preempted()

		r=rospy.Rate(1)

		success = False
		# print('save picture')

		curr_time=rospy.get_time()
		duration = int(curr_time -self.Initial_time)
		self.calculate_statesvariables(duration)

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
		
		# environment = ['wait', 'obstacle2', 'obstacle3', 'workload', 'complete_work_at_workstation', 'complete_dropoff_success', 'complete_dropoff_tries', 'workload_stays_constant']
		environment = ['wait', 'obstacle2', 'workload', 'complete_work_at_workstation', 'complete_dropoff_success', 'complete_dropoff_tries', 'workload_stays_constant']

		self.Slug_state_to_Dictionary()

		environment_states = {}
		for states in environment:
			environment_states[states] = self.cur_dictionary[states]

		self.transition_options = [str(i) for i in self.transitions_dict[self.node_num]]

		success = False 
		while success == False: 

			for node_options in self.transition_options:
					
				if all(self.nodes_dict[node_options][key] == environment_states[key] for key in environment):
					self.node_num = node_options
					# print "SELECTED OPTION", node_num
					success = True 

			print "no match"
			match = False  
			if success == False: 
				for key, node in self.nodes_dict.items():
					if self.nodes_dict[key] == self.cur_dictionary:
						print "NO MATCH"
						      
						print "selected key", key 
						
						self.node_num = key 

						success = True 

						match = True 

					if match == True:
						break 

		# where to go next
		self.transition_options = [str(i) for i in self.transitions_dict[self.node_num]]
		self.node_num=(random.choice(self.transition_options)) 

		# command robot state 
		print "next state", self.nodes_dict[self.node_num]['r_state']
		return self.nodes_dict[self.node_num]['r_state']


	def Slug_state_to_Dictionary(self):

		print "Slug_state_to_Dictionary"
		#self.SlugState ==> dictionary check`

		self.cur_dictionary = {}
		self.cur_dictionary['wait'] = self.SlugState.wait
		self.cur_dictionary['obstacle2'] = self.SlugState.obstacle2
		#self.cur_dictionary['obstacle3'] = self.SlugState.obstacle3
		self.cur_dictionary['workload'] = self.SlugState.workload
		self.cur_dictionary['complete_work_at_workstation'] = self.SlugState.complete_work_at_workstation
		self.cur_dictionary['complete_dropoff_success'] = self.SlugState.complete_dropoff_success
		self.cur_dictionary['complete_dropoff_tries'] = self.SlugState.complete_dropoff_tries

		# todo: check r_state properly
		self.cur_dictionary['r_state'] = self.SlugState.r_state

		self.cur_dictionary['workload_add'] = self.policy_workload_add
		self.cur_dictionary['next_state_is_workstation'] = self.policy_next_state_is_workstation
		self.cur_dictionary['complete_work_with_robot'] = self.policy_complete_work_with_robot

		if self.policy_complete_work_with_robot != self.SlugState.complete_work_with_robot:
			print "work with robot tracking error"
			exit()

		self.cur_dictionary['arriving_at_0'] = self.policy_arriving_at_0
		self.cur_dictionary['workload_stays_constant']=self.SlugState.workload_stays_constant

		print self.cur_dictionary


	def sm_state_callback(self, msg):
		# rospy.loginfo('sm_states_updated')
		self.SlugState=msg


	# def simulate(self, node_init='0', num_steps=20):

	# 	node_num = node_init

	# 	var_list = []


	# 	for i in range (0, num_steps):
	# 		node = self.file_content['nodes'][node_num] 
	# 		print( " ")
	# 		# print(f"Node #{node_num}")
	# 		print "Node #", node_num

	# 		list_local, _, _ = variables_to_base10(node, self.name_and_bits)

	# 		var_list.append(list_local) 
	# 		transition_options =  node['trans']

	# 		not_empty = 0
			
	# 		node_num = str(random.choice(transition_options))
	# 		while not_empty == 0:
	# 			next_node = self.file_content['nodes'][node_num] 
	# 			if next_node['trans'] == []:
	# 				node_num = str(random.choice(transition_options))
	# 			else:
	# 				not_empty = 1

		
	# 	# var_list is for simulation
	# 	return var_list

	# def json_to_dictionary(self):

	# 	node_dictionary = {}
	# 	transition_dictionary = {}

	# 	for key, node in self.file_content['nodes'].items():
	# 		_, dictionary_local, transition_local = variables_to_base10(node, self.name_and_bits)
	# 		node_dictionary[str(key)] = dictionary_local
	# 		transition_dictionary[str(key)] = transition_local

	# 	return (node_dictionary, transition_dictionary)

	# def save_dictionary_as_json(self, dictionary, filename):
	# 	with open(filename, 'w') as f:
	# 		json.dump(dictionary, f)

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
			#{'name': 'obstacle3', 'bits': 1},
			{'name': 'workload', 'bits': 5},
			{'name': 'complete_work_at_workstation', 'bits': 1},
			{'name': 'complete_dropoff_success', 'bits': 1},
			{'name': 'complete_dropoff_tries', 'bits': 2},
			{'name': 'r_state', 'bits': 3},
			{'name': 'workload_add', 'bits': 4},
			{'name': 'next_state_is_workstation', 'bits': 1},
			{'name': 'complete_work_with_robot', 'bits': 1},
			{'name': 'workload_stays_constant','bits': 1},
			{'name': 'arriving_at_0', 'bits': 1},
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
			self.SlugState.obstacle2=1
		else:
			self.SlugState.obstacle2=0
		# msg.data

	def obstacle3_Callback(self,msg):
		# rospy.loginfo('obstacle3_callback')
		# self.SlugState.obstacle3=int(msg.data)
		if msg.data==True:
			self.SlugState.obstacle3=1
		else:
			self.SlugState.obstacle3=0

	def Task_SM_Callback(self,msg):
		# self.Initial_time = msg.data[0]
		self.Initial_time = rospy.get_time()


	def calculate_statesvariables(self, time_duration):
		print "calculate_statesvariables"
		# rospy.loginfo('duration %d',int(time_duration))

		self.policy_r_state= self.nodes_dict[self.node_num]['r_state']
		self.policy_workload_add= self.nodes_dict[self.node_num]['workload_add']
		self.policy_next_state_is_workstation= self.nodes_dict[self.node_num]['next_state_is_workstation']
		self.policy_complete_work_with_robot=self.nodes_dict[self.node_num]['complete_work_with_robot']
		self.policy_arriving_at_0= self.nodes_dict[self.node_num]['arriving_at_0']

		# todo: sense the robot state
		self.SlugState.r_state=self.policy_r_state

		print "self.SlugState.workload", self.SlugState.workload
		print "time_duration",time_duration
		print "int(time_duration%10)", int(time_duration/10)

		#calculate workload
		if self.SlugState.r_state != 4:
			self.SlugState.workload=self.SlugState.workload-int(time_duration/10)
		else: 
			self.SlugState.workload=self.SlugState.workload+self.policy_workload_add_previous

		print "self.SlugState.workload", self.SlugState.workload

		# check if workload changes
		if self.previous_workload == self.SlugState.workload:
			self.SlugState.workload_stays_constant = 1
		else: 
			self.SlugState.workload_stays_constant = 0


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

		self.policy_workload_add_previous=self.policy_workload_add
		self.previous_workload=self.SlugState.workload


if __name__ == '__main__':

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
 
