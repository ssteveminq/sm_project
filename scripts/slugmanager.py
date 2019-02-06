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
        self.SlugState=Slug_state()

        #ros subscriber
        sm_state_topic='sm/sm_states'
        rospy.Subscriber(sm_state_topic,Slug_state,self.sm_state_callback)

        #rosaction_server
        self._feedback = Sm_StateFeedback()
        self._result = Sm_StateResult()
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, Sm_StateAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        rospy.loginfo('action_server_started:%s', self._action_name)

        self.node_num='0'

    def execute_cb(self, goal):
        # helper variables
        if self._as.is_preempt_requested():
            rospy.loginfo('%s: Preempted')
            self._as.set_preempted()

        r=rospy.Rate(1)

        success = False
        # print('save picture')
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
        self.Slug_state_to_Dictionary()

        current_node_states = nodes_dict[self.node_num]

        transition_options = [str(i) for i in self.transitions_dict[node_num]]
        self.node_num = (random.choice(transition_options))

        # Command robot to move 
        robot_commands = {}
        for c in commands:
            robot_commands[c] = nodes_dict[node_num][c]

        for key, node in self.nodes_dict.items():

            if self.nodes_dict[key] == self.cur_dictionary:
                print "selected key", key 
                self.transition_options = self.transitions_dict[key]
                print "transition_options", self.transition_options
                self.random_nextchoice =random.choice(self.transition_options) 
                print "next node", (self.random_nextchoice)
                return self.random_nextchoice
                self.policy_r_state= self.nodes_dict[key]['r_state']
                self.policy_arriving_at_0_= self.nodes_dict[key]['next_state_is_workstation']
                self.policy_next_state_is_workstation= self.nodes_dict[key]['arriving_at_0']

        
        print "Cannot find the node from states"
        exit()


    def Slug_state_to_Dictionary(self):
        #self.SlugState ==> dictionary check`
        self.cur_dictionary = {}
        self.cur_dictionary['wait'] = self.SlugState.wait
        self.cur_dictionary['obstacle2'] = self.SlugState.obstacle2
        #self.cur_dictionary['obstacle3'] = self.SlugState.obstacle3
        self.cur_dictionary['workload'] = self.SlugState.workload
        self.cur_dictionary['complete_work_at_workstation'] = self.SlugState.complete_work_at_workstation
        self.cur_dictionary['complete_dropoff_success'] = self.SlugState.complete_dropoff_success
        self.cur_dictionary['complete_dropoff_tries'] = self.SlugState.complete_dropoff_tries
        self.cur_dictionary['r_state'] = self.SlugState.r_state
        self.cur_dictionary['workload_add'] = self.SlugState.workload_add
        self.cur_dictionary['next_state_is_workstation'] = self.SlugState.next_state_is_workstation
        self.cur_dictionary['complete_work_with_robot'] = self.SlugState.complete_work_with_robot
        self.cur_dictionary['arriving_at_0'] = self.SlugState.arriving_at_0
        self.cur_dictionary['workload_stays_constant']=self.SlugState.workload_stays_constant

        print self.cur_dictionary


    def sm_state_callback(self, msg):
        # rospy.loginfo('sm_states_updated')
        self.SlugState=msg


    def simulate(self, node_init='0', num_steps=20):

        node_num = node_init

        var_list = []


        for i in range (0, num_steps):
            node = self.file_content['nodes'][node_num] 
            print( " ")
            # print(f"Node #{node_num}")
            print "Node #", node_num

            list_local, _, _ = variables_to_base10(node, self.name_and_bits)

            var_list.append(list_local) 
            transition_options =  node['trans']

            not_empty = 0
            
            node_num = str(random.choice(transition_options))
            while not_empty == 0:
                next_node = self.file_content['nodes'][node_num] 
                if next_node['trans'] == []:
                    node_num = str(random.choice(transition_options))
                else:
                    not_empty = 1

        
        # var_list is for simulation
        return var_list

    def json_to_dictionary(self):

        node_dictionary = {}
        transition_dictionary = {}

        for key, node in self.file_content['nodes'].items():
            _, dictionary_local, transition_local = variables_to_base10(node, self.name_and_bits)
            node_dictionary[str(key)] = dictionary_local
            transition_dictionary[str(key)] = transition_local

        return (node_dictionary, transition_dictionary)

    def save_dictionary_as_json(self, dictionary, filename):
        with open(filename, 'w') as f:
            json.dump(dictionary, f)

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
    # def listener(self,wait=0.0):
        # while not rospy.is_shutdown():
            # rospy.spin()

# def main(): 
    
        # print(f"start node: {node_init}, simulation length: {sim_length}")

    # path_location = os.path.dirname(os.path.realpath(__file__))
    # delivery_file = os.path.join(path_location, 'hri_reactive_synthesis', 'ctrl.json')
    # print(delivery_file)
    
    # delivery_lookup = get_lookup()
    # print(delivery_lookup)
    # delivery_sim = Controller()
    # var_list = delivery_sim.simulate(node_init, sim_length)
    # var_list = delivery_sim.simulate(str(node_init), int(sim_length))
    # (node_dictionary, transition_dictionary) = delivery_sim.json_to_dictionary()

    # node_file = os.path.join(path_location, 'hri_reactive_synthesis', 'node_dictionary.json')
    # transition_file = os.path.join(path_location, 'hri_reactive_synthesis', 'transition_dictionary.json')

    # delivery_sim.save_dictionary_as_json(node_dictionary, node_file)
    # delivery_sim.save_dictionary_as_json(transition_dictionary, transition_file)



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
 
