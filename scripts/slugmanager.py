import json 
import random 
import os
import sys

def clean(js):
    """Eliminate all terminal nodes and return."""

    while True:

            # accumulate list of terminal nodes and remove them from `js`
        terminal_nodes = []
        num_nodes = len(js['nodes'])



        for key, node in js['nodes'].items():
            if not node['trans']:
                terminal_nodes.append(int(key))

        # remove references to terminal nodes
        for key, node in js['nodes'].items():
            node['trans'] = [t for t in node['trans'] if t not in terminal_nodes]

        for t in terminal_nodes:
            del js['nodes'][str(t)]

        
        if not terminal_nodes:
            return js

def variables_to_base10(node, name_and_bits):

    state_binary = node['state']
    count = 0 

    list_local = []
    variable_dictionary = {}


    for j in name_and_bits:

        # j is equal to a dictionary
        name = j['name']
        bits = j['bits']

        # create array slice from an array of numbers
        bin_string = state_binary[count:count+bits] # grab the section 
        bin_string = bin_string[::-1] # reverse the section
        # make an array of strings 
        bin_string = [str(str_var) for str_var in bin_string] 
        # make into string
        bin_string = ''.join(bin_string)

        val_base_10 = int(bin_string,2)

        # print(f"{name}:{val_base_10}")

         

        list_local.append(val_base_10)
        
        variable_dictionary[name] = val_base_10
        transitions = node['trans']
        count+=bits 


    return list_local, variable_dictionary, transitions 



class Controller():

    def __init__(self):
        # 'with' is context block
        # 'r' is read mode
        # 'open' is a file handle and is being stored as f 
        # with open(file_name, 'r') as f :
            # self.file_content = json.load(f)
            # self.file_content = clean(self.file_content)

        self.file_name=''
        self.path_location=''

        self.num_nodes = 0 
        self.loadjsonfiles()
        self.name_and_bits = self.get_lookup()


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
        self.file_name = os.path.join(self.path_location, 'config', 'ctrl.json')
        # print(f"path_location:{self.path_location}")
        # 'with' is context block
        # 'r' is read mode
        # 'open' is a file handle and is being stored as f 
        with open(self.file_name, 'r') as f :
            self.file_content = json.load(f)
            self.file_content = clean(self.file_content)

        self.num_nodes = len(self.file_content['nodes'])


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
            {'name': 'arriving_at_0', 'bits': 1},
            ]

        return lookup


def main(): 
    
    if len(sys.argv)>1:
        node_init=sys.argv[1]
        sim_length=int(sys.argv[2])
    else:
        node_init ='0'
        sim_length=int(10)

        # print(f"start node: {node_init}, simulation length: {sim_length}")

    # path_location = os.path.dirname(os.path.realpath(__file__))
    # delivery_file = os.path.join(path_location, 'hri_reactive_synthesis', 'ctrl.json')
    # print(delivery_file)
    
    # delivery_lookup = get_lookup()
    # print(delivery_lookup)
    delivery_sim = Controller()
    var_list = delivery_sim.simulate(node_init, sim_length)
    # var_list = delivery_sim.simulate(str(node_init), int(sim_length))
    # (node_dictionary, transition_dictionary) = delivery_sim.json_to_dictionary()

    # node_file = os.path.join(path_location, 'hri_reactive_synthesis', 'node_dictionary.json')
    # transition_file = os.path.join(path_location, 'hri_reactive_synthesis', 'transition_dictionary.json')

    # delivery_sim.save_dictionary_as_json(node_dictionary, node_file)
    # delivery_sim.save_dictionary_as_json(transition_dictionary, transition_file)



    

if __name__ == '__main__':
    main()




    # # list of dictionaries
    # park_lookup = [
    #   {'name': 'o_state', 'bits': 4},
    #   {'name': 'park', 'bits': 1},
    #   {'name': 'a_state', 'bits': 4}
    # ]

    # park_file = '/home/formal/cyberphysical_systems/hw5/prob1/ctrl.json'
    # park_sim = Controller(park_lookup, park_file)
    # node_init = '0'
    # park_sim.simulate(node_init, 50)

    # list of dictionaries
    # elevator_lookup = [
    #   {'name': 'Call1', 'bits': 3},
    #   # {'name': 'Call2', 'bits': 3},
    #   {'name': 'Open', 'bits': 3},
    #   {'name': 'Floor', 'bits': 2}
    # ]

    # elevator_lookup = [
    #   {'name': 'Req0', 'bits': 1},
    #   {'name': 'Req1', 'bits': 1},
    #   {'name': 'Req2', 'bits': 1},
    #   {'name': 'Req3', 'bits': 1},
    #   {'name': 'Num_Req', 'bits': 2},
    #   # {'name': 'Not_Req3', 'bits': 3},      
    #   {'name': 'Open0', 'bits': 1},
    #   {'name': 'Open1', 'bits': 1},
    #   {'name': 'Open2', 'bits': 1},
    #   {'name': 'Open3', 'bits': 1},
    #   {'name': 'Floor', 'bits': 2}
    # ]


    # elevator_file = '/home/formal/cyberphysical_systems/hw5/prob3/elevator_deadlock.json'
    # elevator_sim = Controller(elevator_lookup, elevator_file)
    # node_init = '0'
    # elevator_sim.simulate(node_init, 2)


    # intersection_lookup = [
    #   {'name': 'Start3', 'bits': 4},
    #   {'name': 'Start4', 'bits': 4},
    #   {'name': 'Start7', 'bits': 4},
    #   {'name': 'Clear_o', 'bits': 1},
    #   {'name': 'Init_Wait3', 'bits': 1},
    #   {'name': 'Init_Wait4', 'bits': 1},
    #   {'name': 'Init_Wait7', 'bits': 1},
    #   {'name': 'C_state', 'bits': 4},
    #   {'name': 'Clear_c', 'bits': 1}
    # ]

    # intersection_file = '/home/formal/cyberphysical_systems/hw5/prob2/ctrl_intersection.json'
    # intersection_sim = Controller(intersection_lookup, intersection_file)
    # node_init = '10'
    # intersection_sim.simulate(node_init, 10)



    
    # dreamer_lookup = [
    #   {'name': 'h_state', 'bits': 2},
    #   {'name': 'h_closed', 'bits': 1},
    #   {'name': 'r_state', 'bits': 2},
    #   {'name': 'r_closed', 'bits': 1}
    # ]

    # dreamer_file = '/home/formal/cyberphysical_systems/hw5/two_player/ctrl.json'
    # dreamer_sim = Controller(dreamer_lookup, dreamer_file)
    # node_init = '0'
    # var_list = dreamer_sim.simulate(node_init, 10)

    # print var_list
