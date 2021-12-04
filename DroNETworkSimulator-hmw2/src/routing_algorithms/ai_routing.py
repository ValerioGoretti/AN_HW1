import numpy as np
from src.utilities import utilities as util
from src.routing_algorithms.BASE_routing import BASE_routing
from matplotlib import pyplot as plt
from src.utilities import config
import random
import time
class AIRouting(BASE_routing):
    
    def __init__(self, drone, simulator):
        BASE_routing.__init__(self, drone, simulator)
        # random generator
        self.packet_set=set()
        self.taken_actions = {}  #id event : (old_action)
        self.drone_path=[]
        self.counter=0
        self.set_collision=set({})
        self.set_waypoint=set({})
        self.completed_lap=False
        self.packet_dictio={}
        self.packet_set=set()
        self.packet_generation=[]
        self.first_waypoint=None
        self.number_lap=0
        self.actions_rewards={}
        self.actions_set=set()
        self.qTable_dictionary={}
        self.actions_timestamp={}
        self.old_choices={}


        # q dictionary
        self.q_dict = {None : [0, 0]}

        # discount factor
        self.gamma = 0.8

        # learning rate
        self.alpha = 0.3

        # list to maintain (state, action) that need an update yet
        self.state_action_list = []

        # dict to maintain the packets that i have when i make a decision
        self.state_action_packets = {}

        # dict to maintain the time that the drone would take to go and return from the depot if he came back at that time
        self.final_time_to_depot = 0

        # pck buffer
        self.pkt_buffer = []

        # last choiche
        self.last_choice_index = None



    def feedback(self, drone, id_event, delay, outcome):
        """ return a possible feedback, if the destination drone has received the packet """
        if config.DEBUG:
            print("Drone: ", self.drone.identifier, "---------- has delivered: ", self.taken_actions)
            print("Drone: ", self.drone.identifier, "---------- just received a feedback:",
                  "Drone:", drone, " - id-event:", id_event, " - delay:",  delay, " - outcome:", outcome)




        # if the last choice was made by AI and the buffer length is still larger than 0,
        # then update Q table for all past actions without reward
        if drone.buffer_length() != 0 and self.last_choice_index == 1:

            # cancel last choice made
            self.last_choice_index = None
            # set the state for final state
            future_state = None
            # packets delivered when the drone moved directly to the depot
            delivered_packets = set([hash(x) for x in drone.all_packets()])
            # empty the buffer of packets
            drone.empty_buffer()
            print("############################## CONSEGNA ###############################")
            print("ID DRONE: ", drone.identifier)
            print(delivered_packets)
            for state, action_index in reversed(self.state_action_list):
                # packets that the drone had in this state
                pk_state = set(self.state_action_packets[state])
                # delivered packets among that the drone had
                pk_state_delivered = pk_state.intersection(delivered_packets)


                # get the number of packets delivered among those the drone had
                state_action_delivered_packets_num = len( set(self.state_action_packets[state]) & delivered_packets )

                # percentage of delivered packets
                delivered_packets_percent = (state_action_delivered_packets_num * 100) / len(set(self.state_action_packets[state]))
                #print(delivered_packets_percent)

                # get the max time the drone would take if it had at the farthest point from depot
                max_time = self.get_max_time_to_depot_and_return(drone)

                # reward
                reward = delivered_packets_percent - self.alpha * (self.final_time_to_depot / max_time)

                # Q update formula
                self.q_dict[state][action_index] = self.q_dict[state][action_index] + self.alpha * (reward + self.gamma * max(self.q_dict[future_state]) - self.q_dict[state][action_index])
                # set the next future state
                future_state = state



            # set the final time to 0
            self.final_time_to_depot = 0

            # empty state action
            self.state_action_list = []

            # empty state action packets dict
            self.state_action_packets = {}



        #if outcome == -1:
        #    print("Packet Expired: ", id_event)

        if id_event in self.packet_set and drone == self.drone:
            self.packet_generation = [x for x in self.packet_generation if x[0].event_ref.identifier != id_event]
            self.packet_generation = sorted(self.packet_generation,key=lambda x: x[1])

        if id_event in self.taken_actions:
            if outcome == 0:
                self.packet_dictio[id_event]=True



    def relay_selection(self, opt_neighbors, pkd):
        """ arg min score  -> geographical approach, take the drone closest to the depot """
        cell_index = util.TraversedCells.coord_to_cell(size_cell=self.simulator.prob_size_cell,
                                                        width_area=self.simulator.env_width,
                                                        x_pos=self.drone.coords[0],  # e.g. 1500
                                                        y_pos=self.drone.coords[1])[0]  # e.g. 500

        if pkd.event_ref.identifier not in self.packet_set:
            self.packet_set.add(pkd.event_ref.identifier)
            self.packet_generation.append((pkd,pkd.time_step_creation))
            self.packet_generation=sorted(self.packet_generation, key=lambda x: x[1])


        if self.is_packet_expiring(self.packet_generation[0][0]) and self.last_choice_index != 1 and self.drone.identifier == 0:

            # check if the drone has already taken an action in this (state, action) sequence for the current state (cell)
            if cell_index not in [x[0] for x in self.state_action_list]:
                if cell_index not in self.q_dict.keys():
                    # make a random action (action_index => index of the action in Q_table, 0 for None, 1 for -1)
                    action_index = random.choice([0, 1])
                    # initialize the dict with the new state
                    self.q_dict[cell_index] = [0, 0]

                # if the state already exists choice the best action in q_dict
                else:
                    is_random_choice = random.choices([True, False], weights=(50, 50), k=1)[0]
                    if is_random_choice or self.q_dict[cell_index][0] == self.q_dict[cell_index][1]:
                        action_index = random.choice([0, 1])
                    else:
                        action_index = self.q_dict[cell_index].index(max(self.q_dict[cell_index]))

                # add new (state, action) to be updated
                self.state_action_list.append((cell_index, action_index))

                # store the packets in the buffer when i take an action
                self.state_action_packets[cell_index] = [hash(x) for x in self.drone.all_packets()]

                print("CELLA :", cell_index, " --> ", self.state_action_packets[cell_index])

                if action_index == 1:
                    # store the time needed to go and return to depot from this point
                    self.final_time_to_depot = self.time_to_depot_and_return(self.drone)
                    self.last_choice_index = 1
                    return -1
                else:
                    self.last_choice_index = 0
                    return None

        if self.is_time_to_goback():
            return -1

        '''if self.first_waypoint==None:
            self.first_waypoint=self.drone.next_target()
        globalhistory=self.drone.waypoint_history
        localHistory = []
        for point in reversed(globalhistory):
            localHistory.insert(0,point)
            self.set_waypoint.add(point)
            if point[0] == globalhistory[0][0] and point[1] == globalhistory[0][1]:
                if len(localHistory)<len(self.set_waypoint):
                    self.completed_lap=True
                break
        if pkd.event_ref.identifier not in self.packet_dictio:
            self.packet_dictio[pkd.event_ref.identifier]=False
        if self.drone_path == [] and self.drone.waypoint_history != []:
            if self.drone.waypoint_history[self.drone.current_waypoint - 1][1] < self.drone.waypoint_history[self.drone.current_waypoint - 2][1]:
                self.drone_path = self.drone.waypoint_history.copy()
        # Se path è ricominciato vai al depot
        elif self.drone.next_target() in self.drone_path:
            self.counter+=1
            if self.drone_path.index(self.drone.next_target()) == 0 and self.drone.buffer_length() >= 3:
                return -1'''

        #CASO COLLISIONE----------------------------------------------------------------------------
        return None # here you should return a drone object!


    def get_max_time_to_depot_and_return(self, drone):
        return ((util.euclidean_distance(self.simulator.depot.coords, (0, 1500))) / drone.speed) * 2

    def time_to_depot_and_return(self, drone):
        return (util.euclidean_distance(self.simulator.depot.coords, drone.coords) / drone.speed) * 2

    def arrival_time(self, drone):
        tot=(util.euclidean_distance(drone.next_target(), drone.coords) / drone.speed)+(util.euclidean_distance(drone.next_target(), self.simulator.depot.coords)/drone.speed)
        return tot 
    def is_time_to_goback(self):
        time_expected=self.arrival_time(self.drone)
        end_expected=self.simulator.len_simulation*self.simulator.time_step_duration-(self.simulator.cur_step*self.simulator.time_step_duration)
        return time_expected>end_expected
    def is_packet_expiring(self,pkd):
        time_left=8000*self.simulator.time_step_duration-(self.simulator.cur_step*self.simulator.time_step_duration-pkd.time_step_creation*self.simulator.time_step_duration)
        expected_time=self.arrival_time(self.drone)
        return expected_time>time_left
    def lap_counter(self,globalHistory):
        if globalHistory==[]:
            return 1
        count_list=[x for x in globalHistory if x==globalHistory[0]]
        return len(count_list)
    def perform_random_action(self,opt_neighbors,cell,pkd):
        opt2=[x[1] for x in opt_neighbors]
        randomChoice=self.untaken_drone(opt2,pkd)
        return randomChoice
        #for collision in [x for x in ]
    def untaken_drone(self,opt_neighbors,pkd):
        while True:
            drone=self.simulator.rnd_routing.choice(opt_neighbors)
            if drone.identifier not in pkd.hops:
                return drone 
    def drone_not_seen(self,opt_neighbors,pkd):
        for collision in opt_neighbors:
            if collision.identifier not in pkd.hops:
                return True
        return False
    def make_choice(self,choice,cell_index,pkd):
        if choice:
            self.taken_actions[pkd.event_ref.identifier][len(self.taken_actions[pkd.event_ref.identifier])-1]=((cell_index,True),self.simulator.cur_step)
        else:
            self.taken_actions[pkd.event_ref.identifier][len(self.taken_actions[pkd.event_ref.identifier])-1]=((cell_index,False),self.simulator.cur_step)
        #print(self.taken_actions[pkd.event_ref.identifier][len(self.taken_actions[pkd.event_ref.identifier])-1])
       # self.taken_actions[pkd.event_ref.identifier].append((cell_index,choice))
    def already_chosen_check(self,cell,pkd):
        for action in self.taken_actions[pkd.event_ref.identifier]:
            if action[0][0]==cell and action[0][1]:
                return True
        return False
    def initialize_state_action(self,pkd,cell):
        if pkd.event_ref.identifier not in self.taken_actions:
            self.taken_actions[pkd.event_ref.identifier]=[]
        for action in self.taken_actions[pkd.event_ref.identifier]:
            if action[0][0]==cell:
                return False
        self.taken_actions[pkd.event_ref.identifier].append(((cell,False),self.simulator.cur_step))
        return True
    def print_past_history(self):
        if self.drone.identifier==1:
            for k in self.taken_actions.keys():
                print(self.taken_actions[k])

    def print(self):
        pass