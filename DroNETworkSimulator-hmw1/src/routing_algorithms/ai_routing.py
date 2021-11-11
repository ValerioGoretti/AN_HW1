import numpy as np
import hashlib
import time
from src.utilities import utilities as util
from src.routing_algorithms.BASE_routing import BASE_routing
import  random
import statistics
from matplotlib import pyplot as plt
class AIRouting(BASE_routing):
    def __init__(self, drone, simulator):
        BASE_routing.__init__(self, drone, simulator)
        # random generator
        self.rnd_for_routing_ai = np.random.RandomState(self.simulator.seed)
        # Dictionary that associate to each packet a list of actions regarding the drone
        # Example: {id_packet:[(cell,choice,hash(history),collision_drone),(cell,choice,hash(history),collision_drone))]}
        self.taken_actions = {}  
        # Dictionary that associate to each action a list of rewards
        #Example : {(cell,choice,hash(history),collision_drone):[R1,R2,R3...]}
        self.actions_rewards={}
        #Set of all the actions
        #Example: {(cell,choice,hash(history),collision_drone),(cell,drone,(hash(history),collision_drone)...)}
        self.actions_set=set({})
        #Probability of random choice
        self.epsilon=10
        #Dictionary containing the timestamp of each action. If an action is taken more than one time
        #the last timestamp is recorded
        self.actions_timestamp={}
    def feedback(self, drone, id_event, delay, outcome):
        """ return a possible feedback, if the destination drone has received the packet """
        # Packets that we delivered and still need a feedback
        #----------------------print(self.drone.identifier, "----------", self.taken_actions)
        # outcome == -1 if the packet/event expired; 0 if the packets has been delivered to the depot
        # Feedback from a delivered or expired packet
        # Be aware, due to network errors we can give the same event to multiple drones and receive multiple feedback for the same packet!!
        # NOTE: reward or update using the old action!!
        # STORE WHICH ACTION DID YOU TAKE IN THE PAST.
        actual_time=time.time()
        if id_event in self.taken_actions.keys():
            action_list=self.taken_actions[id_event]
           # if outcome==1:
            #    print("DELIVERED PACKET: "+str(id_event)+"__")
            #else:
            #    print("EXPIRED PACKET: "+str(id_event)+"__")
            for action in action_list:
                action_delay=actual_time-self.actions_timestamp[action]
                if outcome==-1:
                    #Update function to be implemented
                    self.actions_rewards[action].append(-4)
            #        print("Action: ",action,", Setted reward",self.actions_rewards[action],", Delay action: ",actual_time-self.actions_timestamp[action],", Delay from generation: ",delay)
                else:
                    #Update function to be implemented
                    self.actions_rewards[action].append((-1)*(action_delay)) 
             #       print("Action: ",action,", Setted reward",self.actions_rewards[action],", Delay action: ",actual_time-self.actions_timestamp[action],"Delay from generation: ",delay)
    
    def relay_selection(self, opt_neighbors, pkd):
        """ arg min score  -> geographical approach, take the drone closest to the depot """
        # Only if you need --> several features:
        #Cell index of the position of the drone
        cell_index = util.TraversedCells.coord_to_cell(size_cell=self.simulator.prob_size_cell,
                                                      width_area=self.simulator.env_width,
                                                       x_pos=self.drone.coords[0],  # e.g. 1500
                                                        y_pos=self.drone.coords[1])[0]  # e.g. 500
        #Total set of waypoint
        globalhistory=self.drone.waypoint_history
        localHistory=[]
        for point in reversed(globalhistory):
            if point==(750,0):
                break
            localHistory.insert(0,point)
        #No sense if, it will be used later to distinguish the ferry drones
        if self.drone.identifier not in set(["x"]):
            #Initialization of the reward dictionary with default value for every neighboor
            for d in [v[1] for v in opt_neighbors]:
                if (cell_index,d,hash(str(localHistory))) not in self.actions_rewards.keys():
                    self.actions_rewards[(cell_index,d,hash(str(localHistory)),d)]=[]
                    self.actions_set.add((cell_index,d,hash(str(localHistory)),d))
                    self.actions_rewards[(cell_index,None,hash(str(localHistory)),d)]=[]
                    self.actions_set.add((cell_index,None,hash(str(localHistory)),d))
            # self.drone.residual_energy (that tells us when I'll come back to the depot).
            # Store your current action --- you can add several stuff if needed to take a reward later
            # Check if random choice (see epsilon greedy algorithm). 
            # Since there is no greedy selection implementation, the probability is 100% on random choice
            isRandomChoice=random.choices([True,False],weights=(100,0),k=1)[0]
            opt_neighbors=[v[1] for v in opt_neighbors]
            actual_time=time.time()
            if isRandomChoice:
                drone=self.untaken_drone(opt_neighbors,pkd)
                if pkd.event_ref.identifier not in self.taken_actions.keys():
                    self.taken_actions[pkd.event_ref.identifier]=set([])
                for collision in opt_neighbors:
                    self.taken_actions[pkd.event_ref.identifier].add((cell_index,drone,hash(str(localHistory)),collision))
                    self.actions_timestamp[(cell_index,drone,hash(str(localHistory)),collision)]=actual_time
                    self.actions_set.add((cell_index,drone,hash(str(localHistory)),collision))
                return drone
#----------------------GREEDY ACTION SELECTION, NOT IMPLEMENTED YET(STILL RANDOM)----------------------------------------
            self.perform_greedy_action(opt_neighbors,cell_index,localHistory)
            drone=self.untaken_drone(opt_neighbors,pkd)
            if pkd.event_ref.identifier not in self.taken_actions.keys():
                self.taken_actions[pkd.event_ref.identifier]=set([(cell_index,drone,hash(str(localHistory)))])
            else:
                self.taken_actions[pkd.event_ref.identifier].add((cell_index,drone,hash(str(localHistory))))
            self.actions_set.add((cell_index,drone,hash(str(localHistory))))
            self.actions_timestamp[(cell_index,drone,hash(str(localHistory)))]=time.time()
            return drone
        self.taken_actions[pkd.event_ref.identifier]=(cell_index,None,hash(str(localHistory)))
        return None
    def print(self):
        """
            This method is called at the end of the simulation, can be usefull to print some
                metrics about the learning process
        """
        print(self.actions_rewards)
        pass
    def untaken_drone(self,opt_neighbors,pkd):
        while True:
            drone=self.simulator.rnd_routing.choice(opt_neighbors+[None])
            if drone==None:
                return drone
            if drone.identifier not in pkd.hops:
                return drone
    def perform_greedy_action(self,opt_neighbors,cell_index,localHistory):
        past_actions=[x for x in self.actions_set if x[0]==cell_index and x[2]==hash(str(localHistory))]
        counter_cell_localhistory=0
        for action in past_actions:
            if len(self.actions_rewards[action])!=0:
                counter_cell_localhistory+=1
        if counter_cell_localhistory==0:
            print("Never taken action for",cell_index,localHistory)
        else:
            print("I have taken these actions before ",past_actions)
                


  
        




