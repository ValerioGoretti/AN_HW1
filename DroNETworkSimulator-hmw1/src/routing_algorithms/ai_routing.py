import numpy as np
import hashlib
import time
from src.utilities import utilities as util
from src.routing_algorithms.BASE_routing import BASE_routing
import  random
from matplotlib import pyplot as plt
class AIRouting(BASE_routing):
    def __init__(self, drone, simulator):
        BASE_routing.__init__(self, drone, simulator)
        # random generator
        self.rnd_for_routing_ai = np.random.RandomState(self.simulator.seed)
        self.taken_actions = {}  #id event : (cell,action,hash)
        self.actions_rewards={}
        self.epsilon=10
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
        if id_event in self.taken_actions.keys():
            action=self.taken_actions[id_event]
            if outcome==-1:
                self.actions_rewards[action]=-2
                print(action,self.actions_rewards[action])
            else:
                self.actions_rewards[action]=delay
                print(action,self.actions_rewards[action])

    def relay_selection(self, opt_neighbors, pkd):
        """ arg min score  -> geographical approach, take the drone closest to the depot """
        # Only if you need --> several features:
        cell_index = util.TraversedCells.coord_to_cell(size_cell=self.simulator.prob_size_cell,
                                                      width_area=self.simulator.env_width,
                                                       x_pos=self.drone.coords[0],  # e.g. 1500
                                                        y_pos=self.drone.coords[1])[0]  # e.g. 500
        globalhistory=self.drone.waypoint_history
        localHistory=[]
        for point in reversed(globalhistory):
            if point==(750,0):
                break
            localHistory.insert(0,point)
        if self.drone.identifier not in set(["x"]):
            #Initialization of the reward dictionary if the element (cell,None,hash_history) is not stored yet
            if (cell_index,None,hash(str(localHistory))) not in self.actions_rewards:
                self.actions_rewards[(cell_index,None,hash(str(localHistory)))]=-1
            #Initialization of the reward dictionary with default value for every neighboor
            for d in [v[1] for v in opt_neighbors]:
                if (cell_index,d,hash(str(localHistory))) not in self.actions_rewards:
                    self.actions_rewards[(cell_index,d,hash(str(localHistory)))]=-1   
            # self.drone.history_path (which waypoint I traversed. We assume the mission is repeated)
            # self.drone.residual_energy (that tells us when I'll come back to the depot).
            #  .....
            # Store your current action --- you can add several stuff if needed to take a reward later
            # Check if random choice (see epsilon greedy algorithm)
    
            isRandomChoice=random.choices([True,False],weights=(self.epsilon,90),k=1)[0]
            if isRandomChoice:
                opt_neighbors=[v[1] for v in opt_neighbors]
                drone= self.simulator.rnd_routing.choice(opt_neighbors)
                self.taken_actions[pkd.event_ref.identifier]=(cell_index,drone,hash(str(localHistory)))
                return drone
            self.taken_actions[pkd.event_ref.identifier]=(cell_index,None,hash(str(localHistory)))
            return None
        self.taken_actions[pkd.event_ref.identifier]=(cell_index,None,hash(str(localHistory)))
        return None
    def print(self):
        """
            This method is called at the end of the simulation, can be usefull to print some
                metrics about the learning process
        """
        pass


