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
        #Dictionary for incremental formula action->Q(action)
        self.qN_dictionary={}
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
          #  if outcome==1:
           #     print("DELIVERED PACKET: "+str(id_event)+"__")
           # else:
           #     print("EXPIRED PACKET: "+str(id_event)+"__")
            for action in action_list:
                action_delay=actual_time-self.actions_timestamp[action]
                if outcome==-1:
                    self.actions_rewards[action].append(-4)
                    #incremental-----------
                    n=len(self.actions_rewards[action])
                    qN=self.qN_dictionary[action]
                    self.qN_dictionary[action]=qN+(1/n)*((-4)-qN) 
                else:
                    self.actions_rewards[action].append((-1)*(action_delay))
                    #incremental-------------
                    n=len(self.actions_rewards[action])
                    qN=self.qN_dictionary[action]
                    self.qN_dictionary[action]=qN+(1/n)*(action_delay-qN)
                   

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
                if (cell_index,d,hash(str(localHistory)),d) not in self.actions_rewards.keys():
                    self.actions_rewards[(cell_index,d,hash(str(localHistory)),d)]=[]
                    self.actions_set.add((cell_index,d,hash(str(localHistory)),d))
                    self.actions_rewards[(cell_index,None,hash(str(localHistory)),d)]=[]
                    self.actions_set.add((cell_index,None,hash(str(localHistory)),d))
                    self.qN_dictionary[(cell_index,None,hash(str(localHistory)),d)]=-3
                    self.qN_dictionary[(cell_index,d,hash(str(localHistory)),d)]=-3
            # self.drone.residual_energy (that tells us when I'll come back to the depot).
            # Store your current action --- you can add several stuff if needed to take a reward later
            # Check if random choice (see epsilon greedy algorithm). 
            # Since there is no greedy selection implementation, the probability is 100% on random choice
            isRandomChoice=random.choices([True,False],weights=(20,80),k=1)[0]
            opt_neighbors2=[v[1] for v in opt_neighbors]
            actual_time=time.time()
            if pkd.event_ref.identifier not in self.taken_actions.keys():
                self.taken_actions[pkd.event_ref.identifier]=set([])
           # for hpk,collision in opt_neighbors:
            #    if self.is_coming_back(hpk):
             #       for _,collision2 in opt_neighbors:
              #          if collision2==collision:
               #             self.taken_actions[pkd.event_ref.identifier].add((cell_index,None,hash(str(localHistory)),collision2))
                   ##         self.actions_timestamp[(cell_index,None,hash(str(localHistory)),collision2)]=actual_time
                    #        self.actions_set.add((cell_index,None,hash(str(localHistory)),collision2))
                     #   else:
                           # self.taken_actions[pkd.event_ref.identifier].add((cell_index,collision,hash(str(localHistory)),collision))
                          #  self.actions_timestamp[(cell_index,collision,hash(str(localHistory)),collision)]=actual_time
                            #self.actions_set.add((cell_index,collision,hash(str(localHistory)),collision))
                 #   return collision
            if isRandomChoice:
                #drone=self.untaken_drone(opt_neighbors,pkd)
                geo_drone=self.best_context_selection(opt_neighbors,pkd)
                random_choice=self.untaken_drone(opt_neighbors2,pkd)
                drone=random.choices([geo_drone,random_choice],weights=(70,30),k=1)[0]
                for collision in opt_neighbors2:
                    if collision!=drone:
                        self.taken_actions[pkd.event_ref.identifier].add((cell_index,None,hash(str(localHistory)),collision))
                        self.actions_timestamp[(cell_index,None,hash(str(localHistory)),collision)]=actual_time
                        self.actions_set.add((cell_index,None,hash(str(localHistory)),collision))
                    else:
                        self.taken_actions[pkd.event_ref.identifier].add((cell_index,drone,hash(str(localHistory)),collision))
                        self.actions_timestamp[(cell_index,drone,hash(str(localHistory)),collision)]=actual_time
                        self.actions_set.add((cell_index,drone,hash(str(localHistory)),collision))
              #  print("Chosen action: ",drone)
                return drone
#----------------------GREEDY ACTION SELECTION----------------------------------------
            else:
              #  print("Performing greedy action")
                greedy_action,reward=self.perform_greedy_action_incremental(cell_index,localHistory,opt_neighbors2,pkd,opt_neighbors)
                for collision in opt_neighbors2:        
                    if collision!=greedy_action[1]:
                        self.taken_actions[pkd.event_ref.identifier].add((cell_index,None,hash(str(localHistory)),collision))
                        self.actions_timestamp[(cell_index,None,hash(str(localHistory)),collision)]=actual_time
                        self.actions_set.add((cell_index,None,hash(str(localHistory)),collision))
                    else:
                        self.taken_actions[pkd.event_ref.identifier].add((cell_index,greedy_action[1],hash(str(localHistory)),collision))
                        self.actions_timestamp[(cell_index,greedy_action[1],hash(str(localHistory)),collision)]=actual_time
                        self.actions_set.add((cell_index,greedy_action[1],hash(str(localHistory)),collision))
                return greedy_action[1]
        return None
    def print(self):
        """
            This method is called at the end of the simulation, can be usefull to print some
                metrics about the learning process
        """
        pass
    def untaken_drone(self,opt_neighbors,pkd):
        while True:
            drone=self.simulator.rnd_routing.choice(opt_neighbors+[None])
            if drone==None:
                return drone
            if drone.identifier not in pkd.hops:
                return drone   
    def q_reward_dictionary(self,cell_index,localHistory,collision):
        past_actions=[x for x in self.actions_set if x[0]==cell_index and x[2]==hash(str(localHistory))and x[3]==collision]
        counter_cell_localhistory=0
        result={}
        for action in past_actions:
            if len(self.actions_rewards[action])!=0:
                counter_cell_localhistory+=1
                result[action]=statistics.mean(self.actions_rewards[action])
            else:
                result[action]=-3
        return result
    
    def perform_greedy_action(self,cell_index,localHistory,opt_neighbors,pkd):
        result={}
        for collision in opt_neighbors:
            result.update(self.q_reward_dictionary(cell_index,localHistory,collision))
        
        return [(action,reward) for action,reward in result.items() if reward==max(result.values()) and action[1].identifier not in pkd.hops][0]
    def q_reward_incremental_dictionary(self,cell_index,localHistory,collision):
       # if collision.identifier in set([0,1,2]):
        #    return {(cell_index,collision,hash(str(localHistory)),collision):1,(cell_index,None,hash(str(localHistory)),collision):-4}
        past_actions=[x for x in self.actions_set if x[0]==cell_index and x[2]==hash(str(localHistory))and x[3]==collision]
        counter_cell_localhistory=0
        result={}
        for action in past_actions:
            result[action]=self.qN_dictionary[action]
        return result
    def perform_greedy_action_incremental(self,cell_index,localHistory,opt_neighbors,pkd,opti):
        result={}
        for collision in opt_neighbors:
            result.update(self.q_reward_incremental_dictionary(cell_index,localHistory,collision))
        listResult=[]
        counter_unknown=0
        for action,reward in result.items():
            if action[1]==None:
                listResult.append((action,reward))
            elif action[1].identifier not in pkd.hops:
                listResult.append((action,reward))
            if reward==-3:
                counter_unknown+=1
        if counter_unknown==len(result.items()):
            return (("x",self.best_context_selection(opti,pkd),"x","x"),1)
        listResult.sort(key=lambda x:x[1],reverse=True)
        return listResult[0]
    def best_context_selection(self, opt_neighbors, pkd):
        """ arg min score  -> geographical approach, take the drone closest to the depot """
        best_drone_distance_from_depot = util.euclidean_distance(self.simulator.depot.coords, self.drone.coords)
        best_drone = None
        for hpk, drone_istance in opt_neighbors:
            exp_position = hpk.cur_pos  # without estimation, a simple geographic approach
           # exp_position= self.__estimated_neighbor_drone_position(hpk)
            exp_distance = util.euclidean_distance(exp_position, self.simulator.depot.coords)
            if exp_distance < best_drone_distance_from_depot:
                best_drone_distance_from_depot = exp_distance
                best_drone = drone_istance
        return best_drone
    def is_coming_back(self,hellopacket):
        return hellopacket.next_target==self.simulator.depot_coordinates


        
        
        


            


                


  
        




