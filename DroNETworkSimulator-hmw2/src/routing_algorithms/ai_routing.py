
import numpy as np
from src.utilities import utilities as util
from src.routing_algorithms.BASE_routing import BASE_routing
from matplotlib import pyplot as plt
from src.utilities import config
import time
import random

class AIRouting(BASE_routing):
    def __init__(self, drone, simulator):
        BASE_routing.__init__(self, drone, simulator)
        # random generator
        self.rnd_for_routing_ai = np.random.RandomState(self.simulator.seed)
        self.taken_actions = {}  #id event : [action]
        self.drone_path=[]
        self.counter=0
        self.set_collision=set({})
        self.set_waypoint=set({})
        self.completed_lap=False
        self.packet_dictio={}
        self.packet_set=set()
        self.packet_generation=[]
        self.first_waypoint=None

        self.qtable={}


    def feedback(self, drone, id_event, delay, outcome):
        """ return a possible feedback, if the destination drone has received the packet """
        if config.DEBUG:
            # Packets that we delivered and still need a feedback
            print("Drone: ", self.drone.identifier, "---------- has delivered: ", self.taken_actions)

            # outcome == -1 if the packet/event expired; 0 if the packets has been delivered to the depot
            # Feedback from a delivered or expired packet
            print("Drone: ", self.drone.identifier, "---------- just received a feedback:",
                  "Drone:", drone, " - id-event:", id_event, " - delay:",  delay, " - outcome:", outcome)
        if outcome==-1:
            print("Packet Expired: ",id_event)
        
        if id_event in self.packet_set and drone==self.drone:
            if(self.drone.identifier==1):
                print("fedback--------------------------------------------")
            self.packet_generation=[x for x in self.packet_generation if x[0].event_ref.identifier!=id_event]
            #print(self.packet_generation)
            
            self.packet_generation=sorted(self.packet_generation,key=lambda x: x[1])
            
        if id_event in self.taken_actions:
            action = self.taken_actions[id_event]
            del self.taken_actions[id_event]
            if outcome==0:
                self.packet_dictio[id_event]=True


    def relay_selection(self, opt_neighbors, pkd):
        """ arg min score  -> geographical approach, take the drone closest to the depot """
        cell_index = util.TraversedCells.coord_to_cell(size_cell=self.simulator.prob_size_cell,
                                                        width_area=self.simulator.env_width,
                                                        x_pos=self.drone.coords[0],  # e.g. 1500
                                                        y_pos=self.drone.coords[1])[0]  # e.g. 500

        print("DRONE ", self.drone.identifier, "   ", self.drone.all_packets(), " len ", len(self.drone.all_packets()))

        for hpk, drone_instance in opt_neighbors:
            #QUANDO C'è UNA COLLISIONE AGGUNGIAMO ALLA TABLE L'EVENTO SE NON C'è GIà E INIZIALIZZIAMO LE POSSIBILI MOSSE A 0.6
            if (cell_index, self.drone.identifier, drone_instance.identifier) not in self.qtable.keys():
                self.qtable[(cell_index, self.drone.identifier, drone_instance.identifier)]=(0.6, 0.6)
                #print(self.qtable)
            else:
                if self.qtable[(cell_index, self.drone.identifier, drone_instance.identifier)][0]>self.qtable[(cell_index, self.drone.identifier, drone_instance.identifier)][1]:
                    return None
                else:
                    if self.qtable[(cell_index, self.drone.identifier, drone_instance.identifier)][0]<=self.qtable[(cell_index, self.drone.identifier, drone_instance.identifier)][1]:
                        if self.qtable[(cell_index, self.drone.identifier, drone_instance.identifier)][0] == self.qtable[(cell_index, self.drone.identifier, drone_instance.identifier)][1]:
                            #print("==")
                            a = random.randrange(0, 2)
                            print(a)
                            if a==0:
                                return None
                            else:
                                if drone_instance not in pkd.hops:
                                    if (cell_index,pkd.id) not in self.taken_actions:
                                        print(self.simulator.cur_step, " PASSA A ", drone_instance, " da ", self.drone.identifier)
                                        self.taken_actions[(cell_index,pkd.id)]={drone_instance,self.drone.identifier}
                                        return drone_instance
                                    else:
                                        if {drone_instance,self.drone.identifier}.issubset(self.taken_actions[(cell_index,pkd.id)]):
                                            self.taken_actions[(cell_index,pkd.id)].append({drone_instance,self.drone.identifier})
                                            return drone_instance
                        else:
                            if drone_instance not in pkd.hops:
                                return drone_instance

        '''
        if self.is_time_to_goback():
            return -1



        if self.first_waypoint==None:
            self.first_waypoint=self.drone.next_target()
        globalhistory=self.drone.waypoint_history
        localHistory=[]
        for point in reversed(globalhistory):
            localHistory.insert(0,point)
            self.set_waypoint.add(point)
            if point[0]==globalhistory[0][0] and point[1]==globalhistory[0][1]:
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
                return -1


        if self.arrival_time(self.drone) > (self.simulator.len_simulation - self.simulator.cur_step)-300:
            return -1
            #print("il drone ", self.drone.identifier, " è tornato")

        '''



        #action = None
        #self.taken_actions[pkd.event_ref.identifier] = (action)

    def print(self):
        """
            This method is called at the end of the simulation, can be usefull to print some
                metrics about the learning process
        """
        #count=len([x for x in self.packet_dictio.values() if x==False])
       # print(self.drone.identifier,"Packet not delivered: ",count)
        #print(self.drone_path)
        pass
    def arrival_time(self, drone):
        tot=(util.euclidean_distance(drone.next_target(), drone.coords) / drone.speed)+(util.euclidean_distance(drone.next_target(), self.simulator.depot.coords)/drone.speed)
        return (self.simulator.cur_step*self.simulator.time_step_duration)+tot
    def is_time_to_goback(self):
        time_expected=self.arrival_time(self.drone)
        end_expected=self.simulator.len_simulation*self.simulator.time_step_duration+(self.simulator.cur_step*self.simulator.time_step_duration)
        return time_expected>end_expected
    def is_packet_expiring(self,pkd):
        time_left=8000-(self.simulator.cur_step*self.simulator.time_step_duration-pkd.time_step_creation*self.simulator.time_step_duration)
        expected_time=self.arrival_time(self.drone)
       # if self.drone.identifier==1:
           # print("Tempo rimanente: ",time_left,pkd.event_ref.identifier)
            #print("Tempo di arrivo: ", expected_time)
        return expected_time>time_left



