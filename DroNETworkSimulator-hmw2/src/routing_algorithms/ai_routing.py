
import numpy as np
from src.utilities import utilities as util
from src.routing_algorithms.BASE_routing import BASE_routing
from matplotlib import pyplot as plt
from src.utilities import config

class AIRouting(BASE_routing):
    def __init__(self, drone, simulator):
        BASE_routing.__init__(self, drone, simulator)
        # random generator
        self.rnd_for_routing_ai = np.random.RandomState(self.simulator.seed)
        self.taken_actions = {}  #id event : (old_action)

        self.drone_path=[]
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
            print("SCADUTOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO")
        # Be aware, due to network errors we can give the same event to multiple drones and receive multiple feedback for the same packet!!
        # NOTE: reward or update using the old action!!
        # STORE WHICH ACTION DID YOU TAKE IN THE PAST.
        # do something or train the model (?)
        if id_event in self.taken_actions:
            action = self.taken_actions[id_event]
            del self.taken_actions[id_event]

    def relay_selection(self, opt_neighbors, pkd):
        """ arg min score  -> geographical approach, take the drone closest to the depot """
        # Notice all the drones have different speed, and radio performance!!
        # you know the speed, not the radio performance.
        # self.drone.speed

        # Only if you need --> several features:
        cell_index = util.TraversedCells.coord_to_cell(size_cell=self.simulator.prob_size_cell,
                                                        width_area=self.simulator.env_width,
                                                        x_pos=self.drone.coords[0],  # e.g. 1500
                                                        y_pos=self.drone.coords[1])[0]  # e.g. 500

        #IDEA: Il RL si potrebbe usare per capire se il drone sta per riscendere? ha quasi finito il suo path in salita?
        #QUANDO STA PER FINIRE LA SIMULAZIONE TUTTI I DRONI DEVONO TORNARE ALTRIMENTI MUOIONO CON I PACCHETTI
        #NB: QUANDO UN DRONE STA TORNANDO AL DEPOT NON PUO' PRENDERE PACCHETTI

        # Salvataggio del path al primo giro
        if self.drone_path == [] and self.drone.waypoint_history != []:
            if self.drone.waypoint_history[self.drone.current_waypoint - 1][1] < self.drone.waypoint_history[self.drone.current_waypoint - 2][1]:
                self.drone_path = self.drone.waypoint_history.copy()
                # print(self.drone_path)
                # print(len(self.drone_path))

        # Se path è ricominciato vai al depot
        elif self.drone.next_target() in self.drone_path:
            if self.drone_path.index(self.drone.next_target()) == 0 and self.drone.buffer_length() >= 3:
                # print("giro ricominciato")
                return -1

        for hpk, drone_instance in opt_neighbors:
            if drone_instance.waypoint_history != []:
                if drone_instance.next_target()[1] == drone_instance.waypoint_history[0][1]:
                    #print("PACCHETTO DA: ", self.drone.identifier, "  A: ", drone_instance )
                    #print(self.simulator.cur_step)
                    return drone_instance


        if self.arrival_time(self.drone) > (self.simulator.len_simulation - self.simulator.cur_step)-300:
            return -1
            #print("il drone ", self.drone.identifier, " è tornato")


        action = None

        # self.drone.history_path (which waypoint I traversed. We assume the mission is repeated)
        # self.drone.residual_energy (that tells us when I'll come back to the depot).
        #  .....
        for hpk, drone_instance in opt_neighbors:
            #print(hpk)
            continue

        # Store your current action --- you can add several stuff if needed to take a reward later
        self.taken_actions[pkd.event_ref.identifier] = (action)

        # return action:
        # None --> no transmission
        # -1 --> move to depot
        # 0, ... , self.ndrones --> send packet to this drone
        return None  # here you should return a drone object!


    def print(self):
        """
            This method is called at the end of the simulation, can be usefull to print some
                metrics about the learning process
        """
        pass

    def arrival_time(self, drone):
        return util.euclidean_distance(self.simulator.depot.coords, drone.coords) / drone.speed
