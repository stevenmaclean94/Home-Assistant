import mqttapi as mqtt
import hassapi as hass

import json
import re
import numpy as np
from scipy.optimize import minimize
from pykalman import KalmanFilter


class RoomDetection(mqtt.Mqtt):
    def initialize(self):
        self.set_namespace("mqtt")
        self.hass = self.get_plugin_api("HASS")
        self.devices = {}
        self.data = {}
        self.filter = {}
        self.mean = {}
        self.covariance = {}
        self.initial_state = True
        self.x = np.array([0,0,0,0,0,0])
        self.P = None
        self.dt = self.args["dt"]
        for id, device in self.args["devices"].items():
            self.devices.setdefault(id,{})["name"]=device["name"]
            self.devices.setdefault(id,{})["x0"] = None
            self.devices.setdefault(id,{})["roomname"] = None
            for roomname, room in device["rooms"].items():
                self.log(f"Listening to the state of {room['sensor']}")
                self.hass.listen_state(self.home_assistant_esp, room["sensor"], id=id, roomname=roomname, rssi_1m=room['rssi_1m'])
        self.run_every(self.update_room, "now", self.dt)

    def update_room(self, kwargs):
        for device in self.devices:
            distance_to_station = []
            station_coordinates = []
            sensors_used = []
            for room in self.data:
                if self.data[room]["id"] == device:
                    if (self.get_now() - self.data[room]["time"]).total_seconds() < float(self.args["allowed_time_delta"]):
                        distance_to_station.append(self.data[room]['distance'])
                        sensors_used.append(room)
                        station_coordinates.append(self.args["devices"][device]['rooms'][room]["pos"])
            if len(distance_to_station) > 2:
                res = position_solve(distance_to_station, np.array(station_coordinates), self.devices[device].get("x0", None))
                self.devices[device]["x0"] = res.x
                pos = self.devices[device]["x0"].tolist()  
            else:
                pos = [0, 0, 0]
                pos[0] = self.x[0]
                pos[1] = self.x[2]
                pos[2] = self.x[3]
            if self.initial_state:
                self.kf = self.initialize_kalman2(np.asarray(pos)) # can't do in initialize() because we need the inital state estimate
                self.initial_state = False
            else:
                self.run_filter2(pos)
                pos[0] = self.x[0]
                pos[1] = self.x[2]
                pos[2] = self.x[4]
                self.devices[device]["x0"] = pos

            roomname, dist = room_solve(self,round(pos[0],2,),round(pos[1],2))
            self.mqtt_publish(f"{self.args.get('room_topic', 'roomDetection/rooms')}/{roomname}", json.dumps({"name": self.devices[device]['name'], "id": device, "distance": dist, "x":round(pos[0],2),"y":round(pos[1],2),"z":round(pos[2],2), "fixes":len(distance_to_station)}))
            # self.mqtt_publish(f"{self.args.get('pos_topic', 'roomDetection/position')}/{device}", json.dumps({"x":round(pos[0],2),"y":round(pos[1],2),"z":round(pos[2],2), "fixes":len(distance_to_station)}))
            # self.mqtt_publish(f"{self.args.get('location_topic', 'espresense/location')}/{id}", json.dumps({"name":name, "longitude":(self.config["longitude"]+(pos[0]/111111)),"latitude":(self.config["latitude"]+(pos[1]/111111)),"elevation":(self.config.get("elevation","0")+pos[2]), "fixes":len(distance_to_stations),"measures":device["measures"]}))
            if roomname != self.devices[device]["roomname"]:
                self.fire_event("ROOM_CHANGE", room=roomname)
            self.devices[device]["roomname"] = roomname
            self.fire_event("ROOM_CHANGE", room=roomname, namespace='default')
            self.hass.set_textvalue('input_text.predicted_room', roomname)

            self.hass.set_value('input_number.steven_beacon_x_position', round(self.x[0],2))
            self.hass.set_value('input_number.steven_beacon_y_position', round(self.x[2],2))
            self.hass.set_value('input_number.steven_beacon_z_position', round(self.x[4],2))
            self.hass.set_value('input_number.steven_beacon_x_velocity', round(self.x[1],2))
            self.hass.set_value('input_number.steven_beacon_y_velocity', round(self.x[3],2))
            self.hass.set_value('input_number.steven_beacon_z_velocity', round(self.x[5],2))
            # self.log(f"Detected in {roomname} with x,y coordinates of {pos} and these sensors used: {sensors_used} and this dist {distance_to_station}")
            # self.log(pos)

    def home_assistant_esp(self, entity, attribute, old, new, kwargs):
        """Process data from a home assistant rssi sensor"""
        id = kwargs['id']
        roomname = kwargs['roomname']
        rssi_1m = kwargs['rssi_1m']
        update_time = self.get_now() 
        distance = 10**((rssi_1m - float(new)) / 35);
        self.data[roomname] = {"id": id, "time": update_time, "distance": distance}
        self.log(f"ESP Msg: {roomname} {id} {update_time} {distance}", level="DEBUG")

    
    def run_filter2(self, obs):
        (self.x, self.P) = self.kf.filter_update(filtered_state_mean = self.x, filtered_state_covariance = self.P, observation=obs)
        return self.x

    def initialize_kalman2(self, pos):
        """Super basic model of just velocity -> position filter. 
        The covariance were estimated offline with pykalman em method """
        trans_matrix = np.array([[1, self.dt, 0, 0  , 0, 0  ],
                                 [0, 1  , 0, 0  , 0, 0  ],
                                 [0, 0  , 1, self.dt, 0, 0  ],
                                 [0, 0  , 0, 1  , 0, 0  ],
                                 [0, 0  , 0, 0  , 1, self.dt],
                                 [0, 0  , 0, 0  , 0, 1  ]])
        trans_cov = np.array([[ 1.64479505e-02,  1.47195483e-02,  2.88949784e-05,  2.06811853e-05,  2.88949784e-05,  2.06811853e-05],
                              [ 1.47195483e-02,  6.04056647e-02, -6.67845284e-06,  3.17364102e-05,  2.88949784e-05,  2.06811853e-05],
                              [ 2.88949784e-05, -6.67845284e-06,  1.66022573e-02,  1.48186279e-02,  2.88949784e-05,  2.06811853e-05],
                              [ 2.06811853e-05,  3.17364102e-05,  1.48186279e-02,  6.08256823e-02,  2.88949784e-05,  2.06811853e-05],
                              [ 2.88949784e-05, -6.67845284e-06,  2.88949784e-05,  2.06811853e-05,  1.66022573e-02,  1.48186279e-02],
                              [ 2.06811853e-05,  3.17364102e-05,  2.88949784e-05,  2.06811853e-05,  1.48186279e-02,  6.08256823e-02]])
        trans_offsets = np.array([0,0,0,0,0,0])

        obs_cov = np.array([[0.56363901, 0.00163114, 0.00163114],
                            [0.00163114, 0.56913398, 0.00163114],
                            [0.00163114, 0.00163114, 0.56913398],])

        obs_matrix = np.array([[1, 0, 0, 0, 0, 0],
                               [0, 0, 1, 0, 0, 0],
                               [0, 0, 0, 0, 1, 0]])

        obs_offsets = np.array([0, 0, 0])

        init_state_mean = np.array([pos[0],0,pos[1],0,pos[2],0])
        init_state_cov = np.array([[ 2.82471820e-03, -3.05271446e-03,  3.11647823e-06, -2.28635812e-06,  3.11647823e-06, -2.28635812e-06],
                                   [-3.05271446e-03,  3.00031102e-02, -2.87044073e-06,  1.33237447e-05,  3.11647823e-06, -2.28635812e-06],
                                   [ 3.11647823e-06, -2.87044073e-06,  2.83611834e-03, -3.06284117e-03,  3.11647823e-06, -2.28635812e-06],
                                   [-2.28635812e-06,  1.33237447e-05, -3.06284117e-03,  3.00892379e-02,  3.11647823e-06, -2.28635812e-06],
                                   [ 3.11647823e-06, -2.87044073e-06,  3.11647823e-06, -2.28635812e-06,  2.83611834e-03, -3.06284117e-03],
                                   [-2.28635812e-06,  1.33237447e-05,  3.11647823e-06, -2.28635812e-06, -3.06284117e-03,  3.00892379e-02]])
        self.x = init_state_mean
        self.P = init_state_cov
        return KalmanFilter(transition_matrices = trans_matrix,
                            observation_matrices = obs_matrix,
                            initial_state_mean = init_state_mean,
                            initial_state_covariance = init_state_cov,
                            observation_covariance = obs_cov*500,
                            transition_covariance = trans_cov/10,
                            transition_offsets = trans_offsets,
                            observation_offsets = obs_offsets)   
    def run_filter(self, obs):
        (self.x, self.P) = self.kf.filter_update(filtered_state_mean = self.x, filtered_state_covariance = self.P, observation=obs)
        return self.x

    def initialize_kalman(self, pos):
        """Super basic model of just velocity -> position filter. 
        The covariance were estimated offline with pykalman em method """
        trans_matrix = np.array([[1, 0.5, 0, 0  ],
                                 [0, 1  , 0, 0  ],
                                 [0, 0  , 1, 0.5],
                                 [0, 0  , 0, 1  ]])
        trans_cov = np.array([[ 1.64479505e-02,  1.47195483e-02,  2.88949784e-05,  2.06811853e-05],
                              [ 1.47195483e-02,  6.04056647e-02, -6.67845284e-06,  3.17364102e-05],
                              [ 2.88949784e-05, -6.67845284e-06,  1.66022573e-02,  1.48186279e-02],
                              [ 2.06811853e-05,  3.17364102e-05,  1.48186279e-02,  6.08256823e-02]])
        trans_offsets = np.array([0,0,0,0])

        obs_cov = np.array([[0.56363901, 0.00163114],
                            [0.00163114, 0.56913398]])

        obs_matrix = np.array([[1, 0, 0, 0],
                               [0, 0, 1, 0]])

        obs_offsets = np.array([0, 0])

        init_state_mean = np.array([pos[0],0,pos[1],0,])
        init_state_cov = np.array([[ 2.82471820e-03, -3.05271446e-03,  3.11647823e-06, -2.28635812e-06],
                                   [-3.05271446e-03,  3.00031102e-02, -2.87044073e-06,  1.33237447e-05],
                                   [ 3.11647823e-06, -2.87044073e-06,  2.83611834e-03, -3.06284117e-03],
                                   [-2.28635812e-06,  1.33237447e-05, -3.06284117e-03,  3.00892379e-02]])
        self.x = init_state_mean
        self.P = init_state_cov
        return KalmanFilter(transition_matrices = trans_matrix,
                            observation_matrices = obs_matrix,
                            initial_state_mean = init_state_mean,
                            initial_state_covariance = init_state_cov,
                            observation_covariance = obs_cov*10,
                            transition_covariance = trans_cov,
                            transition_offsets = trans_offsets,
                            observation_offsets = obs_offsets)

class RoomDetection2(hass.Hass):
    def initialize(self):
        self.pos = None
        self.roomname = ''
        self.initial_state = True
        self.run_every(self.update_room, "now", self.args['dt'])

    def update_room(self, kwargs):
        
        distance_to_station = []
        station_coordinates = []
        for room in self.args['rooms']:
            if (self.get_now() - self.convert_utc(self.get_state(room['sensor'], attribute='last_updated'))).total_seconds() < self.args['dt']:
                station_coordinates.append(room['pos'])
                distance_to_station.append(float(self.get_state(room['sensor'])))

        if len(distance_to_station) > 2:
            # self.log(distance_to_station)
            self.pos = position_solve(distance_to_station, np.array(station_coordinates), self.pos)

            if self.initial_state and self.args['use_filter']:
                self.kf = self.initialize_kalman(np.asarray(self.pos)) # can't do in initialize() because we need the inital state estimate
                self.initial_state = False
            else:
                self.run_filter(self.pos[0:2])
                self.pos[0] = self.x[0]
                self.pos[1] = self.x[2]
            self.set_value('input_number.steven_beacon_x_position', self.pos[0])
            self.set_value('input_number.steven_beacon_y_position', self.pos[1])
            roomname, dist = room_solve(self,round(self.pos[0],2,),round(self.pos[1],2))
            if roomname != self.roomname:
                self.fire_event("ROOM_CHANGE", room=roomname, namespace='default')
                self.set_textvalue('input_text.predicted_room', roomname)
                self.roomname = roomname

    def run_filter(self, obs):
        (self.x, self.P) = self.kf.filter_update(filtered_state_mean = self.x, filtered_state_covariance = self.P, observation=obs)
        return self.x

    def initialize_kalman(self, pos):
        """Super basic model of just velocity -> position filter. 
        The covariance were estimated offline with pykalman em method """
        trans_matrix = np.array([[1, 0.5, 0, 0  ],
                                 [0, 1  , 0, 0  ],
                                 [0, 0  , 1, 0.5],
                                 [0, 0  , 0, 1  ]])
        trans_cov = np.array([[ 1.64479505e-02,  1.47195483e-02,  2.88949784e-05,  2.06811853e-05],
                              [ 1.47195483e-02,  6.04056647e-02, -6.67845284e-06,  3.17364102e-05],
                              [ 2.88949784e-05, -6.67845284e-06,  1.66022573e-02,  1.48186279e-02],
                              [ 2.06811853e-05,  3.17364102e-05,  1.48186279e-02,  6.08256823e-02]])
        trans_offsets = np.array([0,0,0,0])

        obs_cov = np.array([[0.56363901, 0.00163114],
                            [0.00163114, 0.56913398]])

        obs_matrix = np.array([[1, 0, 0, 0],
                               [0, 0, 1, 0]])

        obs_offsets = np.array([0, 0])

        init_state_mean = np.array([pos[0],0,pos[1],0])
        init_state_cov = np.array([[ 2.82471820e-03, -3.05271446e-03,  3.11647823e-06, -2.28635812e-06],
                                   [-3.05271446e-03,  3.00031102e-02, -2.87044073e-06,  1.33237447e-05],
                                   [ 3.11647823e-06, -2.87044073e-06,  2.83611834e-03, -3.06284117e-03],
                                   [-2.28635812e-06,  1.33237447e-05, -3.06284117e-03,  3.00892379e-02]])
        self.x = init_state_mean
        self.P = init_state_cov
        return KalmanFilter(transition_matrices = trans_matrix,
                            observation_matrices = obs_matrix,
                            initial_state_mean = init_state_mean,
                            initial_state_covariance = init_state_cov,
                            observation_covariance = obs_cov,
                            transition_covariance = trans_cov,
                            transition_offsets = trans_offsets,
                            observation_offsets = obs_offsets)
def position_solve(distances_to_station, stations_coordinates, last):
    def error(x, c, r):
        return sum([(np.linalg.norm(x - c[i]) - r[i]) ** 2 for i in range(len(c))])

    l = len(stations_coordinates)
    
    S = sum(distances_to_station)
    # compute weight vector for initial guess
    W = [((l - 1) * S) / (S - w) for w in distances_to_station]
    # get initial guess of point location
    # x0 = last if last is not None else sum([W[i] * stations_coordinates[i] for i in range(l)])
    x0 = sum([W[i] * stations_coordinates[i] for i in range(l)])
    # optimize distance from signal origin to border of spheres
    res = minimize(
        error,
        x0,
        args=(stations_coordinates, distances_to_station),
        method="Nelder-Mead",
        options={'xatol': 0.001, 'fatol': 0.001, 'adaptive': True}
    )

    return res

def room_solve(self, xpos, ypos):
    for rooms in self.args["roomplans"]:
        if rooms["x1"] < float(xpos) < rooms["x2"] and rooms["y1"] < float(ypos) < rooms["y2"]:
            dist = np.sqrt((float(xpos) - rooms["x1"])**2 +(float(ypos) - rooms["y1"])**2)
            return rooms["name"], dist
    return "none", 0

def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)