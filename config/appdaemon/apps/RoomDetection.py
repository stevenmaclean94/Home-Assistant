import mqttapi as mqtt
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
        for device in self.args["devices"]:
            self.devices.setdefault(device["id"],{})["name"]=device["name"]
            self.devices.setdefault(device["id"],{})["x0"] = None
            self.devices.setdefault(device["id"],{})["roomname"] = None
            for room, pos in self.args["ep_rooms"].items():
                t = f"{self.args.get('rooms_topic', 'espresense/devices')}/iBeacon:{device['id'][0:8]}-{device['id'][8:12]}-{device['id'][12:16]}-{device['id'][16:20]}-{device['id'][20:32]}-{device['id'][32:36]}-{device['id'][36:40]}/{room}"
                self.log(f"Subscribing to topic {t}")
                self.mqtt_unsubscribe(t)
                self.mqtt_subscribe(t)
                self.listen_event(self.mqtt_message_esp, "MQTT_MESSAGE", topic=t)
                self.filter[room] = KalmanFilter(1, 1, transition_covariance=0.008, observation_covariance=4, observation_offsets=0, transition_offsets=0, initial_state_mean=0, initial_state_covariance=4)
                self.mean[room] = 0
                self.covariance[room] = 4
            for room, pos in self.args["ra_rooms"].items():
                t = f"{self.args.get('rooms_topic', 'room-assistant/entities')}/{room}/bluetooth-low-energy-presence-sensor/ble-{device['id'][0:32]}-{device['id'][32:36]}-{device['id'][36:40]}"
                self.log(f"Subscribing to topic {t}")
                self.mqtt_unsubscribe(t)
                self.mqtt_subscribe(t)
                self.listen_event(self.mqtt_message_ra, "MQTT_MESSAGE", topic=t)
                self.initial_state = True
        # self.run_every(self.update_room, "now", 0.5)
        self.hass.listen_state(self.home_assistant_esp, "input_number.steven_beacon_office_rssi_filter")

    def update_room(self, kwargs):
        for device in self.devices:
            distance_to_station = []
            station_coordinates = []
            sensors_used = []
            for room in self.data:
                if self.data[room]["id"] == device:
                    if (self.get_now() - self.data[room]["time"]).total_seconds() < float(self.args["allowed_time_delta"]):
                        # self.log(f"{room}: {self.data[room]['distance']}")
                        distance_to_station.append(self.data[room]['distance'])
                        sensors_used.append(room)
                        if room[0:2] == 'ra':
                            station_coordinates.append(self.args['ra_rooms'][room[3:]])
                        elif room[0:2] == 'ep':
                            station_coordinates.append(self.args['ep_rooms'][room[3:]])
                        else:
                            self.log("Could not match " + str(room[0:2]) + " id to a known station")
            # if len(distance_to_station) > 2:
            #     self.devices[device]["x0"] = position_solve(distance_to_station, np.array(station_coordinates), self.devices[device].get("x0", None))
            #     pos = self.devices[device]["x0"].tolist()                
            #     roomname, dist = room_solve(self,round(pos[0],2,),round(pos[1],2))
            #     self.mqtt_publish(f"{self.args.get('room_topic', 'roomDetection/rooms')}/{roomname}", json.dumps({"name": self.devices[device]['name'], "id": device, "distance": dist, "x":round(pos[0],2),"y":round(pos[1],2),"z":round(pos[2],2), "fixes":len(distance_to_station)}))
            #     self.mqtt_publish(f"{self.args.get('pos_topic', 'roomDetection/position')}/{device}", json.dumps({"x":round(pos[0],2),"y":round(pos[1],2),"z":round(pos[2],2), "fixes":len(distance_to_station)}))
            #     # self.mqtt_publish(f"{self.args.get('location_topic', 'espresense/location')}/{id}", json.dumps({"name":name, "longitude":(self.config["longitude"]+(pos[0]/111111)),"latitude":(self.config["latitude"]+(pos[1]/111111)),"elevation":(self.config.get("elevation","0")+pos[2]), "fixes":len(distance_to_stations),"measures":device["measures"]}))
            #     if roomname != self.devices[device]["roomname"]:
            #         self.fire_event("ROOM_CHANGE", room=roomname)
            #     self.devices[device]["roomname"] = roomname
            #     self.log(f"Detected in {roomname} with x,y coordinates of {pos} and these sensors used: {sensors_used} and this dist {distance_to_station}")
    
    def home_assistant_esp(self, entity, attribute, old, new, kwargs):
        """Process data from a home assistant rssi sensor"""
        id = '426c7565436861726d426561636f6e7338384949'
        update_time = self.get_now() 
        distance = 10**((-59 - float(new)) / 35);
        self.data["ep_office"] = {"id": id, "time": update_time, "distance": distance}
    def mqtt_message_esp(self, event_name, data, *args, **kwargs):
        """Process a message sent on the MQTT Topic."""
        topic = data.get("topic")
        payload = data.get("payload")

        topic_path = topic.split("/")
        room = topic_path[-1].lower()

        payload_json = {}
        try:
            payload_json = json.loads(payload)
        except ValueError:
            pass

        id = re.sub('[-]', "", payload_json.get("id")[8:])
        update_time = self.get_now()
        self.mean[room], self.covariance[room] = self.filter[room].filter_update(self.mean[room], self.covariance[room], payload_json.get("distance"))
        distance = self.mean[room][0]
        self.log(f"ESP Msg: {room} {id} {update_time} {distance}", level="DEBUG")

        self.data["ep_"+room] = {"id": id, "time": update_time, "distance": distance}

    def mqtt_message_ra(self, event_name, data, *args, **kwargs):
        """Process a message sent on the MQTT Topic."""
        topic = data.get("topic")
        payload = data.get("payload")

        topic_path = topic.split("/")
        room = topic_path[2].lower()

        payload_json = {}
        try:
            payload_json = json.loads(payload)
        except ValueError:
            pass
        # self.log(payload_json)
        id = re.sub('[-]', "", payload_json['entity']["id"][4:])
        update_time = self.convert_utc(payload_json["entity"]['attributes']["lastUpdatedAt"])
        try:
            distance = payload_json["entity"]["distances"][room]["distance"]
        except:
            return
        self.log(f"RA Msg: {room} {id} {update_time} {distance}", level="DEBUG")
        self.data["ra_"+room] = {"id": id, "time": update_time, "distance": distance}


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
    return minimize(
        error,
        x0,
        args=(stations_coordinates, distances_to_station),
        method="Nelder-Mead",
        options={'xatol': 0.001, 'fatol': 0.001, 'adaptive': True}
    ).x

def room_solve(self, xpos, ypos):
    for rooms in self.args["roomplans"]:
        if rooms["x1"] < float(xpos) < rooms["x2"] and rooms["y1"] < float(ypos) < rooms["y2"]:
            dist = np.sqrt((float(xpos) - rooms["x1"])**2 +(float(ypos) - rooms["y1"])**2)
            return rooms["name"], dist
    return "none", 0

def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)