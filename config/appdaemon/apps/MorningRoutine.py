import hassapi as hass
import json
class MorningRoutine(hass.Hass):
    def initialize(self):
        # Grab an object for the MQTT API
        self.mqtt = self.get_plugin_api("MQTT")
        # Make MQTT API Call
        self.mqtt.mqtt_unsubscribe("SleepAsAndroid")
        self.mqtt.mqtt_subscribe("SleepAsAndroid")

        self.mqtt.listen_event(self.handle_alarm, 'MQTT_MESSAGE', topic="SleepAsAndroid")

    def handle_alarm(self, event_name, data, kwargs):
        self.log('Event Incoming: ' + event_name + " with Data: ")
        self.log(data)
        payload = data.get("payload")
        self.log(payload)
        payload_json = {}
        try:
            payload_json = json.loads(payload)
        except ValueError:
            self.log("Error Loading Json")
            pass

        event = payload_json.get("event")
        self.log(event)
        # Handle the various events from SleepAsAndroid mqtt topic
        if event == "alarm_alert_start":
            self.run_in(self, self.turn_on_light, 0, entity_id = "light.bedroom", kelvin = 5000, brightness=255, transition=10)
        elif event == "sleep_tracking_started":
            self.call_service("scene/turn_on", entity_id="scene.goodnight")
        elif event == "smart_period":
            self.run_in(self, self.turn_on_light, 0, entity_id = "light.bedroom", color_name = "red", brightness=1)
            self.run_in(self, self.turn_on_light, 0,  entity_id = "light.bedroom", kelvin = 4500, brightness=255, transition=1800)
        elif event == "alarm_snooze_clicked":
            self.run_in(self, self.turn_on_light, 0,  entity_id = "light.bedroom", color_name = "darkorange", brightness=20, transition=10)
            self.run_in(self, self.turn_on_light, 10, entity_id = "light.bedroom", kelvin = 2500, brightness=255, transition=500)
        elif event == "alarm_alert_dismiss":
            pass
        elif event == "alarm_skip_next":
            pass
        elif event == "alarm_snooze_canceled":
            # Canceled an alarm that is currently snoozed
            pass
        elif event == "show_skip_next_alarm":
            pass
        elif event == "before_smart_period":
            pass
        elif event == "sleep_tracking_stopped":
            pass
        elif event == "sleep_tracking_paused":
            pass
        elif event == "sleep_tracking_resumed":
            pass
        elif event == "time_to_bed_alarm_alert":
            pass
        value1 = payload_json.get("value1")
        value2 = payload_json.get("value2")

    def turn_on_light(self, kwargs):
        if 'color_name' in kwargs:
            self.call_service("light/turn_on", entity_id = kwargs['entity'], color_name = kwargs['color_name'], brightness = kwargs['brightness'], transition = kwargs['transition_time'])
        elif 'temperature' in kwargs:
            self.call_service("light/turn_on", entity_id = kwargs['entity'], kelvin = kwargs['temperature'], brightness = kwargs['brightness'], transition = kwargs['transition_time'])
        else:
            self.call_service("light/turn_on", entity_id = kwargs['entity'], brightness = kwargs['brightness'], transition = kwargs['transition_time'])
