import hassapi as hass
import json
class MorningRoutine(hass.Hass):
    def initialize(self):
        # Grab an object for the MQTT API
        self.mqtt = self.get_plugin_api("MQTT")
        # Make MQTT API Call
        self.mqtt.mqtt_unsubscribe("SleepAsAndroid")
        self.mqtt.mqtt_subscribe("SleepAsAndroid")

        self.mqtt.listen_event(self.print_mqtt, 'MQTT_MESSAGE', topic="SleepAsAndroid")

    def print_mqtt(self, event_name, data, kwargs):
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
            self.turn_on("switch.coffee_maker")
            self.call_service("light/turn_on", entity_id = "light.bedroom", kelvin = 2500, brightness=255, transition=10)
        elif event == "sleep_tracking_started":
            self.call_service("scene/apply", entity_id="scene.turn_off_all_lights")
        elif event == "smart_period":
            self.call_service("light/turn_on", entity_id = "light.bedroom", color_name = "red", brightness=1)
            self.call_service("light/turn_on", entity_id = "light.bedroom", kelvin = 2500, brightness=255, transition=1800)
        elif event == "alarm_snooze_clicked":
            self.call_service("light/turn_on", entity_id = "light.bedroom", color_name = "darkorange", brightness=20, transition=10)
            self.call_service("light/turn_on", entity_id = "light.bedroom", kelvin = 2500, brightness=255, transition=500)
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