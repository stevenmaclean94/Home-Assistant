import hassapi as hass


class PresenceLights(hass.Hass):

    def initialize(self):
        self.log('initializing ...')

        self.listen_event(self.room_change, "ROOM_CHANGE", namespace="mqtt")
        self.prev_room = None

    def room_change(self, event_name, data, kwargs):
        room = data['room']
        # self.log('Event recieved for room ' + room)

        self.set_textvalue('input_text.predicted_room', room)
        if room == 'office':
            self.turn_on('switch.office_light')
            self.turn_off('light.office_bathroom')
        elif room == 'office_bathroom':
            self.turn_on('light.office_bathroom')
            self.turn_off('switch.office_light')
        self.prev_room = room