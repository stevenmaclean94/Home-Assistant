import hassapi as hass


class PresenceLights(hass.Hass):

    def initialize(self):
        self.log('initializing ...')

        self.listen_event(self.room_change, "ROOM_CHANGE")
        self.prev_room = None

    def room_change(self, event_name, data, kwargs):
        room = data['room']
        # self.log('Event recieved for room ' + room)
        if room == 'none':
            return
        if self.prev_room != room:
            self.set_textvalue('input_text.predicted_room', room)

            if self.prev_room == 'office':
                self.turn_off('switch.office_light')
            elif self.prev_room == 'office_bathroom':
                self.turn_off('light.office_bathroom')
            elif self.prev_room == 'living_room':
                self.turn_off('switch.in_wall_smart_switch')
            elif self.prev_room == 'bedroom':
                self.turn_off('light.bedroom')
            if room == 'office':
                self.turn_on('switch.office_light')
                self.turn_off('light.office_bathroom')
            elif room == 'office_bathroom':
                self.turn_on('light.office_bathroom')
                self.turn_off('switch.office_light')
            elif room == 'living_room':
                    self.turn_on('switch.in_wall_smart_switch')
            elif room == 'bedroom':
                self.turn_on('light.bedroom')
        self.prev_room = room