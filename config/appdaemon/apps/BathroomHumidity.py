import hassapi as hass

class BathroomHumidity(hass.Hass):

  def initialize(self):
    self.log('initializing bathroom humidity...')
    self.timer = None
    self.listen_state(self.__humidity_changed, self.args["humidity_sensor"])

  def __humidity_changed(self, entity, attribute, old, new, kwargs):
    if float(new) > float(self.get_state(self.args["humidity_trigger"])):
        self.log("Turning on bathroom fan at " + self.get_state(self.args["humidity_trigger"]) +"\% humidity")
        self.turn_on(self.args["fan"])
    else:
        self.turn_off(self.args["fan"])


