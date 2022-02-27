
import hassapi as hass

class MotionLights(hass.Hass):

    def initialize(self):
        self.log('initializing ...')
        self.timer = None
        self.stop_timer = False
        self.listen_state(self.__motion_detected, self.args["sensors"]["motion_on"], new = "on")
        self.listen_state(self.__motion_stopped, self.args["sensors"]["motion_off"], new = "off")
        self.listen_state(self.__humidity_change, self.args["sensors"]["humidity"])
    def __motion_detected(self, entity, attribute, old, new, kwargs):
        self.turn_on(self.args["light"])
        self.cancel_timer(self.timer) 
    def __motion_stopped(self, entity, attribute, old, new, kwargs):
        self.__set_timer()  

    def __humidity_change(self, entity, attribute, old, new, kwargs):
        if float(new) > 75.0:               
            self.stop_timer = True
            self.cancel_timer(self.timer)
        else:
            self.stop_timer = False

    def __turn_off_light(self, kwargs):
        self.turn_off(self.args["light"])

    def __start_timer(self):
        if not self.stop_timer:
            self.timer = self.run_in(self.__turn_off_light, self.args["timeout"])

    def __set_timer(self): 
        if self.timer_running(self.timer):
            self.log('rescheduling timer ...')  
            self.cancel_timer(self.timer)  
            self.__start_timer()  
        else:
            self.log('starting timer ...')   
            self.__start_timer()