import appdaemon.plugins.hass.hassapi as hass

#
# Hellow World App
#
# Args:
#

class HelloWorld(hass.Hass):

  def initialize(self):
    # # Native logger
    # testlogger = self.get_user_log("test_log")
    # testlogger.info("Hello %s", "Steven")
    # # self.log()
    # self.log("Hello", log="test_log")
