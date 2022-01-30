import hassapi as hass
import joblib
import numpy as np
from sklearn import svm
import tflite_runtime.interpreter as tf

class PresenceLights(hass.Hass):

    def initialize(self):
        self.log('initializing ...')
        self.mlmodel = svm.SVC()
        self.mlmodel = joblib.load('/home/pi/HomeAssistant/config/MLModels/lights.joblib')
        self.rooms = [' Office', ' Living Room', ' Kitchen', ' Office Bathroom', ' Bedroom', ' Master Bathroom']
        self.listen_state(self.__newstate, self.args["sensors"]["office"])
        self.listen_state(self.__newstate, self.args["sensors"]["living_room"])
        self.listen_state(self.__newstate, self.args["sensors"]["kitchen"])
        self.listen_state(self.__newstate, self.args["sensors"]["bedroom"])
        self.interpreter = tf.Interpreter(model_path='/home/pi/HomeAssistant/config/MLModels/model.tflite')
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()
        self.interpreter.allocate_tensors()
        self.room = ['Bedroom', 'Hallway', 'Kitchen', 'Living Room', 'Bathroom', 'Office', 'Office Bathroom']
    def __newstate(self, entity, attribute, old, new, kwargs):
        # load data into array. Strip out unknown and replace with -120 
        #data = np.array([np.nan_to_num(np.genfromtxt(np.array(
        data=np.array([ [self.get_state(self.args["sensors"]["bedroom"]),
                         self.get_state(self.args["sensors"]["kitchen"]),
                         self.get_state(self.args["sensors"]["living_room"]),
                         self.get_state(self.args["sensors"]["office"])
                         ]]).astype('float32') #), nan=-120)])

        p = self.mlmodel.predict_proba(data)
        # Only change the room if the propability is high enough
        #if max(p[0]) > float(self.entities.input_number.room_threshold.state)/100:
            #room = self.mlmodel.predict(data)[0]
            #self.set_textvalue('input_text.predicted_room', room)

        self.interpreter.set_tensor(self.input_details[0]['index'], data)
        self.interpreter.invoke()
        tf_p = self.interpreter.get_tensor(self.output_details[0]['index'])
        #self.log(tf_p)

        self.set_textvalue('input_text.predicted_room', self.room[tf_p[0].argmax()])
        self.set_value('input_number.bedroom_probability', int(tf_p[0][0]*100))
        self.set_value('input_number.hallway_probability', int(tf_p[0][1]*100))
        self.set_value('input_number.kitchen_probability', int(tf_p[0][2]*100))
        self.set_value('input_number.living_room_probability', int(tf_p[0][3]*100))
        self.set_value('input_number.bathroom_probability', int(tf_p[0][4]*100))
        self.set_value('input_number.office_probability', int(tf_p[0][5]*100))
        self.set_value('input_number.office_bathroom_probability', int(tf_p[0][6]*100))
