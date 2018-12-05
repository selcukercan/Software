from abc import ABCMeta, abstractmethod

class AbstractClassExample(ABCMeta):
    @abstractmethod
    def generate_input(self, param_dict):
        """gets a dictionary containing the parameter values that are required to generate the input sequence for the particular input genre"""
        return

class RampUp(AbstractClassExample):

    def __init__(self, experiment_name, parameter_dict):
        self.name = experiment_name
        self.parameter_dict = parameter_dict

        self.generate_input(parameter_dict)

    def generate(self, parameter_dict):
        input_sequence = {'left_wheel': [], 'right_wheel': []}
        rospy.loginfo("[{}-generate] generating input sequence of type {} with parameters {}".format("Sine", str(parameter_dict)))

        for n in range(1, parameter_dict['Nstep'] + 1):
            v = parameter_dict['vFin']/parameter_dict['Nstep'] * n
            input_sequence['left_wheel'].append(v)
            input_sequence['right_wheel'].append(v)
        input_sequence['left_wheel'].append(0)
        input_sequence['right_wheel'].append(0)

    def SinCalib(self):
        rospy.loginfo("Sin calibration starts")

        for t in range(0,self.duration,10):
            self.sendCommand(self.k1+self.k2*cos(self.omega*t),self.k1-self.k2*cos(self.omega*t))
            rospy.sleep(1/self.frequency)

        self.sendCommand(0,0)
