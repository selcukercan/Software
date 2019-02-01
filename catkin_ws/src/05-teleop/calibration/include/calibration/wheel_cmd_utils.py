from abc import ABCMeta, abstractmethod
import rospy
from math import cos
import datetime
from scipy.signal import sweep_poly
import numpy as np

class BaseExperimentClass(object):
    __metaclass__ = ABCMeta

    @abstractmethod
    def __init__(self):
        pass

    @abstractmethod
    def generate_input(self, req_parameter_dict):
        """gets a dictionary containing the parameter values that are required to generate the input sequence for the particular input genre and returns a python list"""
        return

    def generate_experiment_label(self):
        """generates a custom experiment label with experiment time, experiment type, parameter names and their values"""
        now = datetime.datetime.now().strftime("%Y_%m_%d_%H_%M")
        experiment_name_base = self.name + "_"+ now

        param_val_label = ""
        for param in self.parameter_dict.keys():
            param_val_label += param + "_" + str(self.parameter_dict[param]) + "_"
        param_val_label = param_val_label[0:-1]

        return experiment_name_base + "_" + param_val_label

class RampUp(BaseExperimentClass):

    def __init__(self):
        rospy.loginfo("\ninitialized RampUp experiment with the default values for parameters")
        self.name = "ramp_up"
        self.parameter_dict = {'Nstep': 180, 'vFin': 0.5}

    def generate_input(self, req_parameter_dict):
        input_sequence = {'left_wheel': [], 'right_wheel': []}
        rospy.loginfo("[generate_input] generating input sequence of type {} with parameters {}".format("RampUp", str(req_parameter_dict)))

        for n in range(1, int(req_parameter_dict['Nstep']) + 1):
            v = req_parameter_dict['vFin']/req_parameter_dict['Nstep'] * n
            input_sequence['left_wheel'].append(v)
            input_sequence['right_wheel'].append(v)
        input_sequence['left_wheel'].append(0)
        input_sequence['right_wheel'].append(0)

        return input_sequence


class Sine(BaseExperimentClass):

    def __init__(self):
        rospy.loginfo("\ninitialized Sine experiment with the default values for parameters")
        self.name = "sine"
        self.parameter_dict = {'k1': 0.2, 'k2': 0.06, 'omega': 0.007, 'duration': 1500}

    def generate_input(self, req_parameter_dict):
        input_sequence = {'left_wheel': [], 'right_wheel': []}
        rospy.loginfo("[generate_input] generating input sequence of type {} with parameters {}".format("Sine", str(req_parameter_dict)))

        for t in range(0,int(req_parameter_dict['duration']), 10): # increments of 10 ms
            input_sequence['left_wheel'].append(req_parameter_dict['k1'] - req_parameter_dict['k2'] * cos(req_parameter_dict['omega'] * t))
            input_sequence['right_wheel'].append(req_parameter_dict['k1'] + req_parameter_dict['k2'] * cos(req_parameter_dict['omega'] * t))
        input_sequence['left_wheel'].append(0)
        input_sequence['right_wheel'].append(0)

        return input_sequence

class SweepSine(BaseExperimentClass):

    def __init__(self):
        rospy.loginfo("\ninitialized SweepSine experiment with the default values for parameters")
        self.name = "sweep_sine"
        self.parameter_dict = {'k1': 0.2, 'k2': 0.06, 'omega_low': 0.005, 'omega_high': 0.008, 'duration': 1500}

    def generate_input(self, req_parameter_dict):

        input_sequence = {'left_wheel': [], 'right_wheel': []}
        rospy.loginfo("[generate_input] generating input sequence of type {} with parameters {}".format("Sine", str(req_parameter_dict)))

        poly = np.poly1d([(req_parameter_dict['omega_high'] - req_parameter_dict['omega_low']) / req_parameter_dict['duration'], req_parameter_dict['omega_low']])
        t = np.arange(0,int(req_parameter_dict['duration']), 10)
        fval = sweep_poly(t, poly)

        input_sequence['left_wheel'] = req_parameter_dict['k1'] - req_parameter_dict['k2'] * fval
        input_sequence['right_wheel'] = req_parameter_dict['k1'] + req_parameter_dict['k2'] * fval

        input_sequence['left_wheel'] = np.append(input_sequence['left_wheel'], 0).tolist()
        input_sequence['right_wheel'] = np.append(input_sequence['right_wheel'], 0).tolist()

        simple_plot(t, input_sequence['left_wheel'], plot_name="left_wheel")
        #simple_plot(t, input_sequence['right_wheel'], plot_name="right_wheel")

        return input_sequence

class ExperimentUI():

    def __init__(self):
        pass

    def request_param_values(self, exp_name, param_dict):
        param_names = param_dict.keys()

        print "[UI INFO] Enter parameter values for {} experiment as prompted, if not entered default values are used\n".format(exp_name)
        print "[UI INFO] Parameters and their default values for {}: {}".format(exp_name, str(param_dict))

        for param_name in param_names:
            not_float_input = True
            not_pressed_enter = True

            while not_float_input and not_pressed_enter:
                print "\nenter a value for {} ...".format(param_name)
                user_input = raw_input()

                try:
                   valid_val = float(user_input) #try to cast into a float

                   not_float_input = False
                   param_dict[param_name] = valid_val
                   #print("[UI INFO] input float number value is: ", valid_val)
                except ValueError:
                    if user_input == "":
                        print("[UI INFO] pressed enter -> using the default value for {}:{}".format(param_name,param_dict[param_name]))
                        not_pressed_enter = False
                    else:
                        print("[UI WARNING] please enter a valid value (float or int)!")
        print "[UI INFO] completed param_val acquisition"

        return param_dict


if __name__ == "__main__":
    from plotting_utils import simple_plot
    """
    dur = 5.
    omega_low = 0.
    omega_high = 10.

    poly = np.poly1d([(omega_high - omega_low) / dur, omega_low])
    t = np.arange(0,dur,0.01)
    fval = sweep_poly(t, poly)

    simple_plot(t,fval, plot_name="test")
    """
    req_parameter_dict = {'k1': 0.2, 'k2': 0.06, 'omega_low': 0.003, 'omega_high': 0.007, 'duration': 1500}
    sweep_sine = SweepSine()
    sine_val = sweep_sine .generate_input(req_parameter_dict)
    print("sel")
# Include basic utility functions here
