from abc import ABCMeta, abstractmethod
import rospy
from math import cos
import datetime

class BaseExperimentClass(object):
    __metaclass__ = ABCMeta

    @abstractmethod
    def __init__(self):
        pass

    @abstractmethod
    def generate_input(self, param_dict):
        """gets a dictionary containing the parameter values that are required to generate the input sequence for the particular input genre"""
        return

    def generate_experiment_label(self):
        """generates a custom experiment label considering experiment time, experiment type, parameter names and their values"""
        now = datetime.datetime.now().strftime("%Y_%m_%d_%H:%M")
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

    def generate_input(self, parameter_dict):
        parameter_dict = self.parameter_dict

        input_sequence = {'left_wheel': [], 'right_wheel': []}
        rospy.loginfo("[generate_input] generating input sequence of type {} with parameters {}".format("RampUp", str(parameter_dict)))

        for n in range(1, int(parameter_dict['Nstep']) + 1):
            v = parameter_dict['vFin']/parameter_dict['Nstep'] * n
            input_sequence['left_wheel'].append(v)
            input_sequence['right_wheel'].append(v)
        input_sequence['left_wheel'].append(0)
        input_sequence['right_wheel'].append(0)

        return input_sequence


class Sine(BaseExperimentClass):

    def __init__(self):
        rospy.loginfo("\ninitialized Sine experiment with the default values for parameters")
        self.name = "sine"
        self.parameter_dict = {'k1': 0.2, 'k2': 0.06, 'omega': 0.007, 'duration': 2000}

    def generate_input(self, parameter_dict):
        parameter_dict = self.parameter_dict

        input_sequence = {'left_wheel': [], 'right_wheel': []}
        rospy.loginfo("[generate_input] generating input sequence of type {} with parameters {}".format("Sine", str(parameter_dict)))

        for t in range(0,int(parameter_dict['duration']),10):
            input_sequence['left_wheel'].append(parameter_dict['k1'] - parameter_dict['k2'] * cos(parameter_dict['omega'] * t))
            input_sequence['right_wheel'].append(parameter_dict['k1'] + parameter_dict['k2'] * cos(parameter_dict['omega'] * t))
        input_sequence['left_wheel'].append(0)
        input_sequence['right_wheel'].append(0)

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



# Include basic utility functions here
