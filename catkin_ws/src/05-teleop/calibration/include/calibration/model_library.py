from abc import ABCMeta, abstractmethod
import rospy
from scipy.integrate import solve_ivp
import numpy as np
from calibration.data_adapter_utils import *
from math import cos
import datetime

class BaseModelClass(object):
    __metaclass__ = ABCMeta

    @abstractmethod
    def __init__(self):
        pass

    @abstractmethod
    def model(self, x_cur, u_cur, p):
        """
        Args:
            x_cur: state values, pos_x, pos_y, rot_z
            u_cur: input values, wheel_right, wheel_left
            p: model parameters

        Returns:
            x_next: model prediction for the state values
        """
        return

class Model1(BaseModelClass):

    def __init__(self):
        self.name = "model1"
        rospy.loginfo("\nusing model type: [{}]".format(self.name))

    def model(self, t, x, u, p):
        # input commands + model params
        [x, y, theta] = x
        (cmd_right, cmd_left) = u
        (cl, cr) = p
        L = 0.1 # 10 cm

        """
        # kinetic states through actuation
        vx_pred = c * (cmd_right + cmd_left) * 0.5 + tr * (cmd_right - cmd_left) * 0.5
        omega_pred = cl * (cmd_right - cmd_left) * 0.5 + tr * (cmd_right + cmd_left) * 0.5
        """
        # kinetic states through actuation
        vx = (cr * cmd_right + cl * cmd_left)
        omega = (cr * cmd_right - cl * cmd_left) / L

        # position states in relation to kinetic states
        x_dot = (np.cos(theta * np.pi / 180.0) * vx)
        y_dot = (np.sin(theta * np.pi / 180.0) * vx)

        theta_dot = (omega)

        return [x_dot, y_dot, theta_dot]

class Model2(BaseModelClass):

    def __init__(self):
        self.name = "model1"
        rospy.loginfo("\nusing model type: [{}]".format(self.name))

    def model(self, t, x, u, p):
        # input commands + model params
        (cmd_right, cmd_left) = u
        (c, cl, tr) = p

        # kinetic states through actuation
        vx_pred = c * (cmd_right + cmd_left) * 0.5 + tr * (cmd_right - cmd_left) * 0.5
        omega_pred = cl * (cmd_right - cmd_left) * 0.5 + tr * (cmd_right + cmd_left) * 0.5

        # position states in relation to kinetic states
        yaw_pred = (omega_pred)
        x_pred = (np.cos(yaw_pred) * vx_pred)
        y_pred = (np.sin(yaw_pred) * vx_pred)

        return [x_pred, y_pred, yaw_pred]

# Include basic utility functions here

# Motivation for introducing this fn:
#  1) simplify the import procedure in the main script by avoiding the need to explicitly import certain model
def model_generator(model_name = None):
    if model_name == None:
        rospy.logwarn('[model_library] model is not initialized'.format(model_name))
    if model_name == 'model1':
        return Model1()
    else:
        rospy.logwarn('[model_library] model name {} is not valid!'.format(model_name))


def simulate(model_object, t, x0, u, p):
    """
    Args:
        model_object: a model object as defined by model library.
        t (list) : time array for which the predictions will be made.
        x0 (list) : initial values of the states; x, y, yaw
        u (numpy.ndarray): 2*n array, whose first row is wheel_right_exec, and second row is wheel_left_exec. n is the number of time-steps.
        p (list): model parameters.

    Returns:
        x_sim (numpy.ndarray): 3*n array, containing history of state evolution.
    """
    x_sim = np.array(x0).reshape(3, 1) #record the evolution of states in an array

    for i in range(len(t) - 1):
        t_cur, t_next = t[i:i + 2] # prediction will be made in between two consecutive time steps, note that this does not require fixed time step.
        # one-step-ahead prediction
        sol = solve_ivp(fun=lambda t, x: model_object.model(t, x0, u[:,i], p), t_span=(t_cur, t_next), y0=x0, t_eval=[t_next])
        x_sim = np.hstack([x_sim, sol.y]) # add the output to the x history
        x0 = row(sol.y).tolist()[0] # current solution will be used as the initial step for the next step
    return x_sim

