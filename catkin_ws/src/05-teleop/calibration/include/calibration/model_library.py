import rospy
import numpy as np

from abc import ABCMeta, abstractmethod
from scipy.integrate import solve_ivp
from calibration.data_adapter_utils import *


class BaseModelClass(object):
    __metaclass__ = ABCMeta

    @abstractmethod
    def __init__(self, measurement_coordinate_system):

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

    def get_param_initial_guess_dict(self):
        param_initial_guess_dict = {}
        for param_key in self.model_params.keys():
            param_initial_guess_dict[param_key] = self.model_params[param_key]['param_init_guess']
        return param_initial_guess_dict
    def get_param_bounds_list(self):
        param_bounds = []
        for param in self.param_ordered_list:
            param_bounds.append(self.model_params[param]['param_bounds'])
        return param_bounds
    def get_model_param_list(self):
        return self.model_params.keys()

    def set_param(self, param_name, param_val):
        setattr(self, param_name, param_val)

class KinematicDrive(BaseModelClass):

    def __init__(self, measurement_coordinate_system):
        self.name = "kinematic_drive"
        self.param_ordered_list = ['dr', 'dl', 'L'] # it is used to enforce an order (to avoid possible confusions) while importing params from YAML as bounds are imported from model always.
        self.model_params = {'dr': {'param_init_guess':0.85, 'param_bounds': (None, None), 'search': (2.0, 0.4)},
                             'dl': {'param_init_guess':0.85, 'param_bounds': (None, None), 'search': (2.0, 0.4)},
                             'L' : {'param_init_guess':0.055, 'param_bounds': (0.05, 0.06), 'search': (0.050, 0.010)}}
        # "search" is used for for brute-force cost function value evaluatiom: (magnitude of variation in both directions, decimation)
        self.measurement_coordinate_system = measurement_coordinate_system
        rospy.loginfo("\nusing model type: [{}]".format(self.name))

    def model(self, t, x, u, p):
        # input commands + model params
        (cmd_right, cmd_left) = u
        (dr, dl, L) = p

        # kinetic states through actuation
        vx = (dr * cmd_right + dl * cmd_left) # m/s
        omega = (dr * cmd_right - dl * cmd_left) / L # rad/s

        if self.measurement_coordinate_system == 'cartesian':
            [x, y, theta] = x
            # position states in relation to kinetic states
            x_dot = (np.cos(theta) * vx)
            y_dot = (np.sin(theta) * vx)
            theta_dot = (omega) # rad/s

            return [x_dot, y_dot, theta_dot]
        elif self.measurement_coordinate_system == 'polar':
            # position states in relation to kinetic states
            rho_dot = vx # m/s
            theta_dot = omega # rad/s

            return [rho_dot, theta_dot]


class DynamicDrive(BaseModelClass):

    def __init__(self, measurement_coordinate_system):
        self.name = "dynamic_drive"
        self.param_ordered_list = ['u1', 'u2', 'u3', 'w1', 'w2', 'w3', 'u_alpha_r', 'u_alpha_l', 'w_alpha_r', 'w_alpha_l']
        self.model_params = {'u1': {'param_init_guess': 1, 'param_bounds': (None, None), 'search': (None, None)},
                             'u2': {'param_init_guess': 1, 'param_bounds': (None, None), 'search': (None, None)},
                             'u3' : {'param_init_guess': 1, 'param_bounds': (None, None), 'search': (None, None)},
                             'w1': {'param_init_guess': 1, 'param_bounds': (None, None), 'search': (None, None)},
                             'w2': {'param_init_guess': 1, 'param_bounds': (None, None), 'search': (None, None)},
                             'w3': {'param_init_guess': 1, 'param_bounds': (None, None), 'search': (None, None)},
                             'u_alpha_r': {'param_init_guess': 1, 'param_bounds': (None, None), 'search': (None, None)},
                             'u_alpha_l': {'param_init_guess': 1, 'param_bounds': (None, None), 'search': (None, None)},
                             'w_alpha_r': {'param_init_guess': 1, 'param_bounds': (None, None), 'search': (None, None)},
                             'w_alpha_l': {'param_init_guess': 1, 'param_bounds': (None, None), 'search': (None, None)}}
        self.measurement_coordinate_system = measurement_coordinate_system
        rospy.loginfo("\nusing model type: [{}]".format(self.name))

    def model(self, t, x_dot, u, p):
        V = col(np.array(u)) # input array
        (u, w) = x_dot
        (u1, u2, u3, w1, w2, w3, u_alpha_r, u_alpha_l, w_alpha_r, w_alpha_l) = p

        # Nonlinear Dynamics - autonomous response
        f_dynamic = np.array([
        [-u1 * u - u2 * w + u3 * w ** 2],
        [-w1 * u - w2 * w - w3 * u * w]
        ])

        # Input Matrix - forced response
        B = np.array([
            [u_alpha_r, u_alpha_l],
            [w_alpha_r, -w_alpha_l]
        ])
        f_forced = np.matmul(B, V)

        # acceleration
        x_dot_dot = f_dynamic + f_forced

        if self.measurement_coordinate_system == 'cartesian':
            raise NotImplementedError
        elif self.measurement_coordinate_system == 'polar':
            # position states in relation to kinetic states
            rho_dot_dot = np.asscalar(x_dot_dot[0]) # m/s
            theta_dot_dot = np.asscalar(x_dot_dot[1] * 180 / np.pi) # deg/s
            return [rho_dot_dot, theta_dot_dot]


# Include basic utility functions here

# Motivation for introducing this fn:
#  1) simplify the import procedure in the main script by avoiding the need to explicitly import certain model
def model_generator(model_name = None, measurement_coordinate_system = 'cartesian'):
    if model_name == None:
        rospy.logwarn('[model_library] model is not initialized'.format(model_name))
    elif model_name == 'kinematic_drive':
        return KinematicDrive(measurement_coordinate_system)
    elif model_name == 'dynamic_drive':
        return DynamicDrive(measurement_coordinate_system)
    else:
        rospy.logwarn('[model_library] model name {} is not valid!'.format(model_name))


def simulate_horizan(model_object, t, x0, u, p):
    """
    Note that this function performs N step ahead propagation of the initial state
    for given input sequence u.
    Args:
        model_object: a model object as defined by model library.
        t (list) : time array for which the predictions will be made.
        x0 (list) : initial values of the states; x, y, yaw
        u (numpy.ndarray): 2*n array, whose first row is wheel_right_exec, and second row is wheel_left_exec. n is the number of time-steps.
        p (list): model parameters.

    Returns:
        x_sim (numpy.ndarray): 3*n array, containing history of state evolution.
    """
    if model_object.measurement_coordinate_system == 'cartesian':
        x_sim = np.array(x0).reshape(3, 1) #record the evolution of states in an array
    elif model_object.measurement_coordinate_system == 'polar':
        x_sim = np.array(x0).reshape(2, 1)  # record the evolution of states in an array

    for i in range(len(t) - 1):
        t_cur, t_next = t[i:i + 2] # prediction will be made in between two consecutive time steps, note that this does not require fixed time step.
        # one-step-ahead prediction
        """
        sol = solve_ivp(fun=lambda t, x: model_object.model(t, x0, u[:,i], p), t_span=(t_cur, t_next), y0=x0, t_eval=[t_next])
        x_sim = np.hstack([x_sim, sol.y]) # add the output to the x history
        x0 = row(sol.y).tolist()[0] # current solution will be used as the initial step for the next step
        """
        sol = forwardEuler(model_object, (t_next - t_cur), x0, u[:,i], p)

        if model_object.measurement_coordinate_system == 'cartesian':
            b = np.array(sol).reshape(3, 1)  # record the evolution of states in an array
        elif model_object.measurement_coordinate_system == 'polar':
            b = np.array(sol).reshape(2, 1)# record the evolution of states in an array

        a = np.hstack([x_sim, b])
        x_sim = a.copy()
        x0 = sol

    return x_sim


def simulate(model_object, t, x, u, p):
    """
    Note that this function performs N step ahead propagation of the initial state
    for in one step ahead manner
    Args:
        model_object: a model object as defined by model library.
        t (list) : time array for which the predictions will be made.
        x (list) : measured states; x, y, yaw
        u (numpy.ndarray): 2*n array, whose first row is wheel_right_exec, and second row is wheel_left_exec. n is the number of time-steps.
        p (list): model parameters.

    Returns:
        x_sim (numpy.ndarray): 3*n array, containing history of state evolution.
    """
    x0 = x[:, 0]
    if model_object.measurement_coordinate_system == 'cartesian':
        x_sim = np.array(x0).reshape(3, 1) #record the evolution of states in an array
    elif model_object.measurement_coordinate_system == 'polar':
        x_sim = np.array(x0).reshape(2, 1)  # record the evolution of states in an array


    for i in range(len(t) - 1):
        t_cur, t_next = t[i:i + 2] # prediction will be made in between two consecutive time steps, note that this does not require fixed time step.
        x0 = x[:,i]
        # one-step-ahead prediction
        """
        sol = solve_ivp(fun=lambda t, x: model_object.model(t, x0, u[:,i], p), t_span=(t_cur, t_next), y0=x0, t_eval=[t_next])
        x_sim = np.hstack([x_sim, sol.y]) # add the output to the x history
        x0 = row(sol.y).tolist()[0] # current solution will be used as the initial step for the next step
        """
        sol = forwardEuler(model_object, (t_next - t_cur), x0, u[:,i], p)

        if model_object.measurement_coordinate_system == 'cartesian':
            b = np.array(sol).reshape(3, 1)  # record the evolution of states in an array
        elif model_object.measurement_coordinate_system == 'polar':
            b = np.array(sol).reshape(2, 1)# record the evolution of states in an array

        a = np.hstack([x_sim, b])
        x_sim = a.copy()

    return x_sim


def forwardEuler(model_object, dt, x_cur, u_cur, p_cur):
    if model_object.measurement_coordinate_system == 'cartesian':
        x_next = [0, 0, 0]
    elif model_object.measurement_coordinate_system == 'polar':
        x_next = [0, 0]

    ds = model_object.model(dt, x_cur, u_cur, p_cur)
    for i, s in enumerate(ds):
        x_next[i] = x_cur[i] + ds[i] * dt
    return x_next


if __name__ == '__main__':
    from plotting_utils import multiplot
    # Testing model and simulate functions
    dd =model_generator('dynamic_drive', 'polar')

    t = np.arange(0,10,1)
    x0 = [0, 0]
    u = np.vstack([np.ones(np.size(t)) * 1.0, np.ones(np.size(t)) * 1.0])
    p = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1]

    x_sim = simulate_horizan(dd, t, x0, u, p)
    multiplot(states_list=[x_sim], time_list=[t], experiment_name_list=["simulation"], plot_title='dynamics', save=False)
    print("selcuk")
