from abc import ABCMeta, abstractmethod

import numpy as np
import rospy
from calibration.data_adapter_utils import *
from calibration.utils import reshape_x, get_param_from_config_file

used_model = get_param_from_config_file("model")


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

    def __init__(self):
        self.name = "kinematic_drive"
        self.param_ordered_list = ['dr', 'dl',
                                   'L']  # it is used to enforce an order (to avoid possible confusions) while importing params from YAML as bounds are imported from model always.
        self.model_params = {'dr': {'param_init_guess': 0.85, 'param_bounds': (None, None), 'search': (2.0, 0.4)},
                             'dl': {'param_init_guess': 0.85, 'param_bounds': (None, None), 'search': (2.0, 0.4)},
                             'L': {'param_init_guess': 0.055, 'param_bounds': (0.05, 0.06), 'search': (0.050, 0.010)}}
        # "search" is used for for brute-force cost function value evaluatiom: (magnitude of variation in both directions, decimation)
        rospy.loginfo("\nusing model type: [{}]".format(self.name))

    def model(self, t, x, u, p):
        # input commands + model params
        (cmd_right, cmd_left) = u
        (dr, dl, L) = p

        # kinetic states through actuation
        vx = (dr * cmd_right + dl * cmd_left)  # m/s
        omega = (dr * cmd_right - dl * cmd_left) / L  # rad/s

        # position states in relation to kinetic states
        rho_dot = vx  # m/s
        theta_dot = omega  # rad/s

        return [rho_dot, theta_dot]

    def simulate(self, t, x, u, p):
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
        x_sim = reshape_x(x0)

        for i in range(len(t) - 1):
            t_cur, t_next = t[
                            i:i + 2]  # prediction will be made in between two consecutive time steps, note that this does not require fixed time step.
            x0 = x[:, i]
            # one-step-ahead prediction
            sol = forward_euler_vel_to_pos(self.model, (t_next - t_cur), x0, u[:, i], p)
            a = np.hstack([x_sim, reshape_x(sol)])
            x_sim = a.copy()
        return x_sim

    def simulate_horizan(self, t, x0, u, p):
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
        x_sim = reshape_x(x0)
        for i in range(len(t) - 1):
            t_cur, t_next = t[
                            i:i + 2]  # prediction will be made in between two consecutive time steps, note that this does not require fixed time step.
            sol = forward_euler_vel_to_pos(self.model, (t_next - t_cur), x0, u[:, i], p)
            a = np.hstack([x_sim, reshape_x(sol)])
            x_sim = a.copy()
            x0 = sol
        return x_sim


class InputDependentKinematicDrive(BaseModelClass):

    def __init__(self, interval_count=1):
        self.name = "input_dependent_kinematic_drive"
        rospy.loginfo("\nusing model type: [{}]".format(self.name))

        # Note that we assume that the vehicle is movinf forward, hence d in [0,1].
        self.intervals = np.linspace(0, 1, interval_count + 1) # for interval_count = 1 returns array([0., 1.])
        self.param_ordered_list = self.generate_param_ordered_list()
        self.model_params = self.generate_model_params(self.param_ordered_list)

    def model(self, t, x, u, p):
        # input commands + model params
        (cmd_right, cmd_left) = u
        #(dr, dl, L) = p

        # extract the correct parameter depending on the input
        dr, dl, L = self.input_dependent_parameter_selector(u,p)

        # kinetic states through actuation
        vx = (dr * cmd_right + dl * cmd_left)  # m/s
        omega = (dr * cmd_right - dl * cmd_left) / L  # rad/s

        # position states in relation to kinetic states
        rho_dot = vx  # m/s
        theta_dot = omega  # rad/s

        return [rho_dot, theta_dot]

    def simulate(self, t, x, u, p):
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
        x_sim = reshape_x(x0)

        for i in range(len(t) - 1):
            t_cur, t_next = t[
                            i:i + 2]  # prediction will be made in between two consecutive time steps, note that this does not require fixed time step.
            x0 = x[:, i]
            # one-step-ahead prediction
            sol = forward_euler_vel_to_pos(self.model, (t_next - t_cur), x0, u[:, i], p)
            a = np.hstack([x_sim, reshape_x(sol)])
            x_sim = a.copy()
        return x_sim

    def simulate_horizan(self, t, x0, u, p):
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
        x_sim = reshape_x(x0)
        for i in range(len(t) - 1):
            t_cur, t_next = t[
                            i:i + 2]  # prediction will be made in between two consecutive time steps, note that this does not require fixed time step.
            sol = forward_euler_vel_to_pos(self.model, (t_next - t_cur), x0, u[:, i], p)
            a = np.hstack([x_sim, reshape_x(sol)])
            x_sim = a.copy()
            x0 = sol
        return x_sim

    def generate_param_ordered_list(self):
        param_ordered_list = []
        for i in range(len(self.intervals)-1):
            param_ordered_list.extend(["dr_" + str(i), "dl_" + str(i)])
        param_ordered_list.append("L")
        return param_ordered_list

    def generate_model_params(self, param_ordered_list):
        model_params = {}
        for i in np.arange(len(param_ordered_list)-1):
            model_params[param_ordered_list[i]] = {'param_init_guess': 0.85, 'param_bounds': (None, None)}
        model_params["L"] = {'param_init_guess': 0.055, 'param_bounds': (0.05, 0.06)}
        return model_params

    def input_dependent_parameter_selector(self, u, p):
        (cmd_right, cmd_left) = u

        # right drive constant
        bin_id_right = np.searchsorted(self.intervals, cmd_right)
        index_right = 2 * (bin_id_right-1)
        param_right_name = self.param_ordered_list[index_right]
        param_right_val = p[index_right]

        # left drive constant
        bin_id_left = np.searchsorted(self.intervals, cmd_left)
        index_left = 2 * (bin_id_right-1) + 1
        param_left_name = self.param_ordered_list[index_left]
        param_left_val = p[index_left]
        """
        print("intervals: {}".format(self.intervals))
        print(self.model_params)
        print("intervals: {}".format(self.intervals))
        print("p: {}".format(p))
        print("param_ordered_list: {}".format(self.param_ordered_list))
        
        print("cmd_right: {} \t index_right: {} ".format(cmd_right, index_right))
        print("Param Right Name: {} \t Param Right Value: {} ".format(param_right_name, param_right_val))
        print("cmd_left: {} \t index_left: {} ".format(cmd_left, index_left))
        print("Param Left Name: {} \t Param Left Value: {} ".format(param_left_name, param_left_val))
        """
        return param_right_val, param_left_val, p[-1]

class DynamicDrive(BaseModelClass):

    def __init__(self):
        self.name = "dynamic_drive"
        """
        self.param_ordered_list = ['u1', 'u2', 'u3', 'w1', 'w2', 'w3', 'u_alpha_r', 'u_alpha_l', 'w_alpha_r', 'w_alpha_l']
        self.model_params = {'u1': {'param_init_guess': 1, 'param_bounds': (None, None), 'search': (None, None)},
                             'u2': {'param_init_guess': 1, 'param_bounds': (None, None), 'search': (None, None)},
                             'u3': {'param_init_guess': 1, 'param_bounds': (None, None), 'search': (None, None)},
                             'w1': {'param_init_guess': 1, 'param_bounds': (None, None), 'search': (None, None)},
                             'w2': {'param_init_guess': 1, 'param_bounds': (None, None), 'search': (None, None)},
                             'w3': {'param_init_guess': 1, 'param_bounds': (None, None), 'search': (None, None)},
                             'u_alpha_r': {'param_init_guess': 1, 'param_bounds': (None, None), 'search': (None, None)},
                             'u_alpha_l': {'param_init_guess': 1, 'param_bounds': (None, None), 'search': (None, None)},
                             'w_alpha_r': {'param_init_guess': 1, 'param_bounds': (None, None), 'search': (None, None)},
                             'w_alpha_l': {'param_init_guess': 1, 'param_bounds': (None, None), 'search': (None, None)}}
        """
        self.param_ordered_list = ['u1', 'w1', 'u_alpha_r', 'u_alpha_l', 'w_alpha_r', 'w_alpha_l']

        self.model_params = {'u1': {'param_init_guess': 0, 'param_bounds': (None, None), 'search': (None, None)},
                             'w1': {'param_init_guess': 0, 'param_bounds': (None, None), 'search': (None, None)},
                             'u_alpha_r': {'param_init_guess': 0.5, 'param_bounds': (None, None),
                                           'search': (None, None)},
                             'u_alpha_l': {'param_init_guess': 0.5, 'param_bounds': (None, None),
                                           'search': (None, None)},
                             'w_alpha_r': {'param_init_guess': 0.5, 'param_bounds': (None, None),
                                           'search': (None, None)},
                             'w_alpha_l': {'param_init_guess': 0.5, 'param_bounds': (None, None),
                                           'search': (None, None)}}

        rospy.loginfo("\nusing model type: [{}]".format(self.name))

    def model(self, t, x_dot, U, p):
        V = col(np.array(U))  # input array
        (u, w) = x_dot
        # (u1, u2, u3, w1, w2, w3, u_alpha_r, u_alpha_l, w_alpha_r, w_alpha_l) = p
        (u1, w1, u_alpha_r, u_alpha_l, w_alpha_r, w_alpha_l) = p
        u2 = u3 = w2 = w3 = 0

        # Nonlinear Dynamics - autonomous response
        f_dynamic = np.array([
            [-u1 * u - u2 * w + u3 * w ** 2],
            [-w1 * w - w2 * u - w3 * u * w]
        ])

        # Input Matrix
        B = np.array([
            [u_alpha_r, u_alpha_l],
            [w_alpha_r, -w_alpha_l]
        ])
        # Forced response
        f_forced = np.matmul(B, V)

        # acceleration
        x_dot_dot = f_dynamic + f_forced

        # position states in relation to kinetic states
        rho_dot_dot = x_dot_dot[0].item()  # m/s
        theta_dot_dot = x_dot_dot[1].item()  # rad/s

        return [rho_dot_dot, theta_dot_dot]

    def simulate(self, t, x, x_dot, u, p):
        """
        Note that this function performs N step ahead propagation of the initial state
        for in one step ahead manner
        Args:
            model_object: a model object as defined by model library.
            t (list) : time array for which the predictions will be made.
            x_init (float) : initial position of the vehicle
            x_dot (list) : velocity of the vehicle
            u (numpy.ndarray): 2*n array, whose first row is wheel_right_exec, and second row is wheel_left_exec. n is the number of time-steps.
            p (list): model parameters.
        Returns:
            x_sim (numpy.ndarray): 3*n array, containing history of state evolution.
        """
        # create arrays to store simulation results
        x_sim = np.zeros(x.shape)
        x_dot_sim = np.zeros(x_dot.shape)

        # position initial condition is taken from the measurement
        x_sim[:, 0] = x[:, 0]

        for i in np.arange(1, len(t) - 1):
            # prediction will be made in between two consecutive time steps, note that this does not require fixed time step.
            t_cur, t_next = t[i:i + 2]
            x_cur = x[:, i]
            x_dot_prev = x_dot[:, i - 1]
            u_prev = u[:, i - 1]
            # one-step-ahead prediction
            x_next, x_dot_cur = forward_euler_acc_to_pos(self.model, (t_next - t_cur), x_cur, x_dot_prev, u_prev, p)
            # store the results
            x_sim[:, i + 1] = x_next
            x_dot_sim[:, i] = x_dot_cur

        return x_sim

    def simulate_horizan(self, t, x, x_dot, u, p):
        """
        Note that this function performs N step ahead propagation of the initial state
        for in one step ahead manner
        Args:
            model_object: a model object as defined by model library.
            t (list) : time array for which the predictions will be made.
            x_init (float) : initial position of the vehicle
            x_dot (list) : velocity of the vehicle
            u (numpy.ndarray): 2*n array, whose first row is wheel_right_exec, and second row is wheel_left_exec. n is the number of time-steps.
            p (list): model parameters.
        Returns:
            x_sim (numpy.ndarray): 3*n array, containing history of state evolution.
        """
        # create arrays to store simulation results
        x_sim = np.zeros(x.shape)
        x_dot_sim = np.zeros(x_dot.shape)

        # position initial condition is taken from the measurement
        x_sim[:, 0:2] = x[:, 0:2]  # first two points
        x_cur = x[:, 1]

        for i in np.arange(1, len(t) - 1):
            # prediction will be made in between two consecutive time steps, note that this does not require fixed time step.
            t_cur, t_next = t[i:i + 2]
            x_dot_prev = x_dot[:, i - 1]
            u_prev = u[:, i - 1]
            # one-step-ahead prediction
            x_next, x_dot_cur = forward_euler_acc_to_pos(self.model, (t_next - t_cur), x_cur, x_dot_prev, u_prev, p)
            # store the results
            x_cur = x_next
            x_sim[:, i + 1] = x_next
            x_dot_sim[:, i] = x_dot_cur
        return x_sim


# Include basic utility functions here

# Motivation for introducing this fn:
#  1) simplify the import procedure in the main script by avoiding the need to explicitly import certain model
def model_generator(model_name=None):
    if model_name == None:
        rospy.logwarn('[model_library] model is not initialized'.format(model_name))
    elif model_name == 'kinematic_drive':
        return KinematicDrive()
    elif model_name == 'dynamic_drive':
        return DynamicDrive()
    elif model_name == "input_dependent_kinematic_model":
        return InputDependentKinematicDrive()
    else:
        rospy.logwarn('[model_library] model name {} is not valid!'.format(model_name))


def forward_euler_acc_to_pos(model, dt, x_cur, x_dot_prev, u_prev, p):
    """ forward euler only for dynamic case only supports polar coordinate"""
    x_next = [0, 0]
    x_dot_cur = [0, 0]

    ds_prev = model(dt, x_dot_prev, u_prev, p)
    for i, s in enumerate(ds_prev):
        x_dot_cur[i] = x_dot_prev[i] + ds_prev[i] * dt
        x_next[i] = x_cur[i] + dt * x_dot_cur[i]
    return x_next, x_dot_cur


def forward_euler_vel_to_pos(model, dt, x_cur, u_cur, p_cur):
    x_next = [0, 0]

    ds = model(dt, x_cur, u_cur, p_cur)
    for i, s in enumerate(ds):
        x_next[i] = x_cur[i] + ds[i] * dt
    return x_next


if __name__ == '__main__':
    t = 0
    x = 0
    u_r = 1
    u_l = 1
    u = (u_r, u_l)
    p = [1, 1, 1, 1, 1, 1, 1, 1, 0.5]
    ikd = InputDependentKinematicDrive(interval_count=4)
    ikd.model(t,x,u,p)
    print("selcuk")
