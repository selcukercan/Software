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

'''
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

'''

class DynamicDrive(BaseModelClass):

    def __init__(self):
        self.name = "dynamic_drive"

        """
        self.param_ordered_list = ['u1', 'u2', 'u3', 'w1', 'w2', 'w3', 'alpha_r', 'alpha_l']
        self.model_params = {'u1': {'param_init_guess': 1, 'param_bounds': (0, None), 'search': (None, None)},
                             'u2': {'param_init_guess': 0, 'param_bounds': (None, None), 'search': (None, None)},
                             'u3': {'param_init_guess': 0, 'param_bounds': (-0.05, 0.05), 'search': (None, None)},
                             'w1': {'param_init_guess': 1, 'param_bounds': (0, None), 'search': (None, None)},
                             'w2': {'param_init_guess': 0, 'param_bounds': (None, None), 'search': (None, None)},
                             'w3': {'param_init_guess': 0, 'param_bounds': (None, None), 'search': (None, None)},
                             'alpha_r': {'param_init_guess': 1, 'param_bounds': (0, None), 'search': (None, None)},
                             'alpha_l': {'param_init_guess': 1, 'param_bounds': (0, None), 'search': (None, None)}}
        """
        self.param_ordered_list = ['u1', 'u2', 'u3', 'w1', 'w2', 'w3', 'u_alpha_r', 'u_alpha_l', 'w_alpha_r', 'w_alpha_l']
        self.model_params = {'u1': {'param_init_guess': 1, 'param_bounds': (0, None), 'search': (None, None)},
                             'u2': {'param_init_guess': 0, 'param_bounds': (None, None), 'search': (None, None)},
                             'u3': {'param_init_guess': 0, 'param_bounds': (-0.05, 0.05), 'search': (None, None)},
                             'w1': {'param_init_guess': 1, 'param_bounds': (0, None), 'search': (None, None)},
                             'w2': {'param_init_guess': 0, 'param_bounds': (None, None), 'search': (None, None)},
                             'w3': {'param_init_guess': 0, 'param_bounds': (None, None), 'search': (None, None)},
                             'u_alpha_r': {'param_init_guess': 1, 'param_bounds': (0, None), 'search': (None, None)},
                             'u_alpha_l': {'param_init_guess': 1, 'param_bounds': (0, None), 'search': (None, None)},
                             'w_alpha_r': {'param_init_guess': 1, 'param_bounds': (0, None), 'search': (None, None)},
                             'w_alpha_l': {'param_init_guess': 1, 'param_bounds': (0, None), 'search': (None, None)}}


        """
        #self.param_ordered_list = ['u1', 'w1', 'c1', 'c2', 'L1', 'L2', 'm', 'Iz']
        self.param_ordered_list = ['u1', 'u2', 'u3', 'w1', 'w2', 'w3', 'c1', 'c2', 'e1', 'e2', 'm']

        self.model_params = {
                             'u1': {'param_init_guess': 0, 'param_bounds': (None, None)},
                             'u2': {'param_init_guess': 0, 'param_bounds': (None, None)},
                             'u3': {'param_init_guess': 0, 'param_bounds': (None, None)},
                             'w1': {'param_init_guess': 0, 'param_bounds': (None, None)},
                             'w2': {'param_init_guess': 0, 'param_bounds': (None, None)},
                             'w3': {'param_init_guess': 0, 'param_bounds': (None, None)},
                             'c1': {'param_init_guess': 1, 'param_bounds': (0, None)},
                             'c2': {'param_init_guess': 1, 'param_bounds': (0, None)},
                             'e1': {'param_init_guess': 1, 'param_bounds': (0, None)},
                             'e2': {'param_init_guess': 1, 'param_bounds': (0, None)},
                             'm': {'param_init_guess': 0.5, 'param_bounds': (0.4, 1.5)}
                             }
        """
        rospy.loginfo("\nusing model type: [{}]".format(self.name))

    def model(self, t, x_dot, U, p):
        V = col(np.array(U))  # input array
        (u, w) = x_dot
        #(u1, u2, u3, w1, w2, w3, alpha_r, alpha_l) = p
        (u1, u2, u3, w1, w2, w3, u_alpha_r, u_alpha_l, w_alpha_r, w_alpha_l) = p
        #(u1, w1, u_alpha_r, u_alpha_l, w_alpha_r, w_alpha_l) = p
        #(u1, w1, c1, c2, L1, L2, m, Iz) = p
        #(u1, u2, u3, w1, w2, w3, c1, c2, e1, e2, m) = p
        #u_alpha_r = w_alpha_r = alpha_r
        #u_alpha_l = w_alpha_l = alpha_l

        """
        u_alpha_r = c1 / m
        u_alpha_l = c2 / m
        w_alpha_r = e1 * c1
        w_alpha_l = e2 * c2
        """

        #w_alpha_r = (L1/Iz) * c1
        #w_alpha_l = (L2/Iz) * c2



        # Nonlinear Dynamics - autonomous response

        f_dynamic = np.array([
            [-u1 * u - u2 * w - u3 * w ** 2],
            [-w1 * w - w2 * u + w3 * u * w]
        ])
        """
        f_dynamic = np.array([
            [-u1 * u - u2 * w],
            [-w1 * w - w2 * u]
        ])
        """
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
    from plotting_utils import multiplot
    from calibration.utils import work_space_settings

    work_space_settings()
    # Testing model and simulate functions
    dd = model_generator('dynamic_drive', 'polar')

    t_beg = 0
    t_end = 4
    t_step = 1

    t = np.arange(t_beg, t_end, t_step)

    x_dot = np.array([[0, 1, 1, 1], [0, 0, 0, 1]])
    u = np.vstack([np.ones(np.size(t)) * 1.0, np.ones(np.size(t)) * 1.0])
    p = [0.5, 0, 0, 0.5, 0, 0, 1, 1, 1, 1]

    x_sim = dd.simulate(t, x_dot, u, p)

    multiplot(states_list=[x_sim], time_list=[t], experiment_name_list=["simulation"], plot_title='dynamics',
              save=False)
    print("selcuk")
