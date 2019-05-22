from abc import ABCMeta, abstractmethod

import numpy as np
import rospy
from calibration.data_adapter_utils import *
from calibration.utils import reshape_x, get_param_from_config_file, get_valid_drive_constants
from scipy.optimize import fsolve
from scipy.interpolate import interp1d
import time

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

class GainTrim(BaseModelClass):

    def __init__(self):
        self.name = "gain_trim"
        # it is used to enforce an order (to avoid possible confusions) while importing params from YAML as bounds are imported from model always.
        self.param_ordered_list = ['baseline', 'gain', 'k', 'limit', 'radius', 'trim']
        self.model_params = {'baseline': {'param_init_guess': 0.1, 'param_bounds': (None, None), 'search': (2.0, 0.4)},
                             'gain': {'param_init_guess': 1.0, 'param_bounds': (None, None), 'search': (2.0, 0.4)},
                             'k': {'param_init_guess': 27, 'param_bounds': (None, None), 'search': (2.0, 0.4)},
                             'limit': {'param_init_guess': 1.0, 'param_bounds': (None, None), 'search': (2.0, 0.4)},
                             'radius': {'param_init_guess': 0.0318, 'param_bounds': (None, None), 'search': (2.0, 0.4)},
                             'trim': {'param_init_guess': -0.05, 'param_bounds': (None, None), 'search': (2.0, 0.4)}
                             }
        # "search" is used for for brute-force cost function value evaluatiom: (magnitude of variation in both directions, decimation)
        rospy.loginfo("\nusing model type: [{}]".format(self.name))

    def model(self, t, x, u, p):
        # input commands + model params
        (cmd_right, cmd_left) = u

        # model params
        k = self.model_params["k"]['param_init_guess']
        gain = self.model_params["gain"]['param_init_guess']
        trim = self.model_params["trim"]['param_init_guess']
        radius = self.model_params["radius"]['param_init_guess']
        baseline = self.model_params["baseline"]['param_init_guess']

        # compute duty cycle gain
        k_r = k_l = k
        k_r_inv = (gain + trim) / k_r
        k_l_inv = (gain - trim) / k_l

        # Conversion from motor duty to motor rotation rate
        omega_r = cmd_right / k_r_inv
        omega_l = cmd_left / k_l_inv

        # Compute linear and angular velocity of the platform
        v = (radius * omega_r + radius * omega_l) / 2.0
        omega = (radius * omega_r - radius * omega_l) / baseline

        rho_dot = v
        theta_dot = omega

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

class KinematicDrive(BaseModelClass):

    def __init__(self):
        self.name = "kinematic_drive"
        self.param_ordered_list = ['dr', 'dl',
                                   'L']  # it is used to enforce an order (to avoid possible confusions) while importing params from YAML as bounds are imported from model always.
        self.model_params = {'dr': {'param_init_guess': 0.85, 'param_bounds': (None, None), 'search': (2.0, 0.4)},
                             'dl': {'param_init_guess': 0.85, 'param_bounds': (None, None), 'search': (2.0, 0.4)},
                             'L': {'param_init_guess': 0.0522, 'param_bounds': (0.05, 0.06), 'search': (0.050, 0.010)}}
        # "search" is used for for brute-force cost function value evaluatiom: (magnitude of variation in both directions, decimation)
        rospy.loginfo("\nusing model type: [{}]".format(self.name))

    def model(self, t, x, u, p):
        # input commands + model params
        (cmd_right, cmd_left) = u
        (dr, dl, L) = p
        L = 0.0522

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

        # Note that we assume that the vehicle is moving forward, hence d in [0,1].
        self.interval_count = interval_count
        self.intervals = np.linspace(0, 1, self.interval_count + 1) # for interval_count = 1 returns array([0., 1.])
        self.param_ordered_list = self.generate_param_ordered_list()
        self.model_params = self.generate_model_params(self.param_ordered_list)

        # keep count of the actuated intervals, i.e. where drive constants are optimized and non-default
        self.right_wheel_active_intervals = []
        self.left_wheel_active_intervals = []

    def model(self, t, x, u, p):
        # input commands + model params
        (cmd_right, cmd_left) = u

        # extract the correct parameters (dr, dl) depending on the input
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
            # prediction will be made in between two consecutive time steps, note that this does not require fixed time step.
            t_cur, t_next = t[i:i + 2]
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
            param_ordered_list.extend(["dr_" + str(i+1), "dl_" + str(i+1)])
        param_ordered_list.append("L")
        return param_ordered_list

    def generate_model_params(self, param_ordered_list):
        model_params = {}
        for i in np.arange(len(param_ordered_list)-1):
            model_params[param_ordered_list[i]] = {'param_init_guess': -0.000001, 'param_bounds': (None, None)}
        model_params["L"] = {'param_init_guess': 0.055, 'param_bounds': (0.05, 0.06)}
        return model_params

    def input_dependent_parameter_selector(self, u, p):
        mode = "validate"
        if mode == "train":
            (cmd_right, cmd_left) = u

            # right drive constant
            bin_id_right = np.searchsorted(self.intervals, cmd_right) - 1
            index_right = 2 * (bin_id_right-1)
            param_right_name = self.param_ordered_list[index_right]
            param_right_val = p[index_right]

            if bin_id_right not in self.right_wheel_active_intervals:
                self.right_wheel_active_intervals.append(bin_id_right)

            # left drive constant
            bin_id_left = np.searchsorted(self.intervals, cmd_left) - 1
            index_left = 2 * (bin_id_right-1) + 1
            param_left_name = self.param_ordered_list[index_left]
            param_left_val = p[index_left]

            if bin_id_left not in self.left_wheel_active_intervals:
                self.left_wheel_active_intervals.append(bin_id_left)

            #print("intervals: {}".format(self.intervals))
            #print(self.model_params)
            #print("intervals: {}".format(self.intervals))
            #print("p: {}".format(p))
            #print("param_ordered_list: {}".format(self.param_ordered_list))

            #print("cmd_right: {} \t index_right: {} ".format(cmd_right, index_right))
            #print("Param Right Name: {} \t Param Right Value: {} ".format(param_right_name, param_right_val))
            #print("cmd_left: {} \t index_left: {} ".format(cmd_left, index_left))
            #print("Param Left Name: {} \t Param Left Value: {} ".format(param_left_name, param_left_val))

            return param_right_val, param_left_val, p[-1]
        elif mode == "validate":
            first = True
            if first == True: # on the first call fit to motor speed-drive constant curve
                duty_cycle_right, drive_constant_right, duty_cycle_left, drive_constant_left, L = get_valid_drive_constants("mete", self)
                self.fit_to_exponential_model_drive_constants(duty_cycle_right, drive_constant_right, duty_cycle_left, drive_constant_left)
                first = False
            (cmd_right, cmd_left) = u
            p_r = self.exponential_decay(cmd_right, self.popt_right[0], self.popt_right[1], self.popt_right[2])
            p_l = self.exponential_decay(cmd_left, self.popt_left[0], self.popt_left[1], self.popt_left[2])

            return p_r, p_l, L


    """
    def linear_interp_drive_constants(self,duty_cycle_right, drive_constant_right, duty_cycle_left, drive_constant_left):
        self.right_fit = interp1d(duty_cycle_right, drive_constant_right, fill_value='extrapolate')
        self.left_fit = interp1d(duty_cycle_left, drive_constant_left, fill_value='extrapolate')

        return self.right_fit, self.left_fit
    """
    def fit_to_exponential_model_drive_constants(self,duty_cycle_right, drive_constant_right, duty_cycle_left, drive_constant_left):
        from scipy.optimize import curve_fit
        self.popt_right, pcov_right = curve_fit(self.exponential_decay, duty_cycle_right, drive_constant_right,  p0=(1 ,7, 0.35))
        self.popt_left, pcov_left = curve_fit(self.exponential_decay, duty_cycle_left, drive_constant_left, p0=(1 ,7, 0.35))

        return self.popt_right, self.popt_left

    @staticmethod
    def exponential_decay(x, a, c, d):
        return a * np.exp(-c * x) + d

    def inverse_model(self, v_ref = None, w_ref= None, V_r_init = None, V_l_init = None, semi_wheel_distance=None):
        input0 = np.array([V_r_init, V_l_init])
        start_time = time.time()
        sol = fsolve(self.inv_model, input0, args=(v_ref, w_ref, semi_wheel_distance))
        rospy.logwarn("solution found: {} in {} seconds".format(sol, time.time() - start_time))
        return sol

    def inv_model(self, input, v_ref, w_ref, L):
        v_r = input[0]
        v_l = input[1]

        f = np.zeros(2)
        """
        # linear_interp_drive_constants
        f[0] = v_ref - (self.right_fit(v_r) * v_r + self.left_fit(v_l) * v_l)
        f[1] = w_ref - (self.right_fit(v_r) * v_r - self.left_fit(v_l) * v_l) / L
        """
        p_r = self.exponential_decay(v_r, self.popt_right[0], self.popt_right[1], self.popt_right[2])
        p_l = self.exponential_decay(v_l, self.popt_left[0], self.popt_left[1], self.popt_left[2])

        f[0] = v_ref - (p_r * v_r + p_l * v_l)
        f[1] = w_ref - (p_r * v_r - p_l * v_l) / L

        return f

    def forward_model(self, input, v_ref, w_ref, L):
        v_r = input[0]
        v_l = input[1]

        f = np.zeros(2)
        """
        # linear_interp_drive_constants
        f[0] = v_ref - (self.right_fit(v_r) * v_r + self.left_fit(v_l) * v_l)
        f[1] = w_ref - (self.right_fit(v_r) * v_r - self.left_fit(v_l) * v_l) / L
        """
        p_r = self.exponential_decay(v_r, self.popt_right[0], self.popt_right[1], self.popt_right[2])
        p_l = self.exponential_decay(v_l, self.popt_left[0], self.popt_left[1], self.popt_left[2])

        f[0] = v_ref - (p_r * v_r + p_l * v_l)
        f[1] = w_ref - (p_r * v_r - p_l * v_l) / L

        return f
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
        #u2 = w2 = u3 = w3 = 0
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

    def inverse_model(self, x_dot_prev, x_dot_des, dt, p):
        """

        Args:
            x_dot_prev: (u_prev, w_prev)
            x_dot_des: (u_des, w_des)
            dt: float
            p: param_list as defined by "param_ordered_list"

        Returns:
            u_r, u_l: float duty cycles
        """

        # unpack velocities
        (u, w) = x_dot_prev
        (u_des, w_des) = x_dot_des

        # unpack params
        (u1, u2, u3, w1, w2, w3, u_alpha_r, u_alpha_l, w_alpha_r, w_alpha_l) = p

        # Nonlinear Dynamics - autonomous response
        f_dynamic = np.array([
            [-u1 * u - u2 * w + u3 * w ** 2],
            [-w1 * w - w2 * u - w3 * u * w]
        ])
        #print("[{}] f_dynamics:{} shape: {}".format("model_library", f_dynamic, f_dynamic.shape))

        # Input Matrix
        B = np.array([
            [u_alpha_r, u_alpha_l],
            [w_alpha_r, -w_alpha_l]
        ])

        x_dot_prev = col(np.array(x_dot_prev))
        x_dot_des = col(np.array(x_dot_des))
        #print("[{}] x_dot_des:{} shape: {}".format("model_library", x_dot_des, x_dot_des.shape))
        x_dot_del = (x_dot_des - x_dot_prev) / dt
        #print("[{}] x_dot_del:{} shape: {}".format("model_library", x_dot_del, x_dot_del.shape))
        x_dot_req_input = x_dot_del - f_dynamic
        #print("[{}] x_dot_req_input:{} shape: {}".format("model_library", x_dot_req_input, x_dot_req_input.shape))
        #print(B.shape, x_dot_req_input.shape)
        V = np.matmul(B, x_dot_req_input)
        #print("[{}] V:{} shape: {}".format("model_library", V, V.shape))
        u_r, u_l = V

        rospy.loginfo("[{}] desired:\nv: {} w: {} \nprev: v_prev: {} w_prev: {} \ninput d: u_r: {} u_l: {}"
                      .format("model_library", u_des, w_des, u, w, u_r, u_l))

        #print("\nXXXXXXXXXXXX")
        return u_r, u_l

# Include basic utility functions here

# Motivation for introducing this fn:
#  1) simplify the import procedure in the main script by avoiding the need to explicitly import certain model
def model_generator(model_name=None, **kwargs):
    if model_name == None:
        rospy.logwarn('[model_library] model is not initialized'.format(model_name))
    elif model_name == 'kinematic_drive':
        return KinematicDrive()
    elif model_name == 'dynamic_drive':
        return DynamicDrive()
    elif model_name == "input_dependent_kinematic_drive":
        interval_count = get_param_from_config_file("interval_count")
        return InputDependentKinematicDrive(interval_count=interval_count)
    elif model_name == "gain_trim":
        return GainTrim()
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
