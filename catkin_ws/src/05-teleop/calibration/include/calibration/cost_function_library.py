import numpy as np
import rospy
from utils import get_param_from_config_file

used_model = get_param_from_config_file("model")

""" Metrics 

input: 1D arrays of measurements and predictions 
output: scalar measure value """


def SE(x, x_sim):
    """ Root mean square"""
    y = se(x, x_sim)
    return np.mean(y)


def RMSE(x, x_sim):
    """ Root mean square"""
    return rmse(x, x_sim)


def AsRMSE(x, x_sim):
    """ Amplitude scaled root mean square"""
    y = rmse(x, x_sim) / (max(x) - min(x))
    return np.mean(y)


'''
leads faulty results as mean can be negative
def MsRMSE(x, x_sim):
    """ Mean scaled root mean square"""
    y = rmse(x, x_sim) / np.mean(x)
    return np.mean(y)
'''


def hampel(x, x_sim, delta):
    """
    hampel's loss
    delta may be interpreted as the precision requested.
    """
    y = np.where(np.abs(x - x_sim) < delta, 0.5 * ((x - x_sim) ** 2), delta * np.abs(x - x_sim) - 0.5 * (delta ** 2))
    return np.sum(y)


# Helper functions
def rmse(x, x_sim):
    """ root mean square error"""
    return np.sqrt(np.mean(se(x, x_sim)))


def se(x, x_sim):
    """ square error"""
    return np.power((x - x_sim), 2)


def calculate_cost(x, x_sim, metric_eval, p=None):
    cost_val = 0.0


    rho = metric_eval(x[0, :], x_sim[0, :])
    yaw = metric_eval(x[1, :], x_sim[1, :])

    if used_model == "kinematic_drive" or used_model == "input_dependent_kinematic_drive" or used_model == "gain_trim":
        # use only state values
        obj_cost = rho + yaw
    elif used_model == "dynamic_drive":
        # add regularizer to enforce motor similarity (to avoid over-fitting)
        reg_delta_u = 0.00
        reg_delta_w = 0.007

        u_alpha_r = p[6]
        u_alpha_l = p[7]
        w_alpha_r = p[8]
        w_alpha_l = p[9]

        delta_u = (u_alpha_r - u_alpha_l) ** 2 * reg_delta_u
        delta_w = (w_alpha_r - w_alpha_l) ** 2 * reg_delta_w
        obj_cost = rho + yaw + delta_u + delta_w
        """
        alpha_r = p[6]
        alpha_l = p[7]
        obj_cost = (alpha_r - alpha_l) ** 2 * 0.1 + rho + yaw * 10
        """
    return obj_cost


def metric_selector(cost_name):
    if cost_name == "SE":
        return SE
    elif cost_name == "RMSE":
        return RMSE
    elif cost_name == 'AsRMSE':
        return AsRMSE
    else:
        rospy.logwarn('undefined cost function is requested, see cost_fn_library for const function definitions')


if __name__ == '__main__':
    x_sim = np.arange(-10, 10, 1)
    x = np.zeros(x_sim.shape)
    delta = 1
    y = hampel(x, x_sim, delta)
