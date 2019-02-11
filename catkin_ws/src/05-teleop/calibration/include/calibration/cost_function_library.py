import rospy
import numpy as np
from utils import get_param_from_config_file


measurement_coordinate_frame = get_param_from_config_file("express_measurements_in")

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
leads faulty resuls as mean can be negative
def MsRMSE(x, x_sim):
    """ Mean scaled root mean square"""
    y = rmse(x, x_sim) / np.mean(x)
    return np.mean(y)
'''

def hampel(x, x_sim, delta):
    """ hampel's loss
    delta may be interpreted as the precision requested.
    """
    y = np.where(np.abs(x-x_sim) < delta , 0.5*((x-x_sim)**2), delta*np.abs(x - x_sim) - 0.5*(delta**2))
    #simple_plot(None,loss)
    return np.sum(y)


# Helper functions
def rmse(x, x_sim):
    """ root mean square error"""
    return np.sqrt(np.mean(se(x, x_sim)))

def se(x, x_sim):
    """ square error"""
    return np.power((x - x_sim), 2)


def calculate_cost(x, x_sim, metric_eval):
    cost_val = 0.0
    if measurement_coordinate_frame == 'cartesian':
        x_pos = metric_eval(x[0,:], x_sim[0,:])
        y_pos = metric_eval(x[1,:], x_sim[1,:])
        z_rot = metric_eval(x[2,:], x_sim[2,:])
        obj_cost = x_pos + y_pos + z_rot
    elif measurement_coordinate_frame == 'polar':
        rho = metric_eval(x[0,:], x_sim[0,:])
        yaw =  metric_eval(x[1,:], x_sim[1,:])
        obj_cost = rho + yaw
    return obj_cost

"""
def calculate_cost(x, x_sim, metric_eval):
    cost_val = 0.0
    if measurement_coordinate_frame == 'cartesian':
        obj_cost = np.sum([metric_eval(x[0,:], x_sim[0,:]),
                           metric_eval(x[1,:], x_sim[1,:]),
                           metric_eval(x[2,:], x_sim[2,:])])

    elif measurement_coordinate_frame == 'polar':
        obj_cost = np.sum([metric_eval(x[0,:], x_sim[0,:]),
                           metric_eval(x[1,:], x_sim[1,:])])

    return obj_cost
"""
def metric_selector(cost_name):
    if cost_name == "SE":
        return SE
    elif cost_name == "RMSE":
        return RMSE
    elif cost_name == 'AsRMSE':
        return AsRMSE
    elif cost_name == "MsRMSE":
        return MsRMSE
    else:
        rospy.logwarn('undefined cost function is requested, see cost_fn_library for const function definitions')


if __name__ == '__main__':
    x_sim = np.arange(-10,10,1)
    x = np.zeros(x_sim.shape)
    delta = 1
    y = hampel(x, x_sim, delta)