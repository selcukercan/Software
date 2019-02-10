import numpy as np
from utils import get_param_from_config_file
from plotting_utils import simple_plot

measurement_coordinate_frame = get_param_from_config_file("express_measurements_in")

def norm_rmse(x, x_sim):
    if measurement_coordinate_frame == "cartesian":
        norm_mse_x = rmse(x[0, :], x_sim[0, :]) / (max(x[0, :]) - min(x[0, :]))
        norm_mse_y = rmse(x[1, :], x_sim[1, :]) / (max(x[1, :]) - min(x[1, :]))
        norm_mse_yaw = rmse(x[2, :], x_sim[2, :]) / (max(x[2, :]) - min(x[2, :]))
        return np.mean(norm_mse_x + norm_mse_y + norm_mse_yaw)
    elif measurement_coordinate_frame == "polar":
        norm_mse_rho = rmse(x[0, :], x_sim[0, :]) / (max(x[0, :]) - min(x[0, :]))
        norm_mse_yaw = rmse(x[1, :], x_sim[1, :]) / (max(x[1, :]) - min(x[1, :]))
        return np.mean(norm_mse_rho + norm_mse_yaw)

def rmse(x, x_sim):
    """ root mean square error"""
    return np.sqrt(np.mean(np.power((x - x_sim),2)))

def hampel(x, x_sim, delta):
    """ hampel's loss
    delta may be interpreted as the precision requested.
    """
    loss = np.where(np.abs(x-x_sim) < delta , 0.5*((x-x_sim)**2), delta*np.abs(x - x_sim) - 0.5*(delta**2))
    simple_plot(None,loss)
    return np.sum(loss)

if __name__ == '__main__':
    x_sim = np.arange(-10,10,1)
    x = np.zeros(x_sim.shape)
    delta = 1
    y = hampel(x, x_sim, delta)