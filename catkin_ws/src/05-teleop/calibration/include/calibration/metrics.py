import numpy as np
from utils import get_param_from_config_file

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
    return np.sqrt(np.mean(np.power((x - x_sim),2)))