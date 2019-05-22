import numpy as np
import rospy
import time
from calibration.utils import get_param_from_config_file, window, get_workspace_param
from plotting_utils import multiplot
from scipy.interpolate import splrep, splev
from utils import rad, save_gzip

save_plots = get_param_from_config_file("save_experiment_results")

def col(a):
    """
    Args:
        a (numpy array):

    Returns:
    """
    return a.reshape(a.size, 1)


def row(a):
    """
    Args:
        a (numpy array):

    Returns:
    """

    return a.reshape(1, a.size)


def u_adapter(u_dict):
    """ converts from dict to numpy array """
    v_l_r = row(np.array(u_dict['vel_l']))
    v_r_r = row(np.array(u_dict['vel_r']))
    return np.vstack([v_r_r, v_l_r])


def x_adapter_apriltag(x_dict):
    """ selects/converts apriltag measurements from dict to numpy array """
    px = row(np.array(x_dict['px']))
    py = row(np.array(x_dict['py']))
    rz = row(np.array(x_dict['rz']))

    return np.vstack([px, py, rz])


def x_adapter_lane_filter(x_dict):
    """ converts lane filter measurements from dict to numpy array """
    d = row(np.array(x_dict['d']))
    phi = row(np.array(x_dict['phi']))

    return np.vstack([d, phi])


def x_adapter(x_dict, localization_type=None):
    """ from load bag format to optimization format also takig into account the requested coordiate frame representation"""
    if localization_type == 'apriltag':
        return x_cart_to_polar(x_adapter_apriltag(x_dict))
    elif localization_type == 'lane_filter':
        return x_adapter_lane_filter(x_dict)


def x_cart_to_polar(x_cart):
    # initialize the array for storing measurements represented in polar coordinates
    # notice that rho will never be 0, so initializing to zero is robust
    x_polar = np.zeros((2, x_cart.shape[1]))
    # note the negation of rho values, this is to conform the model definition where positive wheel cmds
    # leading to increase in rho.
    x_polar[0, :] = -1 * np.sqrt(x_cart[0, :] ** 2 + x_cart[1, :] ** 2)
    # note the negation of theta values, this is to conform the model definition where;
    # if wheel_cmd_right > wheel_cmd_left the vehicle in increasing theta direction
    # remark: this is not a hackish fix:
    # 1) We do need to have measurements represented in vehicle coordinate frame vehicle_T_world
    # (see roscd apriltags2_ros/include/rotation_utils.py)
    # 2) Attempting to take the inverse of vehicle_T_world is an errenous attempt as then the displacement
    # vector will be represented in AT coordinate frame.
    x_polar[1, :] = -1 *x_cart[2, :]
    return x_polar


def x_polar_to_cart(x_polar):
    # initialize the array for storing measurements represented in cartesian coordinates
    # notice that x and y will never be 0, so initializing to zero is robust.
    x_cart = np.zeros((3, x_polar.shape[1]))
    x_cart[0, :] = x_polar[0, :] * np.cos(rad(x_polar[1, :]))  # rho * cos(theta)
    x_cart[1, :] = -x_polar[0, :] * np.sin(rad(x_polar[1, :]))  # rho * sin(theta)
    x_cart[2, :] = x_polar[1, :]

    return x_cart


def add_x_dot_estimate_to_dataset(dataset, dataset_type=None):
    """ estimates the velocity form position data and adds it to dataset dictionary """

    # load spline-fitting related settings from the config file
    spline_smoothing_factor = get_param_from_config_file("spline_smoothing_factor")
    lsq_spline_order = get_param_from_config_file("lsq_spline_order")
    lsq_spline_knot_every = get_param_from_config_file("lsq_spline_knot_every")
    lsq_discard_n_at_boundary = get_param_from_config_file("lsq_discard_n_at_boundary")
    spline_type = get_param_from_config_file("spline_type")
    densification_factor = get_param_from_config_file("densification_factor")
    discard_n_point_at_boundaries = get_param_from_config_file("discard_n_after_spline_fitting")

    show_plots = get_param_from_config_file("show_plots")

    for exp_name in dataset.keys():
        # unpack data for each experiment
        exp_data = dataset[exp_name].data
        x = np.array(exp_data["robot_pose"])
        t = np.array(exp_data["timestamp"])
        t_eval = densify(t, densification_factor)  # create a denser array for evaluation

        x_spline = fit_spline_x(x, t, t_eval, spline_type='spline', spline_smoothing_factor=spline_smoothing_factor)
        x_dot_spline, t_calc_spline = get_x_dot(x, t, mode="spline", spline_smoothing_factor=spline_smoothing_factor)
        x_lsq_spline = fit_spline_x(x, t, t_eval,
                                    spline_type='lsq_spline',
                                    order=lsq_spline_order,
                                    knot_every=lsq_spline_knot_every,
                                    discard_n_at_boundary=lsq_discard_n_at_boundary)
        x_lsq_dot_spline, t_calc_lsq_spline = get_x_dot(x, t,
                                                        mode="lsq_spline",
                                                        order=lsq_spline_order,
                                                        knot_every=lsq_spline_knot_every,
                                                        discard_n_at_boundary=lsq_discard_n_at_boundary)

        # select an inner region to avoid the boundary effects
        x_window = window(x, discard_n_point_at_boundaries)
        t_window = window(t, discard_n_point_at_boundaries)
        t_eval_window = densify(t_window, densification_factor)  # create a denser array for evaluation

        x_spline_window = window(x_spline, discard_n_point_at_boundaries)
        x_dot_spline_window = window(x_dot_spline, discard_n_point_at_boundaries)
        x_lsq_spline_window = window(x_lsq_spline, discard_n_point_at_boundaries)
        x_lsq_dot_spline_window = window(x_lsq_dot_spline, discard_n_point_at_boundaries)

        if show_plots:
            # Position Measurement Fitting
            multiplot(states_list=[x, x_spline, x_lsq_spline],
                      time_list=[t, t_eval, t_eval],
                      experiment_name_list=['_experiment-data', '_spline-fitted', '_lsq-spline-fitted'],
                      plot_title='Position Measurements <br>' +
                                 dataset_type + ' Dataset: {}'.format(exp_name),
                      save=save_plots,
                      save_dir=get_workspace_param("results_preprocessing_dir"),
                      upload_this=False
                      )
            # Velocity Estimations from position
            multiplot(states_list=[x_dot_spline, x_lsq_dot_spline],
                      time_list=[t, t],
                      experiment_name_list=['_spline-fitted', '_lsq-spline-fitted'],
                      plot_title='Velocity Estimations With Spline Fitting <br>' +
                                 dataset_type + ' Dataset: {}'.format(exp_name),
                      save=save_plots,
                      save_dir=get_workspace_param("results_preprocessing_dir"),
                      upload_this=False
                      )
            # Position Measurement Fitting
            multiplot(states_list=[x_window, x_spline_window, x_lsq_spline_window],
                      time_list=[t_window, t_eval_window, t_eval_window],
                      experiment_name_list=['_experiment-data', '_spline-fitted', '_lsq-spline-fitted'],
                      plot_title='Position Measurements After Discarding n Boundary Points<br>' +
                                 dataset_type + ' Dataset: {}'.format(exp_name),
                      save=save_plots,
                      save_dir=get_workspace_param("results_preprocessing_dir"),
                      upload_this=False
                      )
            # Velocity Estimations from position
            multiplot(states_list=[x_dot_spline_window, x_lsq_dot_spline_window],
                      time_list=[t_window, t_window],
                      experiment_name_list=['_spline-fitted', '_lsq-spline-fitted'],
                      plot_title='Velocity Estimations With Spline Fitting After Discarding n Boundary Points<br>' +
                                 dataset_type + ' Dataset: {}'.format(exp_name),
                      save=save_plots,
                      save_dir=get_workspace_param("results_preprocessing_dir"),
                      upload_this=False
                      )

        # Update the dataset
        dataset[exp_name].data["robot_pose"] = x_window
        dataset[exp_name].data["timestamp"] = t_window

        if spline_type == 'b_spline':
            dataset[exp_name].data["robot_velocity"] = x_dot_spline_window
        elif spline_type == 'lsq_spline':
            dataset[exp_name].data["robot_velocity"] = x_lsq_dot_spline_window
        else:
            rospy.logfatal('[data_adapter_utils] [{}] is not a valid spline type'.format(spline_type))

    return dataset


def estimate_x_dot_eql(dataset, dataset_type):
    # generate a random index array
    random_index_array = np.random.permutation(all_t.size)
    # shuffle the data
    shuffled_x = all_x_dot[:, random_index_array]
    shuffled_u = all_u[:, random_index_array]

    # transpose is required as eql software concatenates training points vertically
    shuffled_data = (np.transpose(shuffled_u), np.transpose(shuffled_x))
    save_gzip("experiment_xdot_ramp", shuffled_data, dataset_type)


def fit_spline_x(x, t, t_eval, spline_type='spline', **kwargs):
    if spline_type == 'spline':
        # https://docs.scipy.org/doc/scipy/reference/generated/scipy.interpolate.splrep.html
        spline_smoothing_factor = kwargs["spline_smoothing_factor"]
        x_spline = create_spline_representation(t, x, spline_smoothing_factor=spline_smoothing_factor)
        x_eval = evaluate_spline(t_eval, x_spline, mode="evaluate")
    elif spline_type == 'lsq_spline':
        # https://docs.scipy.org/doc/scipy/reference/generated/scipy.interpolate.make_lsq_spline.html
        order = kwargs["order"]
        knot_every = kwargs["knot_every"]
        discard = kwargs["discard_n_at_boundary"]

        x_lsq_spline = create_lsq_spline_object(t, x, order, knot_every, discard)
        x_eval = evaluate_lsq_spline(t_eval, x_lsq_spline, mode='evaluate')
    else:
        rospy.logfatal('[data_adapter_utils] [{}] is not a valid spline type'.format(spline_type))
    return x_eval


def get_x_dot(x, t, mode=None, get_x_fit=True, **kwargs):
    """
    calculates the differentiation of the position measurement

    Args:
        x: 2*N polar input measurement
        t: 1*N time measurement
        mode: select which differentiation type to use

    Returns: d

    """
    time_beg = time.time()
    if mode == "simple":
        # gradient along row direction: dx/dt
        # for details: https://docs.scipy.org/doc/numpy/reference/generated/numpy.gradient.html
        x_dot = np.gradient(x, t, axis=1, edge_order=1)
        t_xdot = t
    elif mode == "spline":
        # https://docs.scipy.org/doc/scipy/reference/generated/scipy.interpolate.splrep.html
        spline_smoothing_factor = kwargs["spline_smoothing_factor"]
        x_spline = create_spline_representation(t, x, spline_smoothing_factor=spline_smoothing_factor)
        # x_val = evaluate_spline(t, x_spline, mode="eval")
        x_dot = evaluate_spline(t, x_spline, mode="derivative")
        t_xdot = t
    elif mode == "lsq_spline":
        # https://docs.scipy.org/doc/scipy/reference/generated/scipy.interpolate.make_lsq_spline.html
        order = kwargs["order"]
        knot_every = kwargs["knot_every"]
        discard = kwargs["discard_n_at_boundary"]

        x_lsq_spline = create_lsq_spline_object(t, x, order, knot_every, discard)
        x_dot = evaluate_lsq_spline(t, x_lsq_spline, mode='derivative')
        t_xdot = t
    elif mode == "dense":
        from scipy import interpolate
        f = interpolate.interp1d(t, x)  # create interpolation function
        n_int = kwargs["n_int"]
        t_xdot = t_dense = densify(t, n=n_int)  # create a denser t array by linear interpolation
        # simple_plot(None, t_dense)
        x_dense = f(t_dense)  # evaluate the function at the denser interval
        x_dot = np.gradient(x_dense, t_dense, axis=1, edge_order=1)

    calc_duration = time.time() - time_beg

    return x_dot, calc_duration


def create_lsq_spline_object(t, x, order, knot_every, discard):
    """ returns a list containing spline object for each x entry"""
    from scipy.interpolate import make_lsq_spline
    x_spline = []
    # Open question: what is the optimal value for knots in terms of time-seperation?
    for i in range(x.shape[0]):
        t_n = np.r_[(t[0],) * (order + 1), t[discard:-discard:knot_every], (t[-1],) * (order + 1)]
        spl_lsq = make_lsq_spline(t, x[i, :], t_n, order)  # creates a spline object
        x_spline.append(spl_lsq)
    return x_spline


def evaluate_lsq_spline(t_eval, x_spline, mode="eval"):
    first = True
    for i in range(len(x_spline)):
        x_lsq_spline_i = x_spline[i]
        if mode == "evaluate":
            x_eval_i = x_lsq_spline_i(t_eval)  # evaluates the spline at query points
        elif mode == "derivative":
            spl_lsq_der = x_lsq_spline_i.derivative()  # creates another spline object representing the derivative
            x_eval_i = spl_lsq_der(t_eval)  # evaluates the derivative of spline at query points

        if first:
            x_int = np.zeros((len(x_spline), t_eval.size))
            first = False
        x_int[i, :] = x_eval_i
    return x_int


def create_spline_representation(t, x, spline_smoothing_factor=0):
    from scipy.interpolate import splrep
    x_spline = []
    for i in range(x.shape[0]):
        tck = splrep(t, x[i, :], s=spline_smoothing_factor)
        x_spline.append(tck)
    return x_spline


def evaluate_spline(t_eval, x_spline, mode="eval"):
    from scipy.interpolate import splev
    first = True
    for i in range(len(x_spline)):
        x_spline_i = x_spline[i]
        if mode == "evaluate":
            x_interp_i = splev(t_eval, x_spline_i, der=0)
        elif mode == "derivative":
            x_interp_i = splev(t_eval, x_spline_i, der=1)

        if first:
            x_int = np.zeros((len(x_spline), t_eval.size))
            first = False
        x_int[i, :] = x_interp_i

    return x_int


def densify(x, n):
    """ add n point in between each data points
    x 1d array """
    first = True
    for i in range(x.size - 1):
        x_s, x_e = x[i], x[i + 1]
        x_densified_interval = np.linspace(x_s, x_e, num=n + 2)
        if first:
            x_densified = x_densified_interval[0:-1]
            first = False
        else:
            x_densified = np.concatenate((x_densified, x_densified_interval[0:-1]), axis=None)
    return x_densified


if __name__ == "__main__":
    import time
    from scipy.interpolate import make_lsq_spline

    mu, sigma = 0, 0.2  # mean and standard deviation
    t_beg = 0
    t_end = 4 * np.pi
    t = np.arange(t_beg, t_end, 0.05)  # sampling frequency of 20 Hz
    noise = np.random.normal(mu, sigma, t.size)
    # Position Information
    x_original = np.sin(t)
    x_corr = x_original + noise

    t_eval = np.arange(t_beg, t_end, 0.01)

    time_regular_spline_beg = time.time()
    x_spline_rep = splrep(t, x_corr, s=10)
    x_spline = splev(t_eval, x_spline_rep)
    duration_standart = time.time() - time_regular_spline_beg

    # t_n = [-1, 0, 1]
    k = 3
    # Open question: what is the optimal value for knots in terms of time-seperation?
    t_n = np.r_[(t[0],) * (k + 1),
                t[k:-k:20],
                (t[-1],) * (k + 1)]

    time_lsq_spline_beg = time.time()
    spl_lsq = make_lsq_spline(t, x_corr, t_n, k)
    x_lsq_spline = spl_lsq(t_eval)
    duration_lsq_spline = time.time() - time_lsq_spline_beg

    print('regular spline: {} lsq_spline: {} ration: {}'.format(duration_standart, duration_lsq_spline,
                                                                duration_lsq_spline / duration_standart))

    spl_lsq_der = spl_lsq.derivative()
    x_dot_lsq_spline = spl_lsq_der(t_eval)
    multiplot(states_list=[x_original, x_corr, x_spline, x_lsq_spline], time_list=[t, t, t_eval, t_eval],
              experiment_name_list=['original', 'corrupted', 'spline_fitted', 'lsq_spline_fitted'])

    time.sleep(1)
    # Velocity Information
    x_dot_simple = np.gradient(x_original, t, edge_order=1)
    x_dot_spline = splev(t_eval, x_spline_rep, der=1)

    multiplot(states_list=[x_dot_simple, x_dot_spline, x_dot_lsq_spline], time_list=[t, t_eval, t_eval],
              experiment_name_list=['original_dot', 'spline_fitted', 'lsq_spline'])
