import numpy as np
from plotting_utils import simple_plot
from scipy.interpolate import splrep, splev
from utils import rad, save_gzip


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


def x_adapter(x_dict):
    """ converts from dict to numpy array """
    px = row(np.array(x_dict['px']))
    py = row(np.array(x_dict['py']))
    rz = row(np.array(x_dict['rz']))

    return np.vstack([px, py, rz])


def x_cart_to_polar(x_cart):
    # initialize the array for storing measurements represented in polar coordinates
    # notice that rho will never be 0, so initializing to zero is robust
    x_polar = np.zeros((2, x_cart.shape[1]))
    # note the negation of rho values, this is to conform the model definition where positive wheel cmds
    # leading to increase in rho.
    x_polar[0, :] = -1 * np.sqrt(x_cart[0, :] ** 2 + x_cart[1, :] ** 2)
    x_polar[1, :] = x_cart[2, :]
    return x_polar


def x_polar_to_cart(x_polar):
    # initialize the array for storing measurements represented in cartesian coordinates
    # notice that x and y will never be 0, so initializing to zero is robust.
    x_cart = np.zeros((3, x_polar.shape[1]))
    x_cart[0, :] = x_polar[0, :] * np.cos(rad(x_polar[1, :]))  # rho * cos(theta)
    x_cart[1, :] = -x_polar[0, :] * np.sin(rad(x_polar[1, :]))  # rho * sin(theta)
    x_cart[2, :] = x_polar[1, :]

    return x_cart


def to_eql(dataset, dataset_type):
    first_experiment = True
    n_int = 5
    for exp_name in dataset.keys():
        # unpack data for each experiment
        opt_data = dataset[exp_name].data
        opt_data_x = np.array(opt_data["robot_pose"])
        opt_data_u = np.array(opt_data["wheel_cmd_exec"])
        opt_data_t = np.array(opt_data["timestamp"])

        # simple_plot(opt_data_t, opt_data_x[0, :])
        # simple_plot(opt_data_t, opt_data_x[1, :])

        # create a denser t array by linear interpolation
        # opt_data_t_int = densify(opt_data_t, n=n_int)
        # cubic spline interpolation on t_dense
        # opt_data_x_int = interpolate(opt_data_x, opt_data_t_int, n=n_int)

        # simple_plot(opt_data_t_int, opt_data_x_int[0,:])
        # simple_plot(opt_data_t_int, opt_data_x_int[1,:])

        if first_experiment:
            all_x = opt_data_x.copy()
            all_u = opt_data_u.copy()
            all_t = opt_data_t.copy()
            first_experiment = False
        else:
            all_x = np.hstack((all_x, opt_data_x))
            all_u = np.hstack((all_u, opt_data_u))
            all_t = np.hstack((all_t, opt_data_t))

    x_spline = create_spline_representation(opt_data_t, opt_data_x)
    x_val = evaluate_spline(opt_data_t, x_spline, mode="eval")
    x_dot = evaluate_spline(opt_data_t, x_spline, mode="diff")

    simple_plot(opt_data_t, all_x[0, :])
    simple_plot(opt_data_t, all_x[1, :])
    all_x_int = all_x.copy()

    simple_plot(opt_data_t, all_x_int[1, :])

    # gradient along row direction: dx/dt
    # for details: https://docs.scipy.org/doc/numpy/reference/generated/numpy.gradient.html
    all_x_dot = np.gradient(all_x, opt_data_t, axis=1, edge_order=1)

    # simple_plot(opt_data_t, all_x)
    simple_plot(opt_data_t, all_x_dot[0, :])
    simple_plot(opt_data_t, all_x_dot[1, :])

    simple_plot(opt_data_t, x_dot[0, :])
    simple_plot(opt_data_t, x_dot[1, :])

    # generate a random index array
    random_index_array = np.random.permutation(all_t.size)
    # shuffle the data
    shuffled_x = all_x_dot[:, random_index_array]
    shuffled_u = all_u[:, random_index_array]

    # transpose is required as eql software concatenates training points vertically
    shuffled_data = (np.transpose(shuffled_u), np.transpose(shuffled_x))
    save_gzip("experiment_xdot_ramp", shuffled_data, dataset_type)


def create_spline_representation(t, x):
    x_spline = []
    for i in range(x.shape[0]):
        tck = splrep(t, x[i, :], s=0)
        x_spline.append(tck)
    return x_spline


def evaluate_spline(t_eval, x_spline, mode="eval"):
    first = True
    for i in range(len(x_spline)):
        x_spline_i = x_spline[i]
        if mode == "eval":
            x_interp_i = splev(t_eval, x_spline_i, der=0)
        elif mode == "diff":
            x_interp_i = splev(t_eval, x_spline_i, der=1)

        if first:
            x_int = np.zeros((len(x_spline), t_eval.size))
            first = False
        x_int[i, :] = x_interp_i

    return x_int


def densify(x, n):
    """ add n point in between each data points
    x 1d array """
    x_densified = x.copy()
    for i in range(x.size - 1):
        x_s, x_e = x[i], x[i + 1]
        x_densified_interval = np.linspace(x_s, x_e, num=n + 2, endpoint=open)
        x_densified = np.concatenate((x_densified, x_densified_interval[1:-1]), axis=None)
    print("selcuk")
    return x_densified
