import numpy as np
from plotting_utils import simple_plot, multiplot
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

        if first_experiment:
            all_x = opt_data_x.copy()
            all_u = opt_data_u.copy()
            all_t = opt_data_t.copy()
            first_experiment = False
        else:
            all_x = np.hstack((all_x, opt_data_x))
            all_u = np.hstack((all_u, opt_data_u))
            all_t = np.hstack((all_t, opt_data_t))

    get_x_dot(all_x, all_t, mode="simple")
    get_x_dot(all_x, all_t, mode="spline")
    #get_x_dot(all_x, all_t, mode="dense", n_int=1)


    # generate a random index array
    random_index_array = np.random.permutation(all_t.size)
    # shuffle the data
    shuffled_x = all_x_dot[:, random_index_array]
    shuffled_u = all_u[:, random_index_array]

    # transpose is required as eql software concatenates training points vertically
    shuffled_data = (np.transpose(shuffled_u), np.transpose(shuffled_x))
    save_gzip("experiment_xdot_ramp", shuffled_data, dataset_type)

def get_x_dot(x, t, mode=None, **kwargs):
    """
    calculates the differentiation of position measurement

    Args:
        x: 2*N polar input measurement
        t: 1*N time measurement

    Returns: d

    """
    if mode == "simple":
        # gradient along row direction: dx/dt
        # for details: https://docs.scipy.org/doc/numpy/reference/generated/numpy.gradient.html
        x_dot = np.gradient(x, t, axis=1, edge_order=1)
        t_xdot = t
    elif mode == "spline":
        x_spline = create_spline_representation(t, x)
        x_val = evaluate_spline(t, x_spline, mode="eval")
        x_dot = evaluate_spline(t, x_spline, mode="diff")
        t_xdot = t
    elif mode == "dense":
        from scipy import interpolate
        f = interpolate.interp1d(t, x) # create interpolation function
        n_int = kwargs["n_int"]
        t_xdot = t_dense = densify(t, n=n_int) # create a denser t array by linear interpolation
        #simple_plot(None, t_dense)
        x_dense = f(t_dense) # evaluate the function at the denser interval
        x_dot = np.gradient(x_dense, t_dense, axis=1, edge_order=1)

    multiplot(states_list=[x, x_dot], time_list=[t, t_xdot], experiment_name_list=["x", "x_dot"], plot_title='Using differentiation scheme {}'.format(mode), save=False,
              save_dir="")

    return x_dot


def create_spline_representation(t, x):
    from scipy.interpolate import splrep
    x_spline = []
    for i in range(x.shape[0]):
        tck = splrep(t, x[i, :], s=0)
        x_spline.append(tck)
    return x_spline


def evaluate_spline(t_eval, x_spline, mode="eval"):
    from scipy.interpolate import splev
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

    mu, sigma = 0, 0.2 # mean and standard deviation
    t_beg = 0
    t_end = 4 * np.pi
    t = np.arange(t_beg, t_end, 0.05) # sampling frequency of 20 Hz
    noise = np.random.normal(mu, sigma, t.size)
    # Position Information
    x_original = np.sin(t)
    x_corr = x_original + noise

    t_eval = np.arange(t_beg, t_end, 0.01)

    time_regular_spline_beg = time.time()
    x_spline_rep =  splrep(t, x_corr, s=10)
    x_spline = splev(t_eval, x_spline_rep)
    duration_standart = time.time() - time_regular_spline_beg

    #t_n = [-1, 0, 1]
    k = 3
    # Open question: what is the optimal value for knots in terms of time-seperation?
    t_n = np.r_[(t[0],)*(k+1),
          t[3:-3:20],
          (t[-1],)*(k+1)]

    time_lsq_spline_beg = time.time()
    spl_lsq = make_lsq_spline(t, x_corr, t_n, k)
    x_lsq_spline = spl_lsq(t_eval)
    duration_lsq_spline = time.time() - time_lsq_spline_beg

    print('regular spline: {} lsq_spline: {} ration: {}'.format(duration_standart, duration_lsq_spline, duration_lsq_spline/duration_standart))

    spl_lsq_der = spl_lsq.derivative()
    x_dot_lsq_spline = spl_lsq_der(t_eval)
    multiplot(states_list=[x_original, x_corr, x_spline, x_lsq_spline], time_list=[t, t, t_eval, t_eval], experiment_name_list=['original', 'corrupted',  'spline_fitted', 'lsq_spline_fitted'])

    time.sleep(1)
    # Velocity Information
    x_dot_simple = np.gradient(x_original, t, edge_order=1)
    x_dot_spline = splev(t_eval, x_spline_rep, der=1)

    multiplot(states_list=[x_dot_simple, x_dot_spline, x_dot_lsq_spline], time_list=[t, t_eval, t_eval], experiment_name_list=['original_dot', 'spline_fitted', 'lsq_spline'])
