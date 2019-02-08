import numpy as np
from utils import rad

def col(a):
    """

    Args:
        a (numpy array):

    Returns:

    """
    return a.reshape(a.size,1)

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
    x_polar[0,:] = -1 * np.sqrt(x_cart[0,:] ** 2 + x_cart[1,:] ** 2)
    x_polar[1,:] = x_cart[2,:]
    return x_polar

def x_polar_to_cart(x_polar):
    # initialize the array for storing measurements represented in cartesian coordinates
    # notice that x and y will never be 0, so initializing to zero is robust.
    x_cart = np.zeros((3, x_polar.shape[1]))
    x_cart[0,:] = x_polar[0,:] * np.cos(rad(x_polar[1,:])) # rho * cos(theta)
    x_cart[1,:] = -x_polar[0,:] * np.sin(rad(x_polar[1,:]))  # rho * sin(theta)
    x_cart[2, :] = x_polar[1,:]

    return x_cart
