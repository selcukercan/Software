import numpy as np
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