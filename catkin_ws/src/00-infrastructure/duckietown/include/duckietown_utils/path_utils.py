import os

from .exceptions import DTConfigException

__all__ = [
    'get_ros_package_path',
    'expand_all',
    'get_baseline_config_path',
]


def expand_all(filename):
    """
        Expands ~ and ${ENV} in the string.

        Raises DTConfigException if some environment variables
        are not expanded.

    """
    fn = filename
    fn = os.path.expanduser(fn)
    fn = os.path.expandvars(fn)
    if '$' in fn:
        msg = 'Could not expand all variables in path %r.' % fn
        raise DTConfigException(msg)
    return fn


def get_ros_package_path(package_name):
    """ Returns the path to a package. """
    import rospkg  # @UnresolvedImport
    rospack = rospkg.RosPack()  # @UndefinedVariable
    return rospack.get_path(package_name)

# def display_filename(filename):
#     """ Displays a filename in a possibly simpler way """
#     return friendly_path(filename)

def get_baseline_config_path(package_name, config_name="default.yaml"):
    """ return the config ** config_name ** under baselines for the package ** package_name **"""
    base = get_ros_package_path("duckietown")
    return os.path.join(base, "config", "baseline", package_name, config_name)