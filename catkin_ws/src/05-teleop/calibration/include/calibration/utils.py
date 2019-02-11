import rospy
import os
import yaml
import numpy as np
import datetime
from os.path import join
from os import mkdir
from duckietown_utils import (logger, get_duckiefleet_root)
from duckietown_utils.yaml_wrap import yaml_load_file

def defined_ros_param(ros_param_name):
    try:
        param_val = rospy.get_param(ros_param_name)
    except KeyError:
        rospy.logfatal('Parameter {} is not defined in ROS parameter server'.format(ros_param_name))
        param_val = None
    return param_val

def get_file_path(veh_name, model_name):
    if model_name == 'gt':
        return (get_duckiefleet_root()+'/calibrations/kinematics/' + veh_name + ".yaml")
    else:
        return (get_duckiefleet_root() + '/calibrations/kinematics/' + veh_name + "_" + model_name + ".yaml")

def read_param_from_file(veh_name,model_object):
    # Check file existence
    fname = get_file_path(veh_name, model_object.name)
    # Use default.yaml if file doesn't exsit
    if not os.path.isfile(fname):
        rospy.logwarn("[%s] %s does not exist, will use model defaults" %('kinematic_calibration',fname))
        return None

    with open(fname, 'r') as in_file:
        try:
            yaml_dict = yaml.load(in_file)
        except yaml.YAMLError as exc:
            rospy.logfatal("[%s] YAML syntax error. File: %s fname. Exc: %s" %('kinematic_calibration', fname, exc))
            rospy.signal_shutdown()
            return

    if yaml_dict is None:
        # Empty yaml file
        rospy.logwarn('[{}] calibration file exists, but has no content, see: {}'.format('kinematic_calibration', fname))
        return
    else:
        rospy.loginfo('[{}] using the parameter values from existing YAML file as the initial guesses: {}'.format('kinematic_calibration', fname))
        model_param_dict = {}
        for param in model_object.model_params.keys():
            try:
                model_param_dict[param] = yaml_dict[param]
            except KeyError:
                rospy.logfatal(
                    '[{}] attempting to access non-existing key [{}], is it defined in the YAML file?'.format('kinematic_calibration', param))
        return model_param_dict

def dict_to_ordered_array(model_object, param_dict):
    param_array = []
    for param in model_object.param_ordered_list:
        param_array.append(param_dict[param])
    return param_array

def get_package_root(package_name):
    import rospkg
    rospack = rospkg.RosPack()
    return rospack.get_path(package_name)

def create_results_dir(package_root):
    """ create a results directory under the package root with the date and time information """
    time_label = datetime.datetime.now().strftime('%Y-%m-%d__%H:%M:%S')
    result_dir = join(str(time_label), package_root, "results", time_label)
    mkdir(result_dir)
    return result_dir

def input_folder_to_experiment_dict(folder_path):
    experiments = {}
    bag_files = os.listdir(folder_path)
    for bag in bag_files:
        bag_name = os.path.splitext(bag)[0]
        experiments[bag_name] = {'wheel_cmd_exec': None, 'robot_pose': None, 'path': join(folder_path, bag)}
    return experiments

def safe_create_dir(path):
    if not os.path.isdir(path):
        mkdir(path)
    return path

def get_param_from_config_file(param_name):
    return yaml_load_file(get_package_root("calibration") + '/config.yaml', plain_yaml=True)[param_name]

def x_in_np(data):
    if type(data) == dict:
        data = data['robot_pose']
    return data

def deg(x):
    """ converts a numpy array expressed in radians to degrees"""
    if not type(np.ndarray):
        raise("deg expects an input of type numpy.ndarray")
    return x * 180 / np.pi

def rad(x):
    """ converts a numpy array expressed in degrees to radians"""
    if not type(np.ndarray):
        raise ("rad expects an input of type numpy.ndarray")
    return x * np.pi / 180


