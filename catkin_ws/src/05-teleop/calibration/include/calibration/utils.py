import datetime
import gzip
import os
import pickle
import shutil
from os import mkdir
from os.path import join

import numpy as np
import rospy
import yaml
from duckietown_utils import (get_duckiefleet_root)
from duckietown_utils.yaml_wrap import yaml_load_file

options = {
    "results_dir": None,
    "tmp_dir": None,
    "path_to_config_file": None
}


def defined_ros_param(ros_param_name):
    try:
        param_val = rospy.get_param(ros_param_name)
    except KeyError:
        rospy.logfatal('Parameter {} is not defined in ROS parameter server'.format(ros_param_name))
        param_val = None
    return param_val


def get_file_path(veh_name, model_name):
    if model_name == 'gt':
        return (get_duckiefleet_root() + '/calibrations/kinematics/' + veh_name + ".yaml")
    else:
        return (get_duckiefleet_root() + '/calibrations/kinematics/' + veh_name + "_" + model_name + ".yaml")


def read_param_from_file(veh_name, model_object):
    # Check file existence
    fname = get_file_path(veh_name, model_object.name)
    # Use default.yaml if file doesn't exsit
    if not os.path.isfile(fname):
        rospy.logwarn("[%s] %s does not exist, will use model defaults" % ('kinematic_calibration', fname))
        return None

    with open(fname, 'r') as in_file:
        try:
            yaml_dict = yaml.load(in_file)
        except yaml.YAMLError as exc:
            rospy.logfatal("[%s] YAML syntax error. File: %s fname. Exc: %s" % ('kinematic_calibration', fname, exc))
            rospy.signal_shutdown()
            return

    if yaml_dict is None:
        # Empty yaml file
        rospy.logwarn(
            '[{}] calibration file exists, but has no content, see: {}'.format('kinematic_calibration', fname))
        return
    else:
        rospy.loginfo('[{}] using the parameter values from existing YAML file as the initial guesses: {}'.format(
            'kinematic_calibration', fname))
        model_param_dict = {}
        for param in model_object.model_params.keys():
            try:
                model_param_dict[param] = yaml_dict[param]
            except KeyError:
                rospy.logfatal(
                    '[{}] attempting to access non-existing key [{}], is it defined in the YAML file?'.format(
                        'kinematic_calibration', param))
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
    time_label = datetime.datetime.now().strftime('%Y-%m-%d_%H:%M:%S')
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


def save_gzip(file_name, processed_dataset, dataset_type):
    if dataset_type == "train":
        data_file = os.path.join(file_name + '_train_val')
    elif dataset_type == "test":
        data_file = os.path.join(file_name + '_test')
    pickle.dump(processed_dataset, gzip.open(data_file, "wb"))


def get_param_from_config_file(param_name):
    return yaml_load_file(get_package_root("calibration") + '/config.yaml', plain_yaml=True)[param_name]


def x_in_np(data):
    if type(data) == dict:
        data = data['robot_pose']
    return data


def deg(x):
    """ converts a numpy array expressed in radians to degrees"""
    if not type(np.ndarray):
        raise ("deg expects an input of type numpy.ndarray")
    return x * 180 / np.pi


def rad(x):
    """ converts a numpy array expressed in degrees to radians"""
    if not type(np.ndarray):
        raise ("rad expects an input of type numpy.ndarray")
    return x * np.pi / 180


def cost_function_over_param_space(self, model_object, experiments):
    """
    place right before the actual optimization
    # brute-force cost calculation and plotting over parameter-space
    #cost, params_space_list = self.cost_function_over_param_space(model_object, experiments)
    #param_space_cost_plot(cost, params_space_list)
    """
    cost = []
    params_space_list = []

    i = 0
    param_list = model_object.param_ordered_list
    tup_dr, tup_dl, tup_L = [model_object.model_params[param_list[i]]['search'] for i in range(3)]
    init_dr, init_dl, init_L = [model_object.model_params[param_list[i]]['param_init_guess'] for i in range(3)]
    dr_list = np.arange((init_dr - tup_dr[0]), (init_dr + tup_dr[0]), tup_dr[1])
    dl_list = np.arange((init_dl - tup_dl[0]), (init_dl + tup_dl[0]), tup_dl[1])
    L_list = np.arange((init_L - tup_L[0]), (init_L + tup_L[0]), tup_L[1])
    for dr in dr_list:
        for dl in dl_list:
            for L in L_list:
                cost.append(self.cost_function((dr, dl, L), model_object, experiments))
                params_space_list.append((dr, dl, L))
                print(i)
                i += 1
    return cost, params_space_list


def work_space_settings():
    package_root = get_package_root("calibration")
    options["results_dir"] = create_results_dir(package_root)
    options["tmp_dir"] = safe_create_dir(os.path.join(package_root, "tmp"))
    options["path_to_config_file"] = package_root + '/config.yaml'

def get_workspace_param(option):
    return options[option]


def cautious_read_param_from_file(robot_name, model_object):
    """ attempt to read the parameter values from config file, fallback model defaults if ccnfig file does not exist"""
    # see if there already is a yaml file for the model we can use
    model_param_dict = read_param_from_file(robot_name, model_object)
    if model_param_dict is not None:
        p0 = dict_to_ordered_array(model_object,
                                        model_param_dict)  # if you have one, use these values as your initial guesses.
    else:
        p0 = dict_to_ordered_array(model_object,
                                        model_object.get_param_initial_guess_dict())  # otherwise use the default initial guesses as defined in the class of our model choice
        rospy.logwarn('[{}] using default initial guesses defined in model {}'.format('kinematic_calibration',model_object.name))
    return p0

def pack_results(results_dir):
    result_name = os.path.basename(results_dir)
    results_root = os.path.dirname(results_dir)
    shutil.make_archive(results_dir, 'zip', results_root, result_name)
