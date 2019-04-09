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
    "path_to_config_file": None,
    "results_optimization_dir": None,
    "results_preprocessing_dir": None
}

config={}

def update_local_config_dict():
    global config
    config = yaml_load_file(get_workspace_param("path_to_config_file"), plain_yaml=True)

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


def get_software_version(path_to_repo):
    """
    recursively search for a git repo given a candidate path:
    return False if an exception is raised indicating no git repo is found which is the case on deployment to the duckiebot
    """
    from git import Repo
    try:
        repo = Repo(path_to_repo, search_parent_directories=True)
        last_commit_id = repo.head.commit.hexsha
        return last_commit_id
    except:
        return False

def create_time_label():
    import datetime
    return datetime.datetime.now().strftime('%Y_%m_%d_%H_%M_%S')

def create_results_dir(package_root):
    """ create a results directory under the package root with the date and time information """
    time_label = create_time_label()
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

def copy_calibrations_folder(results_dir):
    from distutils.dir_util import copy_tree
    calibrations_folder = os.path.join(get_duckiefleet_root(), 'calibrations')
    dst = os.path.join(results_dir, 'calibrations')
    os.mkdir(dst)
    copy_tree(calibrations_folder, dst)

def copy_folder(src_folder, dst_dir):
    from distutils.dir_util import copy_tree
    copy_tree(src_folder, dst_dir)

def safe_create_dir(path):
    if not os.path.isdir(path):
        mkdir(path)
    return path

def get_files_in_dir(dir_name):
    """ get files inside a directory """
    return [f for f in os.listdir(dir_name) if os.path.isfile(os.path.join(dir_name, f))]

def save_gzip(file_name, processed_dataset, dataset_type):
    import gzip
    if dataset_type == "train":
        data_file = os.path.join(file_name + '_train_val')
    elif dataset_type == "test":
        data_file = os.path.join(file_name + '_test')
    pickle.dump(processed_dataset, gzip.open(data_file, "wb"))


def get_param_from_config_file(param_name):
    #return yaml_load_file(get_package_root("calibration") + '/config.yaml', plain_yaml=True)[param_name]
    config_file_path = get_config_file_path()
    return yaml_load_file(config_file_path, plain_yaml=True)[param_name]


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
    options["results_optimization_dir"] = safe_create_dir(os.path.join(options["results_dir"], "optimization"))
    options["results_preprocessing_dir"] = safe_create_dir(os.path.join(options["results_dir"], "preprocessing"))
    options["tmp_dir"] = safe_create_dir(os.path.join(package_root, "tmp"))
    options["path_to_config_file"] = get_config_file_path()

    # strecthing the function a bit, called here kinematic calls it.
    update_local_config_dict()

def get_workspace_param(option):
    return options[option]

def get_config_file_path():
    # use meta config file to decide which configuration file to use.
    package_root = get_package_root("calibration")
    meta_config = yaml_load_file(package_root + '/meta_config.yaml', plain_yaml=True)
    use_config_version = meta_config["system_identification_config_version"]
    return package_root + '/configs/system_identification/config_{}.yaml'.format(use_config_version)

def cautious_read_param_from_file(robot_name, model_object):
    """ attempt to read the parameter values from config file, fallback model defaults if config file does not exist"""
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
    shutil.make_archive(results_dir, 'tar', results_root, result_name)

def defaulted_param_load(model_object, robot_name):
    """ check if there already exists a previous calibration, if none use default values defined by the model """
    model_param_dict = read_param_from_file(robot_name, model_object)
    if model_param_dict is not None:
        p0 = dict_to_ordered_array(model_object,
                                        model_param_dict)  # if you have one, use these values as your initial guesses.
        rospy.logwarn('using existing parameters..')
    else:
        p0 = dict_to_ordered_array(model_object,
                                        model_object.get_param_initial_guess_dict())  # otherwise use the default initial guesses as defined in the class of our model choice
        rospy.logwarn('using default initial guesses defined in model {} ..'.format(model_object.name))
    return p0

def get_hostname():
    import socket
    hostname = socket.gethostname()
    return hostname

def get_cpu_info():
    import platform
    return platform.processor()

def reshape_x(x):
    return np.array(x).reshape(2, 1)

def window(x, discard_n):
    """ discard n points at the boundaries"""
    if x.ndim is 1:
        return x[discard_n:-discard_n]
    else:
        return x[:,discard_n:-discard_n]

def is_valid_param(self, param_name=None, param_address=None, valid_params=None):
    """ checks if param_name is allowed  """
    param_val = rospy.get_param(param_address)
    if param_val in valid_params:
        return param_val
    else:
        rospy.logfatal('[{}] {} is not a valid argument, please select one of {}'.format(self.node_name, param_name, str(valid_params)))

def get_baseline_config(package_name, config_name="default.yaml"):
    """ return the config ** config_name ** under baselines for the package ** package_name **"""
    base = get_package_root("duckietown")
    return os.path.join(base, "config", "baseline", package_name, config_name)

if __name__ == '__main__':
    print get_files_in_dir('/home/selcuk/multi_bag_processing/')
