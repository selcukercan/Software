import rospy
import os
import yaml
from duckietown_utils import (logger, get_duckiefleet_root)

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
    elif model_name == 'sysid':
        return (get_duckiefleet_root()+'/calibrations/kinematics/' + veh_name + "_sysid" + ".yaml")
    elif model_name == 'kinematic_drive':
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
