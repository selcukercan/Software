#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import rosbag
import rospy
import time
import datetime
import os
import math
import yaml
import os.path

from duckietown_utils.path_utils import get_ros_package_path
from duckietown_utils.yaml_wrap import (yaml_load_file, yaml_write_to_file)
from duckietown_utils import (logger, get_duckiefleet_root)
#from duckietown_utils import rgb_from_ros
from scipy.optimize import minimize
from os.path import expanduser
from os.path import join

from calibration.data_preperation_utils import DataPreparation
from calibration.data_preperation_utils import load_pickle, save_pickle
from calibration.model_library import model_generator, simulate
from calibration.data_adapter_utils import *
from calibration.plotting_utils import *


def norm_rmse(x, x_sim):
    norm_mse_x = rmse(x[0, :], x_sim[0, :]) / (max(x[0, :]) - min(x[0, :]))
    norm_mse_y = rmse(x[1, :], x_sim[1, :]) / (max(x[1, :]) - min(x[1, :]))
    norm_mse_yaw = rmse(x[2, :], x_sim[2, :]) / (max(x[2, :]) - min(x[2, :]))
    return np.mean(norm_mse_x + norm_mse_y + norm_mse_yaw)

def rmse(x, x_sim):
    return np.sqrt(np.mean(np.power((x - x_sim),2)))


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

class calib():
    def __init__(self):
        PREPARE_CALIBRATION_DATA_FOR_OPTIMIZATION = True
        DEBUG = True

        # namespace variables
        if not DEBUG:
            host_package = rospy.get_namespace()  # as defined by <group> in launch file
        else:
            host_package = "/mete/calibration/"

        self.node_name = 'kinematic_calibration'  # node name , as defined in launch file
        host_package_node = host_package + self.node_name
        self.veh = host_package.split('/')[1]
        # Initialize the node with rospy
        rospy.init_node('calibration', anonymous=True)

        # Parameters
        param_veh = host_package_node + '/' + "veh"
        self.robot_name = defined_ros_param(param_veh)
        #self.robot_name = rospy.get_param(param_veh)
        param_folder_path = host_package_node + '/' + "folder_path"
        #path_training_data = rospy.get_param(param_folder_path)
        path_training_data = defined_ros_param(param_folder_path)
        path_test_data = '/home/selcuk/test_bags/test_set'

        # training data set container
        experiments = self.input_folder_to_experiment_dict(path_training_data)

        # topics of interest
        top_wheel_cmd_exec = "/" + self.robot_name + "/wheels_driver_node/wheels_cmd_executed"
        top_robot_pose = "/" + self.robot_name + "/apriltags2_ros/publish_detections_in_local_frame/tag_detections_local_frame"

        source = 'folder'
        load_from_pickle = 'test_run'
        save_to_pickle = True # True/False
        set_name = 'test_run'

        # load and process experiment data to be used in in the optimization
        if source == 'folder':
            for exp in experiments.keys():
                data_raw = DataPreparation(input_bag=experiments[exp]['path'], top_wheel_cmd_exec=top_wheel_cmd_exec, top_robot_pose=top_robot_pose)
                experiments[exp]['wheel_cmd_exec'], experiments[exp]['robot_pose'], experiments[exp]['timestamp']= data_raw.process_raw_data() # bring data set to format usable by the optimizer
            save_pickle(object=experiments, save_as=set_name)
        elif source == 'pickle':
            experiments = load_pickle(load_from_pickle)
        else:
            rospy.logfatal('[{}] is not a valid source type'.format(source))

        # construct a model by specifying which model to use
        model_object = model_generator('kinematic_drive')

        # Optimization Settings - for details refer to "https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.minimize.html"
        # see if there already is a yaml file for the model we can use
        model_param_dict = read_param_from_file(self.robot_name, model_object)
        if model_param_dict is not None:
            self.p0 = dict_to_ordered_array(model_object, model_param_dict) # if you have one, use these values as your initial guesses.
        else:
            self.p0 = dict_to_ordered_array(model_object, model_object.get_param_initial_guess_dict()) # otherwise use the default initial guesses as defined in the class of our model choice
            rospy.logwarn('[{}] using default initial guesses defined in model {}'.format('kinematic_calibration', model_object.name))
        # use the parameter bounds defined in the class of our model choice
        self.bounds = model_object.get_param_bounds_list()

        # self.delta =  0.00

        # run the optimization problem
        popt = self.nonlinear_model_fit(model_object, experiments)

        # test data set container
        test_dataset = self.input_folder_to_experiment_dict(path_test_data)

        # load and process the experiment data to be used in for testing the model
        for exp in test_dataset.keys():
            data_raw = DataPreparation(input_bag=test_dataset[exp]['path'], top_wheel_cmd_exec=top_wheel_cmd_exec,top_robot_pose=top_robot_pose)
            test_dataset[exp]['wheel_cmd_exec'], test_dataset[exp]['robot_pose'], test_dataset[exp]['timestamp'] = data_raw.process_raw_data()  # bring data set to format usable by the optimizer

        self.model_predictions(model_object, experiments, popt)
        #self.model_predictions(model_object, test_dataset, popt)

        # write to the kinematic calibration file
        self.write_calibration(model_object, popt)

    def cost_function(self, p, model_object, experiments):
        obj_cost = 0.0

        for exp_name in experiments.keys():
            exp_data = experiments[exp_name]
            t = exp_data['timestamp']
            x = exp_data['robot_pose']
            u = exp_data['wheel_cmd_exec']
            x0 = x[:,0]

            #simulate the model
            #states for a particular p set
            x_sim = simulate(model_object, t, x0, u, p)

            '''
            if self.cost_fn_plot_measurement:
                #plot_system(states= x, time=t, experiment_name=exp_name)
                self.cost_fn_plot_measurement = False
            #plot_system(states=x_sim, time=t, experiment_name=exp_name + '_simulated')
            '''

            range_x = float(max(x[0,:]) - min(x[0,:]))
            range_y = float(max(x[1,:]) - min(x[1, :]))
            range_yaw = float(max(x[2, :]) - min(x[2, :]))
            #print('range x: {} range y: {} range yaw: {}'.format(range_x,range_y,range_yaw))

            for i in range(len(t)):
                """
                obj_cost += ( abs(((x_sim[0, i] - x[0, i])) / range_x) +
                              abs(((x_sim[1, i] - x[1, i])) / range_y) +
                              abs(((x_sim[2, i] - x[2, i])) / range_yaw)
                    )
                """


                obj_cost += (
                             ((x_sim[0, i] - x[0, i])) ** 2 +
                             ((x_sim[1, i] - x[1, i])) ** 2 +
                             ((x_sim[2, i] - x[2, i])) ** 2
                            )
                """
                obj_cost += (
                             ((x_sim[0, i] - x[0, i]) / range_x) ** 2 +
                             ((x_sim[1, i] - x[1, i]) / range_y) ** 2  +
                             ((x_sim[2, i] - x[2, i]) / range_yaw) ** 2
                            )
                """
        return obj_cost

    def nonlinear_model_fit(self, model_object, experiments):
        print 'IN NONLINEAR MODEL FIT'
        # Actual Parameter Optimization/Fitting
        # Minimize the least squares error between the model prediction
        result = minimize(self.cost_function, self.p0, args=(model_object, experiments), bounds=self.bounds)
        print('[BEGIN] Optimization Result\n {} [END] Optimization Result\n'.format(result))

        return result.x

    def model_predictions(self, model_object, experiments, popt):
        for exp_name in experiments.keys():
            exp_data = experiments[exp_name]
            t = exp_data['timestamp']
            x = exp_data['robot_pose']
            u = exp_data['wheel_cmd_exec']
            x0 = x[:, 0]

            # simulate the model
            # states for a particular p set
            x_sim_opt = simulate(model_object, t, x0, u, popt)
            x_sim_init = simulate(model_object, t, x0, u, self.p0)

            """
            plot_system(states=x, time=t, experiment_name=exp_name + '_measurement')
            plot_system(states=x_sim_init, time=t, experiment_name=exp_name + '_simulated_init')
            plot_system(states=x_sim_opt, time=t, experiment_name=exp_name + '_simulated_optimal')
            """
            exp_mse = norm_rmse(x, x_sim_opt)

            print('\nModel Performance Evaluation:\nModel Name: {}\nMSE: {}'.format(exp_name, exp_mse))


            multiplot(states_list=[x, x_sim_init, x_sim_opt],
                      input_list=[u,u,u],
                      time_list=[t,t,t],
                      experiment_name_list=[exp_name + '_measurement', exp_name + '_simulated_init', exp_name + '_simulated_optimal'],
                      mode = 'single_view')


    @staticmethod
    def input_folder_to_experiment_dict(folder_path):
        experiments = {}
        bag_files = os.listdir(folder_path)
        for bag in bag_files:
            bag_name = bag.split('.')[0]
            experiments[bag_name] = {'wheel_cmd_exec': None, 'robot_pose': None, 'path': join(folder_path, bag)}
        return experiments

    def write_calibration(self, model_object, popt):
       # Form yaml content to write
       yaml_dict = {}
       for i, param in enumerate(model_object.param_ordered_list):
           yaml_dict[param] = popt[i].item()
       yaml_dict['calibration_time'] =  datetime.datetime.now().strftime('%Y-%m-%d__%H:%M:%S')

       # load calibration file
       filename = get_file_path(self.robot_name, model_object.name) #TODO getpath yerine get_name olmali

       if not os.path.isfile(filename):
           os.mknod(filename)
       rospy.loginfo('writing the YAML file to: [{}]'.format(filename))
       yaml_write_to_file(yaml_dict, filename)



if __name__ == '__main__':
    calib=calib()
