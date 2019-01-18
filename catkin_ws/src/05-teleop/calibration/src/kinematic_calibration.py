#!/usr/bin/env python
# -*- coding: utf-8 -*-

# first save .bag file as robot_name.bag in duckiefleet/calibrations/sysid

import numpy as np
import rosbag
import rospy
import time
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
        model_object = model_generator('model1')

        # Optimization Settings - for details refer to "https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.minimize.html"
        self.p0 = [1,1] # initial guesses for the parameters
        self.bnds = ((None, None), (None, None)) # bounds for the parameters
        # self.delta =  0.00

        # run the optimization problem
        popt= self.nonlinear_model_fit(model_object, experiments)

        # test data set container
        test_dataset = self.input_folder_to_experiment_dict(path_test_data)

        # load and process the experiment data to be used in for testing the model
        for exp in test_dataset.keys():
            data_raw = DataPreparation(input_bag=test_dataset[exp]['path'], top_wheel_cmd_exec=top_wheel_cmd_exec,top_robot_pose=top_robot_pose)
            test_dataset[exp]['wheel_cmd_exec'], test_dataset[exp]['robot_pose'], test_dataset[exp]['timestamp'] = data_raw.process_raw_data()  # bring data set to format usable by the optimizer

        self.model_predictions(model_object, experiments, popt)
        #self.model_predictions(model_object, test_dataset, popt)
        print('selcuk')
        #self.plots()
        # write to the kinematic calibration file
        #self.write_calibration()

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

                """
                #best so far
                obj_cost += (
                             ((x_sim[0, i] - x[0, i])) ** 2 +
                             ((x_sim[1, i] - x[1, i])) ** 2 +
                             0.0000001 * ((x_sim[2, i] - x[2, i])) ** 2
                            )
                """
                obj_cost += (
                             ((x_sim[0, i] - x[0, i]) / range_x) ** 2 +
                             ((x_sim[1, i] - x[1, i]) / range_y) ** 2  +
                             ((x_sim[2, i] - x[2, i]) / range_yaw) ** 2
                            )

        return obj_cost

    def nonlinear_model_fit(self, model_object, experiments):
        print 'IN NONLINEAR MODEL FIT'
        # Actual Parameter Optimization/Fitting
        # Minimize the least squares error between the model prediction
        result = minimize(self.cost_function, self.p0, args=(model_object, experiments), bounds=self.bnds)
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

    def write_calibration(self):
       '''Load kinematic calibration file'''
       home = expanduser("~")
       filename = (home + "/duckietown_sysid/kinematics/" + self.robot_name + ".yaml")
       if not os.path.isfile(filename):
           logger.warn("no kinematic calibration parameters for {}, taking some from default".format(self.robot_name))
           filename = (home+"/duckietown_sysid/kinematics/default.yaml")
           if not os.path.isfile(filename):
               logger.error("can't find default either, something's wrong, is the duckiefleet root correctly set?")
           else:
               data = yaml_load_file(filename)
       else:
           rospy.loginfo("Loading some kinematic calibration parameters of " + self.robot_name)
           data = yaml_load_file(filename)
       logger.info("Loaded homography for {}".format(os.path.basename(filename)))

       # Load some of the parameters that will not be changed
       param_k        = data['k']
       param_limit    = data['limit']
       param_radius   = data['radius']
       baseline   = data['baseline']
       # simply to increase readability

       gain = self.fit_['gain']
       trim = self.fit_['trim']

       # Write to yaml
       #datasave = {  # This is similar to the inverse_kinematics_node, but it did not work...
       #    "calibration_time": time.strftime("%Y-%m-%d-%H-%M-%S"),
       #    "gain": gain,
       #    "trim": trim,
       #    "baseline": baseline,
       #    "radius": param_radius,
       #    "k": param_k,
       #    "limit": param_limit,
       #}
       datasave = \
       "calibration_time: {}".format(time.strftime("%Y-%m-%d-%H-%M-%S")) + \
       "\ngain: {}".format(gain) + \
       "\ntrim: {}".format(trim) + \
       "\nbaseline: {}".format(baseline) + \
       "\nradius: {}".format(param_radius) + \
       "\nk: {}".format(param_k) + \
       "\nlimit: {}".format(param_limit)


       print("\nThe Estimated Kinematic Calibration is:")
       print("gain     = {}".format(gain))
       print("trim     = {}".format(trim))

       # Write to yaml file
       filename = ( home + "/duckietown_sysid/kinematics/" + self.robot_name + ".yaml")
       with open(filename, 'w') as outfile:
           outfile.write(datasave)
           #outfile.write(yaml.dump(datasave, default_flow_style=False))  # This did not work and gave very weird results

       print("Saved Parameters to " + self.robot_name + ".yaml" )

       print("\nPlease check the plots and judge if the parameters are reasonable.")
       print("Once done inspecting the plot, close them to terminate the program.")


if __name__ == '__main__':
    calib=calib()