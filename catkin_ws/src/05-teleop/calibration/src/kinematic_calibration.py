#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import datetime
import os
import os.path

from scipy.optimize import minimize
from os.path import join

from duckietown_utils.yaml_wrap import (yaml_load_file, yaml_write_to_file)
from calibration.data_preperation_utils import DataPreparation
from calibration.data_preperation_utils import load_pickle, save_pickle
from calibration.model_library import model_generator, simulate
from calibration.data_adapter_utils import *
from calibration.plotting_utils import *
from calibration.metrics import *
from calibration.utils import *


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
        param_model_type = host_package_node + '/' + "model"
        model_type = defined_ros_param(param_model_type)

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
            for i, exp in enumerate(experiments.keys()):
                data_raw = DataPreparation(input_bag=experiments[exp]['path'],
                                           top_wheel_cmd_exec=top_wheel_cmd_exec,
                                           top_robot_pose=top_robot_pose,
                                           exp_name='Training Data {}: {}'.format(i + 1,exp))
                experiments[exp]['wheel_cmd_exec'], experiments[exp]['robot_pose'], experiments[exp]['timestamp']= data_raw.process_raw_data() # bring data set to format usable by the optimizer
            save_pickle(object=experiments, save_as=set_name)
        elif source == 'pickle':
            experiments = load_pickle(load_from_pickle)
        else:
            rospy.logfatal('[{}] is not a valid source type'.format(source))

        # construct a model by specifying which model to use
        model_object = model_generator(model_type)

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

        # optimization results-related
        self.param_hist = self.init_param_hist(model_object.model_params)
        self.cost_fn_val_list = []
        # self.delta =  0.00

        # run the optimization problem
        popt = self.nonlinear_model_fit(model_object, experiments)

        # parameter converge plots and cost fn
        param_convergence_plot(self.param_hist)
        simple_plot(range(len(self.cost_fn_val_list)), self.cost_fn_val_list, 'Cost Function')

        # test data set container
        test_dataset = self.input_folder_to_experiment_dict(path_test_data)

        # load and process the experiment data to be used for testing the model
        for i, exp in enumerate(test_dataset.keys()):
            test_data_raw = DataPreparation(input_bag=test_dataset[exp]['path'],
                                            top_wheel_cmd_exec=top_wheel_cmd_exec,
                                            top_robot_pose=top_robot_pose,
                                            exp_name='Test Data {}: {}'.format(i+1, exp))
            test_dataset[exp]['wheel_cmd_exec'], test_dataset[exp]['robot_pose'], test_dataset[exp]['timestamp'] = test_data_raw.process_raw_data()  # bring data set to format usable by the optimizer

        #self.model_predictions(model_object, experiments, popt)
        self.model_predictions(model_object, test_dataset, popt, plot_title= "Model: {} DataSet: {}".format(model_object.name, test_data_raw.exp_name))

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

        self.update_param_hist(model_object.param_ordered_list, p)
        self.cost_fn_val_list.append(obj_cost)
        return obj_cost

    def nonlinear_model_fit(self, model_object, experiments):
        """ for more information on scipy.optimize.min fn:
        https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.minimize.html
        """
        
        rospy.loginfo('started the nonlinear optimization ... ')
        # Actual Parameter Optimization/Fitting
        # Minimize the error between the model predictions and position estimations
        result = minimize(self.cost_function, self.p0, args=(model_object, experiments), bounds=self.bounds)
        print('[BEGIN] Optimization Result\n {} [END] Optimization Result\n'.format(result))

        return result.x

    def model_predictions(self, model_object, experiments, popt, plot_title=''):
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

            # calculate the error metric
            exp_mse = norm_rmse(x, x_sim_opt)

            print('\nModel Performance Evaluation:\nModel Name: {}\nnormRMSE: {}'.format(exp_name, exp_mse))


            multiplot(states_list=[x, x_sim_init, x_sim_opt],
                      input_list=[u,u,u],
                      time_list=[t,t,t],
                      experiment_name_list=[exp_name + '_measurement', exp_name + '_simulated_init', exp_name + '_simulated_optimal'],
                      mode = 'single_view',
                      plot_title=plot_title)

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

    def init_param_hist(self, model_params):
        param_hist = {}
        for param_name in model_params:
            param_hist[param_name] = []
        return param_hist
    def update_param_hist(self, model_ordered_param_list, p):
        for i, param_name in enumerate(model_ordered_param_list):
            self.param_hist[param_name].append(p[i])




if __name__ == '__main__':
    calib=calib()
