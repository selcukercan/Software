#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import datetime
import os
import os.path
import numpy as np
from scipy.optimize import minimize

from duckietown_utils.yaml_wrap import (yaml_load_file, yaml_write_to_file)
from calibration.data_preperation_utils import DataPreparation
from calibration.data_preperation_utils import load_pickle, save_pickle
from calibration.model_library import model_generator, simulate
from calibration.data_adapter_utils import *
from calibration.plotting_utils import *
from calibration.metrics import *
from calibration.utils import *

# TODO: why cost fn plot values are different from brute-force param space exploration?

class calib():
    def __init__(self):
        # flow-control parameters
        PREPARE_CALIBRATION_DATA_FOR_OPTIMIZATION = True
        DEBUG = True
        self.save_experiment_results = True

        # initialize the node
        rospy.init_node('calibration', anonymous=True)

        # configure the results directory where the plots and optimization outcome etc will be used.
        package_root = get_package_root("calibration")
        self.results_dir = create_results_dir(package_root)

        # namespace variables
        if not DEBUG:
            host_package = rospy.get_namespace()  # as defined by <group> in launch file
        else:
            host_package = "/mete/calibration/"

        self.node_name = 'kinematic_calibration'  # node name , as defined in launch file
        self.host_package_node = host_package + self.node_name

        # parameters
        self.rosparam_to_program()

        # topics of interest
        self.top_wheel_cmd_exec = "/" + self.robot_name + "/wheels_driver_node/wheels_cmd_executed"
        self.top_robot_pose = "/" + self.robot_name + "/apriltags2_ros/publish_detections_in_local_frame/tag_detections_local_frame"

        # load data for use in optimization
        experiments, data_raw = self.load_fitting_data_routine()

        # construct a model by specifying which model to use
        model_object = model_generator(self.model_type)

        # optimization results-related
        self.param_hist = self.init_param_hist(model_object.model_params)
        self.cost_fn_val_list = []

        # brute-force cost calculation and plotting over parameter-space
        #cost, params_space_list = self.cost_function_over_param_space(model_object, experiments)
        #param_space_cost_plot(cost, params_space_list)

        # optimization settings - for details refer to "https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.minimize.html"
        # see if there already is a yaml file for the model we can use
        model_param_dict = read_param_from_file(self.robot_name, model_object)
        if model_param_dict is not None:
            self.p0 = dict_to_ordered_array(model_object, model_param_dict) # if you have one, use these values as your initial guesses.
        else:
            self.p0 = dict_to_ordered_array(model_object, model_object.get_param_initial_guess_dict()) # otherwise use the default initial guesses as defined in the class of our model choice
            rospy.logwarn('[{}] using default initial guesses defined in model {}'.format('kinematic_calibration', model_object.name))

        # inspect the 2D path vehicle followed
        path_plot(experiments['ramp_up_2019_01_19_15_04_Nstep_120.0_vFin_0.5_pp'], plot_name='measured_traj')

        """
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

        # load and process the experiment data to be used for testing the model
        test_dataset, test_data_raw = self.load_testing_data_routine()

        # make predictions with the optimization results
        #self.model_predictions(model_object, experiments, popt)
        self.model_predictions(model_object, test_dataset, popt, plot_title= "Model: {} DataSet: {}".format(model_object.name, test_data_raw.exp_name))

        # write to the kinematic calibration file
        self.write_calibration(model_object, popt)
        """
    def cost_function_over_param_space(self, model_object, experiments):
        cost=[]
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
                    i+=1
        return cost, params_space_list

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
                      plot_title=plot_title,
                      save=self.save_experiment_results,
                      save_dir=self.results_dir)

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

    @staticmethod
    def init_param_hist(model_params):
        param_hist = {}
        for param_name in model_params:
            param_hist[param_name] = []
        return param_hist

    def update_param_hist(self, model_ordered_param_list, p):
        for i, param_name in enumerate(model_ordered_param_list):
            self.param_hist[param_name].append(p[i])

    def load_fitting_data_routine(self):
        # training data set container
        experiments = input_folder_to_experiment_dict(self.path_training_data)

        source = 'folder'
        load_from_pickle = 'test_run'
        save_to_pickle = True # True/False
        set_name = 'test_run'

        # load and process experiment data to be used in the optimization
        if source == 'folder':
            for i, exp in enumerate(experiments.keys()):
                data_raw = DataPreparation(input_bag=experiments[exp]['path'],
                                           top_wheel_cmd_exec=self.top_wheel_cmd_exec,
                                           top_robot_pose=self.top_robot_pose,
                                           exp_name='Training Data {}: {}'.format(i + 1,exp))
                experiments[exp]['wheel_cmd_exec'], experiments[exp]['robot_pose'], experiments[exp]['timestamp']= data_raw.process_raw_data() # bring data set to format usable by the optimizer
            save_pickle(object=experiments, save_as=set_name)
        elif source == 'pickle':
            experiments = load_pickle(load_from_pickle)
        else:
            rospy.logfatal('[{}] is not a valid source type'.format(source))

        return experiments, data_raw

    def load_testing_data_routine(self):
        # test data set container
        path_test_data = '/home/selcuk/test_bags/test_set'
        test_dataset = input_folder_to_experiment_dict(path_test_data)

        for i, exp in enumerate(test_dataset.keys()):
            test_data_raw = DataPreparation(input_bag=test_dataset[exp]['path'],
                                            top_wheel_cmd_exec=self.top_wheel_cmd_exec,
                                            top_robot_pose=self.top_robot_pose,
                                            exp_name='Test Data {}: {}'.format(i+1, exp))
            test_dataset[exp]['wheel_cmd_exec'], test_dataset[exp]['robot_pose'], test_dataset[exp]['timestamp'] = test_data_raw.process_raw_data()  # bring data set to format usable by the optimizer

        return test_dataset, test_data_raw

    def rosparam_to_program(self):
        # rosparam server addresses
        param_veh = self.host_package_node + '/' + "veh"
        param_folder_path = self.host_package_node + '/' + "folder_path"
        param_model_type = self.host_package_node + '/' + "model"

        # rosparam values
        self.robot_name = defined_ros_param(param_veh)
        self.path_training_data = defined_ros_param(param_folder_path)
        self.model_type = defined_ros_param(param_model_type)


if __name__ == '__main__':
    calib=calib()
