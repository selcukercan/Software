#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import datetime
import os
import os.path
import numpy as np
from scipy.optimize import minimize
from copy import deepcopy
from duckietown_utils.yaml_wrap import (yaml_load_file, yaml_write_to_file)
from calibration.data_preperation_utils import DataPreparation
from calibration.data_preperation_utils import load_pickle, save_pickle
from calibration.model_library import model_generator, simulate, simulate_horizan
from calibration.data_adapter_utils import *
from calibration.plotting_utils import *
from calibration.utils import *
from calibration.cost_function_library import *

"""
TODO:

1) Generate a result yaml.

* conf
* used model
* initial kinetic parameter/whether was default
* train / validation files
* optimization results
    - minimize output + metric calculations + time
* computer_name / platform info
* camera calibrations
* vehicle name

2) Collect all the plots under results folder


"""
class calib():
    def __init__(self):
        # initialize the node
        rospy.init_node('calibration', anonymous=True)

        # configure the results directory where the plots and optimization outcome etc will be used.
        package_root = get_package_root("calibration")
        self.results_dir = create_results_dir(package_root)
        self.tmp_dir = safe_create_dir(os.path.join(package_root, "tmp"))

        self.conf = yaml_load_file(package_root + '/config.yaml', plain_yaml=True)

        # flow-control parameters
        DEBUG = self.conf['debug']
        self.show_plots = self.conf['show_plots']
        self.save_experiment_results = self.conf['save_experiment_results']

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
        self.measurement_coordinate_frame = self.conf['express_measurements_in']
        experiments = self.load_dataset("Training", self.path_training_data)
        # load and process the experiment data to be used for testing the model
        validation_dataset = self.load_dataset("Validation", self.path_validation_data)

        #save_gzip()

        # construct a model by specifying which model to use
        model_object = model_generator(self.model_type, self.measurement_coordinate_frame)

        # optimization settings - for details refer to "https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.minimize.html"
        # see if there already is a yaml file for the model we can use
        model_param_dict = read_param_from_file(self.robot_name, model_object)
        if model_param_dict is not None:
            self.p0 = dict_to_ordered_array(model_object, model_param_dict) # if you have one, use these values as your initial guesses.
        else:
            self.p0 = dict_to_ordered_array(model_object, model_object.get_param_initial_guess_dict()) # otherwise use the default initial guesses as defined in the class of our model choice
            rospy.logwarn('[{}] using default initial guesses defined in model {}'.format('kinematic_calibration', model_object.name))

        # inspect the 2D path vehicle followed
        exp = "ramp_up_2019_02_08_19_47_Nstep_100.0_vFin_0.5_pp"
        #multiplot(states_list=[data1, data2], plot_title='Deneme', save=False, save_dir="")

        # use the parameter bounds defined in the class of our model choice
        self.bounds = model_object.get_param_bounds_list()

        # optimization results-related
        self.param_hist = self.init_param_hist(model_object.model_params)
        self.cost_fn_val_list = []
        # self.delta =  0.00

        # define the type of metric to use while constructing the cost
        self.req_metric = self.conf['cost_function_type']
        self.metric = metric_selector(self.req_metric)

        # brute-force cost calculation and plotting over parameter-space
        #cost, params_space_list = self.cost_function_over_param_space(model_object, experiments)
        #param_space_cost_plot(cost, params_space_list)

        # run the optimization problem
        popt = self.nonlinear_model_fit(model_object, experiments)


        # parameter converge plots and cost fn
        if self.show_plots: param_convergence_plot(self.param_hist)
        if self.show_plots: simple_plot(range(len(self.cost_fn_val_list)), self.cost_fn_val_list, 'Cost Function')

        # make predictions with the optimization results
        self.model_predictions(model_object, validation_dataset, popt, plot_title="Model: {} DataSet: {}".format(model_object.name, exp))

        """
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
        for exp_name in experiments.keys():
            exp_data = experiments[exp_name].data
            t = exp_data['timestamp']
            x = exp_data['robot_pose']
            u = exp_data['wheel_cmd_exec']
            #x0 = x[:,0]

            #simulate the model
            x_sim = simulate(model_object, t, x, u, p) # states for a particular p set
            obj_cost = calculate_cost(x, x_sim, self.metric)

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
            exp_data = experiments[exp_name].data
            t = exp_data['timestamp']
            x = exp_data['robot_pose']
            u = exp_data['wheel_cmd_exec']
            x0 = x[:, 0]

            # simulate the model
            # states for a particular p set
            x_sim_opt = simulate(model_object, t, x, u, popt)
            x_sim_init = simulate(model_object, t, x, u, self.p0)

            x_sim_opt_osap = simulate_horizan(model_object, t, x0, u, popt)
            x_sim_init_osap = simulate_horizan(model_object, t, x0, u, self.p0)

            simple_plot(None, x[1,:] / x_sim_opt_osap[1,:])
            # calculate the error metric
            error = calculate_cost(x, x_sim_opt, self.metric)

            print('\nModel Performance Evaluation:\nModel Name: {}\n Metric Type: {} Value: {}'.format(exp_name, self.metric , error))

            if self.show_plots:
                multiplot(states_list=[x, x_sim_init, x_sim_opt],
                      input_list=[u,u,u],
                      time_list=[t,t,t],
                      experiment_name_list=[exp_name + '_measurement', exp_name + '_simulated_init', exp_name + '_simulated_optimal'],
                      plot_title= "OSAP " + plot_title,
                      save=self.save_experiment_results,
                      save_dir=self.results_dir)

            if self.show_plots:
                multiplot(states_list=[x, x_sim_init_osap, x_sim_opt_osap],
                      input_list=[u,u,u],
                      time_list=[t,t,t],
                      experiment_name_list=[exp_name + '_measurement', exp_name + '_simulated_init', exp_name + '_simulated_optimal'],
                      plot_title= "N-Horizan " + plot_title,
                      save=self.save_experiment_results,
                      save_dir=self.results_dir)

            """
            multi_path_plot([exp_data, x_sim_init, x_sim_opt], ["measurement", "initial_values", "optimal_values"] )
            """

    def write_calibration(self, model_object, popt):
       # Form yaml content to write
       yaml_dict = {}
       for i, param in enumerate(model_object.param_ordered_list):
           yaml_dict[param] = popt[i].item()
       yaml_dict['calibration_time'] = datetime.datetime.now().strftime('%Y-%m-%d__%H:%M:%S')

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

    @staticmethod
    def save_gzip(file_name, processed_dataset):
        data_file = os.path.join(file_name + '_train_val')
        pickle.dump(processed_dataset, gzip.open(data_file, "wb"))

    def update_param_hist(self, model_ordered_param_list, p):
        for i, param_name in enumerate(model_ordered_param_list):
            self.param_hist[param_name].append(p[i])

    def load_dataset(self, dataset_prefix, path_to_dataset):
        # training data set container
        experiments = input_folder_to_experiment_dict(path_to_dataset)

        source = 'folder'
        load_from_pickle = 'test_run'
        save_to_pickle = self.conf["save_to_pickle"]
        set_name = 'test_run'

        # load and process experiment data to be used in the optimization
        if source == 'folder':
            for i, exp in enumerate(experiments.keys()):
                experiments[exp] = DataPreparation(input_bag=experiments[exp]['path'],
                                           top_wheel_cmd_exec=self.top_wheel_cmd_exec,
                                           top_robot_pose=self.top_robot_pose,
                                           exp_name= dataset_prefix + ' Data {}: {}'.format(i + 1,exp),
                                           measurement_coordinate_frame=self.measurement_coordinate_frame)
                #experiments[exp]['wheel_cmd_exec'], experiments[exp]['robot_pose'], experiments[exp]['timestamp']= data_raw.process_raw_data() # bring data set to format usable by the optimizer
            #rospy.logwarn(self.tmp_dir)
            if save_to_pickle:
                save_pickle(object=experiments, save_as= os.path.join(self.tmp_dir, set_name))
        elif source == 'pickle':
            experiments = load_pickle(load_from_pickle)
        else:
            rospy.logfatal('[{}] is not a valid source type'.format(source))
        return experiments
    """
    def load_validation_data_routine(self):
        # validation data set container
        validation_dataset = input_folder_to_experiment_dict(self.path_validation_data)

        for i, exp in enumerate(validation_dataset.keys()):
            validation_data_raw = DataPreparation(input_bag=validation_dataset[exp]['path'],
                                            top_wheel_cmd_exec=self.top_wheel_cmd_exec,
                                            top_robot_pose=self.top_robot_pose,
                                            exp_name='Validation Data {}: {}'.format(i+1, exp),
                                            measurement_coordinate_frame=self.measurement_coordinate_frame)
            validation_dataset[exp]['wheel_cmd_exec'], validation_dataset[exp]['robot_pose'], validation_dataset[exp]['timestamp'] = validation_data_raw.process_raw_data()  # bring data set to format usable by the optimizer

        return validation_dataset, validation_data_raw
    """
    def rosparam_to_program(self):
        # rosparam server addresses
        param_veh = self.host_package_node + '/' + "veh"
        param_train_path = self.host_package_node + '/' + "train_path"
        param_validation_path = self.host_package_node + '/' + "validation_path"
        param_model_type = self.host_package_node + '/' + "model"

        # rosparam values
        self.robot_name = defined_ros_param(param_veh)
        self.path_training_data = defined_ros_param(param_train_path)
        self.path_validation_data = defined_ros_param(param_validation_path)
        self.model_type = defined_ros_param(param_model_type)


if __name__ == '__main__':
    calib=calib()
