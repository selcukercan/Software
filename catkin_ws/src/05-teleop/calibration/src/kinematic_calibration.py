#!/usr/bin/env python
# -*- coding: utf-8 -*-

import datetime
import os
import os.path

# python imports
import rospy
from calibration.cost_function_library import *
from calibration.data_adapter_utils import *
# ros-package-level imports
from calibration.data_preperation_utils import DataPreparation
from calibration.data_preperation_utils import load_pickle, save_pickle
from calibration.model_library import model_generator, simulate
from calibration.plotting_utils import *
from calibration.utils import work_space_settings, get_workspace_param, \
    defined_ros_param, input_folder_to_experiment_dict, read_param_from_file, get_file_path,  defaulted_param_load, \
    pack_results
# duckietown imports
from duckietown_utils.yaml_wrap import yaml_load_file, yaml_write_to_file
from scipy.optimize import minimize


class calib():
    def __init__(self):
        # initialize the node
        rospy.init_node('calibration', anonymous=True)

        # configure the results directory where the plots and optimization outcome etc will be used.
        work_space_settings()
        self.results_dir = get_workspace_param("results_dir")
        self.tmp_dir = get_workspace_param("tmp_dir")
        self.conf = yaml_load_file(get_workspace_param("path_to_config_file"), plain_yaml=True)

        # flow-control parameters
        DEBUG = self.conf['debug']
        self.show_plots = self.conf['show_plots']
        self.save_experiment_results = self.conf['save_experiment_results']

        # namespace variables
        if not DEBUG:
            host_package = rospy.get_namespace()  # as defined by <group> in launch file
        else:
            robot_name_from_conf = self.conf['vehicle_name']
            host_package = "/" + robot_name_from_conf + "/calibration/"

        self.node_name = 'kinematic_calibration'  # node name , as defined in launch file
        self.host_package_node = host_package + self.node_name

        # set parameters / train and or validate options
        self.rosparam_to_program()

        # topics of interest
        self.top_wheel_cmd_exec = "/" + self.robot_name + "/wheels_driver_node/wheels_cmd_executed"
        self.top_robot_pose_apriltag = "/" + self.robot_name + '/apriltags2_ros/publish_detections_in_local_frame/tag_detections_array_local_frame'
        self.top_robot_pose_lane_filter = "/" + self.robot_name + "/lane_filter_node/lane_pose"

        self.initial_param_vs_optimal_param = False


        self.measurement_coordinate_frame = self.conf['express_measurements_in']
        # construct a model by specifying which model to use
        model_object = model_generator(self.model_type, self.measurement_coordinate_frame)

        if self.do_train:
            # load data for use in optimization
            experiments = self.load_dataset("Training", self.path_training_data, localization_type='apriltag')
            self.training_routine(model_object, experiments)
        else:
            rospy.logwarn('[{}] not training the model'.format(self.node_name))

        if self.do_validate:
            # load and process the experiment data to be used for testing the model
            validation_dataset = self.load_dataset("Validation", self.path_validation_data, localization_type='apriltag')
            self.validation_routine(model_object, validation_dataset)
        else:
            rospy.logwarn('[{}] not validating the model'.format(self.node_name))

        pack_results(self.results_dir)

        """
        #add_x_dot_estimate_to_dataset(experiments, "train")
        """

    # train
    def training_routine(self, model_object, experiments):
        """ train the model and write the results to the YAML file"""
        # see if there already is a yaml file for the model we can use
        self.p0 = defaulted_param_load(model_object, self.robot_name)

        # use the parameter bounds defined in the class of our model choice
        self.bounds = model_object.get_param_bounds_list()

        # optimization process monitoring
        self.param_hist = self.init_param_hist(model_object.model_params)
        self.cost_fn_val_list = []

        # define the type of metric to use while constructing the cost
        self.req_metric = self.conf['cost_function_type']
        self.metric = metric_selector(self.req_metric)

        # run the optimization problem
        popt = self.nonlinear_model_fit(model_object, experiments)

        # parameter converge plots and cost fn
        if self.show_plots: param_convergence_plot(self.param_hist, save_dir=self.results_dir)
        if self.show_plots: simple_plot(range(len(self.cost_fn_val_list)), self.cost_fn_val_list,
                                        plot_name='Cost Function', save_dir=self.results_dir)

        # write to the kinematic calibration file
        self.write_calibration(model_object, popt)

    # Data Operations
    def load_dataset(self, dataset_name, path_to_dataset, localization_type=None):
        """
        loads the rosbags from path_to_dataset

        Args:
            dataset_name: name of the dataset, used in plot title.
            path_to_dataset: path to folder containing rosbags.

        Returns:
            experiments: experiment data ready to be fed to the optimization.

        Notes:
             see include/calibration/data_preperation_utils.py for detailed information about "experiments".
        """
        # loads the experiments from the path_to_dataset
        experiments = input_folder_to_experiment_dict(path_to_dataset)
        pose_topic = self.select_pose_topic(localization_type)

        # options
        source = 'folder'  # change to "pickle" to load a previously dumped experiment data
        save_to_pickle = self.conf["save_to_pickle"]  # interface through the config file

        # load and process experiment data to be used in the optimization
        if source == 'folder':
            for i, exp in enumerate(experiments.keys()):
                experiments[exp] = DataPreparation(input_bag=experiments[exp]['path'],
                                                   top_wheel_cmd_exec=self.top_wheel_cmd_exec,
                                                   top_robot_pose=pose_topic,
                                                   exp_name=dataset_name + ' Data {}: {}'.format(i + 1, exp),
                                                   measurement_coordinate_frame=self.measurement_coordinate_frame,
                                                   localization_method=localization_type)
            if save_to_pickle:
                set_name = 'test_run'
                save_pickle(object=experiments, save_as=os.path.join(self.tmp_dir, set_name))
        elif source == 'pickle':
            load_from_pickle = 'test_run'
            experiments = load_pickle(load_from_pickle)
        else:
            rospy.logfatal('[{}] is not a valid source type'.format(source))
        return experiments

    # Optimization-Related Functions
    def cost_function(self, p, model_object, experiments):
        """
        calculates t
        """
        for exp_name in experiments.keys():
            exp_data = experiments[exp_name].data
            t = exp_data['timestamp']
            x = exp_data['robot_pose']
            u = exp_data['wheel_cmd_exec']
            # x0 = x[:,0]

            # simulate the model
            x_sim = simulate(model_object, t, x, u, p)  # states for a particular p set
            obj_cost = calculate_cost(x, x_sim, self.metric)

        self.update_param_hist(model_object.param_ordered_list, p)
        self.cost_fn_val_list.append(obj_cost)
        return obj_cost

    def nonlinear_model_fit(self, model_object, experiments):
        """
        for more information on scipy.optimize.min fn:
        https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.minimize.html
        """
        rospy.loginfo('started the nonlinear optimization ... ')
        # Actual Parameter Optimization/Fitting
        # Minimize the error between the model predictions and position estimations
        result = minimize(self.cost_function, self.p0, args=(model_object, experiments), bounds=self.bounds)
        print('[BEGIN] Optimization Result\n {} [END] Optimization Result\n'.format(result))

        return result.x

    def validation_routine(self, model_object, experiments):
        """ performs validation with the specified model and its latest parameters """
        popt = defaulted_param_load(model_object, self.robot_name)

        for exp_name in experiments.keys():
            exp_data = experiments[exp_name].data
            t = exp_data['timestamp']
            x = exp_data['robot_pose']
            u = exp_data['wheel_cmd_exec']

            # simulate the model
            # states for a particular p set
            x_sim_opt = simulate(model_object, t, x, u, popt)

            # calculate the error metric
            error = calculate_cost(x, x_sim_opt, self.metric)

            print('\nModel Performance Evaluation:\nModel Name: {}\nMetric Type: {} Value: {}\n'.format(exp_name,
                                                                                                        self.metric,
                                                                                                        error))

            if self.show_plots:
                multiplot(states_list=[x, x_sim_opt],
                          input_list=[u, u],
                          time_list=[t, t],
                          experiment_name_list=[exp_name + '_measurement', exp_name + '_simulated_optimal'],
                          plot_title="One Step Ahead Predictions for Model: {} Dataset: {}".format(model_object.name,
                                                                                                   exp_name),
                          save=self.save_experiment_results)

            if self.initial_param_vs_optimal_param:
                x_sim_init = simulate(model_object, t, x, u, self.p0)
                if self.show_plots:
                    multiplot(states_list=[x, x_sim_init, x_sim_opt],
                              input_list=[u, u, u],
                              time_list=[t, t, t],
                              experiment_name_list=[exp_name + '_measurement', exp_name + '_simulated_init',
                                                    exp_name + '_simulated_optimal'],
                              plot_title="One Step Ahead Predictions for Model: {} Dataset: {}".format(model_object.name,
                                                                                                       exp_name),
                              save=self.save_experiment_results)
                """
                x0 = x[:, 0]
                x_sim_opt_osap = simulate_horizan(model_object, t, x0, u, popt)
                x_sim_init_osap = simulate_horizan(model_object, t, x0, u, self.p0)
    
                if self.show_plots:
                    multiplot(states_list=[x, x_sim_init_osap, x_sim_opt_osap],
                          input_list=[u,u,u],
                          time_list=[t,t,t],
                          experiment_name_list=[exp_name + '_measurement', exp_name + '_simulated_init', exp_name + '_simulated_optimal'],
                          plot_title= "N-Horizan " + plot_title,
                          save=self.save_experiment_results,
                          save_dir=self.results_dir)
    
    
                multi_path_plot([exp_data, x_sim_init, x_sim_opt], ["measurement", "initial_values", "optimal_values"] )
                """


    # Save results
    def write_calibration(self, model_object, popt):
        """
        generates a YAML file including optimal model parameters.

        Args:
            model_object: an instance of a model class
            popt: optimal model parameters

        Returns:
            None

        Notes:
            to learn more about model classes, please refer to /include/calibration/model_library
        """
        # Form yaml content to write
        yaml_dict = {}
        for i, param in enumerate(model_object.param_ordered_list):
            yaml_dict[param] = popt[i].item()
        yaml_dict['calibration_time'] = datetime.datetime.now().strftime('%Y-%m-%d__%H:%M:%S')

        # load calibration file
        filename = get_file_path(self.robot_name, model_object.name)

        if not os.path.isfile(filename):
            os.mknod(filename)
        else:
            from shutil import copyfile
            file_dir = os.path.dirname(filename)
            new_file_name = self.robot_name + "_" + model_object.name + "_previous.yaml"
            copyfile(filename, os.path.join(file_dir, new_file_name))
        rospy.loginfo('writing the YAML file to: [{}]'.format(filename))
        yaml_write_to_file(yaml_dict, filename)

    # Utility Functions
    def rosparam_to_program(self):
        """
        a convenience function to increase readability
        """
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

        self.do_train = True
        self.do_validate = True
        if self.path_training_data == 'for_good_reason':
            self.do_train = False
        if self.path_validation_data == 'for_good_reason':
            self.do_validate = False

    @staticmethod
    def init_param_hist(model_params):
        param_hist = {}
        for param_name in model_params:
            param_hist[param_name] = []
        return param_hist

    def update_param_hist(self, model_ordered_param_list, p):
        for i, param_name in enumerate(model_ordered_param_list):
            self.param_hist[param_name].append(p[i])

    def select_pose_topic(self,localization_type):
        if localization_type == 'apriltag':
            return self.top_robot_pose_apriltag
        elif localization_type == 'lane_filter':
            return self.top_robot_pose_lane_filter
        else:
            rospy.logfatal('BOOM')


if __name__ == '__main__':
    calib = calib()

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
"""
