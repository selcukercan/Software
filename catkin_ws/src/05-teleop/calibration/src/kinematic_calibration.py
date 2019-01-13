#!/usr/bin/env python
# -*- coding: utf-8 -*-

# first save .bag file as robot_name.bag in duckiefleet/calibrations/sysid


import numpy as np
import rosbag
import rospy

import os.path
from duckietown_utils.path_utils import get_ros_package_path
from duckietown_utils.yaml_wrap import (yaml_load_file, yaml_write_to_file)
from duckietown_utils import (logger, get_duckiefleet_root)
#from duckietown_utils import rgb_from_ros

import time
import os
import math
import matplotlib.pyplot as plt
from matplotlib.pyplot import pause,imshow
from scipy.optimize import curve_fit
from scipy.optimize import minimize
from scipy.integrate import odeint
from mpl_toolkits.mplot3d import Axes3D
import yaml
from os.path import expanduser
from os.path import join

from calibration.data_preperation_utils import DataPreparation
from calibration.data_preperation_utils import load_pickle, save_pickle

from calibration.model_library import model_generator, simulate
from calibration.data_adapter_utils import *

from calibration.plotting_utils import *

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
        self.robot_name = rospy.get_param(param_veh)
        param_folder_path = host_package_node + '/' + "folder_path"
        input_folder = rospy.get_param(param_folder_path)

        experiments = self.input_folder_to_experiment_dict(input_folder)

        self.Ts = 1 / 30.0
        self.d = 0.6
        self.p0 = [1,1,0]

        # topics of interest
        top_wheel_cmd_exec = "/" + self.robot_name + "/wheels_driver_node/wheels_cmd_executed"
        top_robot_pose = "/" + self.robot_name + "/apriltags2_ros/publish_detections_in_local_frame/tag_detections_local_frame"

        source = 'folder'
        load_from_pickle = 'test_run'
        save_to_pickle = True # True/False
        set_name = 'test_run'

        #self.delta =  0.00
        if source == 'folder':
            for exp in experiments.keys():
                data_raw = DataPreparation(input_bag=experiments[exp]['path'], top_wheel_cmd_exec=top_wheel_cmd_exec, top_robot_pose=top_robot_pose)
                experiments[exp]['wheel_cmd_exec'], experiments[exp]['robot_pose'], experiments[exp]['timestamp']= data_raw.process_raw_data() # bring data set to format usable by the optimizer
            save_pickle(object=experiments, save_as=set_name)
        elif source == 'pickle':
            experiments = load_pickle(load_from_pickle)
        else:
            rospy.logfatal('[{}] is not a valid source type'.format(source))

        # define which model to use
        model_object = model_generator('model1')


        self.cost_fn_plot_measurement = True
        popt= self.nonlinear_model_fit(model_object, experiments)

        self.model_predictions(model_object, experiments, popt)
        print('selcuk')
        #self.plots()
        # write to the kinematic calibration file
        #self.write_calibration()


    '''
    def forwardEuler(self,s_cur, Ts, cmd_right, cmd_left,p):
        c, cl, tr = p

        x_0 = s_cur[0]
        y_0 = s_cur[1]
        yaw_0 = s_cur[2]
        d = self.d

        # velocity term predictions based on differential drive
        vx_pred = c * (cmd_right+cmd_left) * 0.5 + c * tr * (cmd_right-cmd_left)*0.5
        omega_pred = -1*cl * (cmd_right-cmd_left) * 0.5 + -1*cl * tr * (cmd_right+cmd_left) * 0.5
        vy_pred = -1*omega_pred * d

        # forward euler integration
        yaw_pred = (omega_pred) * Ts + yaw_0
        x_pred = (np.cos(yaw_pred) * vx_pred - vy_pred * np.sin(yaw_pred)) * Ts + x_0
        y_pred = 5 * (np.sin(yaw_pred) * vx_pred + vy_pred * np.cos(yaw_pred)) * Ts + y_0

        return np.array([x_pred, y_pred, yaw_pred]).reshape(3)

    def simulate(self,p, cmd_right, cmd_left, s_init,time):
        # States
        ## Note that values take checkerboard as the origin.
        s = np.zeros((len(time),3))

        s[0,0] = s_init[0] # x
        s[0,1] = s_init[1] # y
        s[0,2] = s_init[2] # yaw

        Ts = self.Ts
        s_cur = s[0]

        for i in range(len(time)-1):
            s_next = self.forwardEuler(s_cur, Ts, cmd_right[i],cmd_left[i],p)
            s_cur = np.copy(s_next)
            s[i+1] = s_cur

        return s
    '''

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
            if self.cost_fn_plot_measurement:
                #plot_system(states= x, time=t, experiment_name=exp_name)
                self.cost_fn_plot_measurement = False
            #plot_system(states=x_sim, time=t, experiment_name=exp_name + '_simulated')
            for i in range(len(t)):
                obj_cost += ( ((x_sim[0,i] - x[0,i])) ** 2 +
                              ((x_sim[1,i] - x[1,i])) ** 2 +
                              0.2 * ((x_sim[2, i] - x[2, i])) ** 2
                              )

        return obj_cost

    def nonlinear_model_fit(self, model_object, experiments):
        print 'IN NONLINEAR MODEL FIT'

        p0 = self.p0

        # Actual Parameter Optimization/Fitting
        # Minimize the least squares error between the model prediction
        result = minimize(self.cost_function, p0, args=(model_object, experiments))
        popt = result.x

        print('[BEGIN] Optimization Result\n {} [END] Optimization Result\n'.format(result))

        return popt

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
            multiplot(states_list=[x, x_sim_init, x_sim_opt],
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
    rospy.spin()