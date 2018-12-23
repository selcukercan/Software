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
import plotly.graph_objs as go
import plotly.offline as opy

from calibration.data_preperation_utils import DataPreparation
from calibration.data_preperation_utils import load_pickle, save_pickle

#opy.init_notebook_mode(connected=True)


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

        # topics of interest
        top_wheel_cmd_exec = "/" + self.robot_name + "/wheels_driver_node/wheels_cmd_executed"
        top_robot_pose = "/" + self.robot_name + "/apriltags2_ros/publish_detections_in_local_frame/tag_detections_local_frame"

        source = 'pickle'
        load_from_pickle = 'test_run'
        save_to_pickle = True # True/False
        set_name = 'test_run'

        #self.delta =  0.00
        if source == 'folder':
            for exp in experiments.keys():
                data_raw = DataPreparation(input_bag=experiments[exp]['path'], top_wheel_cmd_exec=top_wheel_cmd_exec, top_robot_pose=top_robot_pose)
                experiments[exp]['wheel_cmd_exec'], experiments[exp]['robot_pose']= data_raw.process_raw_data() # bring data set to format usable by the optimizer
            save_pickle(object=experiments, save_as=set_name)
        elif source == 'pickle':
            experiments = load_pickle(load_from_pickle)
        else:
            rospy.logfatal('[{}] is not a valid source type'.format(source))

        #self.fit_=self.nonlinear_model_fit()
        #self.plots()
        # write to the kinematic calibration file
        #self.write_calibration()

        # make plots & visualizations
        #self.plot=self.visualize()
        #plt.show()

    @staticmethod
    def input_folder_to_experiment_dict(folder_path):
        experiments = {}
        bag_files = os.listdir(folder_path)
        for bag in bag_files:
            bag_name = bag.split('.')[0]
            experiments[bag_name] = {'wheel_cmd_exec': None, 'robot_pose': None, 'path': join(folder_path, bag)}
        return experiments
    @staticmethod
    def kinematic_model(s, t,cmd_right,cmd_left, p):
        d = 0.06  # Distance of camera from Baseline is fixed and not part of the optimization for now
        # optimized parameters
        c, cl, tr = p

        # Simple Kinematic model
        # Assumptions: Rigid body, No lateral slip
        x_dot_rob = c * (cmd_right+cmd_left) * 0.5  + tr * (cmd_right-cmd_left)*0.5
        omega_rob = cl * (cmd_right-cmd_left) * 0.5 + tr * (cmd_right+cmd_left) * 0.5
        y_dot_rob = omega_rob * d  # The model currently also estimates the offset of the camera position

        return [x_dot_rob,  y_dot_rob, omega_rob]


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

    def cost_function(self,p, p0, cmd_right_ramp, cmd_left_ramp, s_init_ramp, x_meas_ramp, y_meas_ramp, yaw_meas_ramp, time_ramp, cmd_right_sine, cmd_left_sine, s_init_sine, x_meas_sine, y_meas_sine, yaw_meas_sine, time_sine):
        #simulate the model
        #states for a particular p set
        s_p_ramp = self.simulate(p, cmd_right_ramp, cmd_left_ramp, s_init_ramp, time_ramp)
        s_p_sine = self.simulate(p, cmd_right_sine, cmd_left_sine, s_init_sine, time_sine)

        c_init, cl_init, tr_init = p0
        c_cur, cl_cur, tr_cur = p

        delta = self.delta

        obj_cost = 0.0

        for i in range(len(time_ramp)):
            obj_cost+= ( ((s_p_ramp[i,0] - x_meas_ramp[i])) ** 2 +
                         ((s_p_ramp[i,1] - y_meas_ramp[i])) ** 2 +
                        ((s_p_ramp[i,2] - yaw_meas_ramp[i])) ** 2
                       )

        for i in range(len(time_sine)):
            obj_cost+= (((s_p_sine[i,0] - x_meas_sine[i])) ** 2 +
                        ((s_p_sine[i,1] - y_meas_sine[i])) ** 2 +
                        ((s_p_sine[i,2] - yaw_meas_sine[i])) ** 2
                       )

        obj_cost+= delta * ((c_cur - c_init) ** 2 + (cl_cur - cl_init) ** 2 + (tr_cur - tr_init) ** 2)

        return obj_cost

    def nonlinear_model_fit(self):
        start = self.DATA_BEG_INDEX

        (x_ramp_meas,y_ramp_meas, yaw_ramp_meas,
        x_sine_meas,y_sine_meas,yaw_sine_meas,
        timepoints_ramp,time_ramp ,
        cmd_ramp_right, cmd_ramp_left,
        timepoints_sine,time_sine,
        cmd_sine_right, cmd_sine_left) = self.processData(starting_ind = self.DATA_BEG_INDEX, ending_ind = self.DATA_END_INDEX)

        #initial conditions for the states
        s_init_sine = [x_sine_meas[start],y_sine_meas[start], yaw_sine_meas[start]]
        s_init_ramp = [x_ramp_meas[start], y_ramp_meas[start], yaw_ramp_meas[start]]

        #initial guesses for the optimization parameters
        p0 = self.p0

        # Actual Parameter Optimization/Fitting
        # Minimize the least squares error between the model prediction
        result = minimize(self.cost_function, p0, args=(p0, cmd_ramp_right, cmd_ramp_left, s_init_ramp, x_ramp_meas, y_ramp_meas, yaw_ramp_meas, timepoints_ramp, cmd_sine_right, cmd_sine_left, s_init_sine, x_sine_meas, y_sine_meas, yaw_sine_meas, timepoints_sine))
        popt = result.x
        print('[BEGIN] Optimization Result\n')
        print(result)
        print('[END] Optimization Result\n')

        # Make a prediction based on the fitted parameters for ramp experiment data
        y_opt_predict_ramp = self.simulate(popt, cmd_ramp_right, cmd_ramp_left, s_init_ramp , timepoints_ramp) # Predict to calculate Error
        self.y_opt_predict_ramp = y_opt_predict_ramp
        # Make a prediction based on the fitted parameters for sine experiment data
        y_opt_predict_sine = self.simulate(popt, cmd_sine_right, cmd_sine_left, s_init_sine , timepoints_sine) # Predict to calculate Error
        self.y_opt_predict_sine = y_opt_predict_sine

        self.sine_plots(p0, popt, y_opt_predict_sine, cmd_sine_right, cmd_sine_left, s_init_sine, x_sine_meas, y_sine_meas, yaw_sine_meas, time_sine, timepoints_sine)
        self.ramp_plots(p0, popt, y_opt_predict_ramp, cmd_ramp_right, cmd_ramp_left, s_init_ramp, x_ramp_meas, y_ramp_meas, yaw_ramp_meas, time_ramp, timepoints_ramp)

        popt_dict = {"gain": popt[0], "trim":popt[2]}
        return popt_dict

    def sine_plots(self,p0, popt, y_opt_predict_sine, cmd_sine_right, cmd_sine_left, s_init_sine, x_sine_meas, y_sine_meas, yaw_sine_meas, time_sine, timepoints_sine):

        Y = np.stack((x_sine_meas, y_sine_meas, yaw_sine_meas), axis=1)

        MSE_sine = np.sum((Y-y_opt_predict_sine)**2)/y_opt_predict_sine.size # Calculate the Mean Squared Error

        y_pred_sine_default_params = self.simulate(p0, cmd_sine_right, cmd_sine_left, s_init_sine , timepoints_sine)
        self.y_pred_sine_default_params = y_pred_sine_default_params

        # PLOTTING
        fig2, ax1 = plt.subplots()

        x_sine_meas_handle, = ax1.plot(time_sine[timepoints_sine],x_sine_meas,'x',color=(0.5,0.5,1), label = 'x measured')
        y_sine_meas_handle, = ax1.plot(time_sine[timepoints_sine],y_sine_meas,'x',color=(0.5,1,0.5), label = 'y measured')
        yaw_sine_meas_handle, = ax1.plot(time_sine[timepoints_sine],yaw_sine_meas,'x',color=(1,0.5,0.5), label = 'yaw measured')

        # Model predictions with default parameters
        x_sine_pred_default_handle, = ax1.plot(time_sine[timepoints_sine],y_pred_sine_default_params[:,0],'bo', label = 'x predict def')
        y_sine_pred_default_handle, = ax1.plot(time_sine[timepoints_sine],y_pred_sine_default_params[:,1],'go', label = 'y predict def')
        yaw_sine_pred_default_handle, = ax1.plot(time_sine[timepoints_sine],y_pred_sine_default_params[:,2],'ro', label = 'yaw predict def')


        # Model predictions with optimal parametes

        x_sine_pred_handle, = ax1.plot(time_sine[timepoints_sine],y_opt_predict_sine[:,0],'b', label = 'x predict opt')
        y_sine_pred_handle, = ax1.plot(time_sine[timepoints_sine],y_opt_predict_sine[:,1],'g', label = 'y predict opt')
        yaw_sine_pred_handle, = ax1.plot(time_sine[timepoints_sine],y_opt_predict_sine[:,2],'r', label = 'yaw predict opt')
        ax1.set_xlabel('time [s]')
        ax1.set_ylabel('position [m] / heading [rad]')

        handles = [x_sine_meas_handle,y_sine_meas_handle,yaw_sine_meas_handle,
                   x_sine_pred_handle,y_sine_pred_handle,yaw_sine_pred_handle,
                   x_sine_pred_default_handle, y_sine_pred_default_handle, yaw_sine_pred_default_handle]
        labels = [h.get_label() for h in handles]

        fig2.legend(handles=handles, labels=labels, prop={'size': 10}, loc=0)
        fig2.suptitle('Measurements and Prediction with Default/Optimal Parameter Values - Sine Manouver', fontsize=16)
        #plt.show(block=True)
    def ramp_plots(self,p0, popt, y_opt_predict_ramp, cmd_ramp_right, cmd_ramp_left, s_init_ramp, x_ramp_meas, y_ramp_meas, yaw_ramp_meas, time_ramp, timepoints_ramp):
        Y = np.stack((x_ramp_meas, y_ramp_meas, yaw_ramp_meas), axis=1)

        MSE_ramp = np.sum((Y-y_opt_predict_ramp)**2)/y_opt_predict_ramp.size # Calculate the Mean Squared Error

        y_pred_ramp_default_params = self.simulate(p0, cmd_ramp_right, cmd_ramp_left, s_init_ramp , timepoints_ramp)
        self.y_pred_ramp_default_params = y_pred_ramp_default_params

        # PLOTTING
        fig1, ax1 = plt.subplots()

        x_ramp_meas_handle, = ax1.plot(time_ramp[timepoints_ramp],x_ramp_meas,'x',color=(0.5,0.5,1), label = 'x measured')
        y_ramp_meas_handle, = ax1.plot(time_ramp[timepoints_ramp],y_ramp_meas,'x',color=(0.5,1,0.5), label = 'y measured')
        yaw_ramp_meas_handle, = ax1.plot(time_ramp[timepoints_ramp],yaw_ramp_meas,'x',color=(1,0.5,0.5), label = 'yaw measured')

        # Model predictions with default parameters
        x_ramp_pred_default_handle, = ax1.plot(time_ramp[timepoints_ramp],y_pred_ramp_default_params[:,0],'bo', label = 'x predict def')
        y_ramp_pred_default_handle, = ax1.plot(time_ramp[timepoints_ramp],y_pred_ramp_default_params[:,1],'go', label = 'y predict def')
        yaw_ramp_pred_default_handle, = ax1.plot(time_ramp[timepoints_ramp],y_pred_ramp_default_params[:,2],'ro', label = 'yaw predict def')

        # Model predictions with optimal parametes
        x_ramp_pred_handle, = ax1.plot(time_ramp[timepoints_ramp],y_opt_predict_ramp[:,0],'b', label = 'x predict opt')
        y_ramp_pred_handle, = ax1.plot(time_ramp[timepoints_ramp],y_opt_predict_ramp[:,1],'g', label = 'y predict opt')
        yaw_ramp_pred_handle, = ax1.plot(time_ramp[timepoints_ramp],y_opt_predict_ramp[:,2],'r', label = 'yaw predict opt')

        ax1.set_xlabel('time [s]')
        ax1.set_ylabel('position [m] / heading [rad]')

        handles = [x_ramp_meas_handle,y_ramp_meas_handle,yaw_ramp_meas_handle,
                   x_ramp_pred_handle,y_ramp_pred_handle,yaw_ramp_pred_handle,
                   x_ramp_pred_default_handle, y_ramp_pred_default_handle, yaw_ramp_pred_default_handle]

        labels = [h.get_label() for h in handles]

        fig1.legend(handles=handles, labels=labels, prop={'size': 10}, loc=0)
        fig1.suptitle('Measurements and Prediction with Default/Optimal Parameter Values - Ramp Manouver', fontsize=16)

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