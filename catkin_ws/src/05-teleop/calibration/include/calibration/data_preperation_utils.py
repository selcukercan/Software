import rospy, rosbag
import numpy as np
#import plotly.graph_objs as go
#import plotly.offline as opy
import copy
import pickle
from calibration.data_adapter_utils import *


class DataPreparation():
    DEBUG_MODE = True # convenience flag
    TEST_MODE = False # in test mode no plots are drawn
    DISCARD_FIRST = 30 # discard first n data
    DISCARD_LAST = 10  # discard last n data

    def __init__(self, input_bag = None, top_wheel_cmd_exec = None, top_robot_pose = None, save_as = None, dump = False, exp_name='', mode='train'):
        self.input_bag = input_bag
        self.wheel_cmd, self.robot_pose = self.load_bag(input_bag, top_wheel_cmd_exec, top_robot_pose)
        self.exp_name = exp_name
        self.operation_mode = mode
    def process_raw_data(self):
        """
        process the data contained in a rosbag file and bring it to the form accepted by the optimization

        returns:
            3-element tuple containing

        - **wheel_cmd_exec_opt** (*ndarray*) - 2*N ndarray, where first row is right wheel, and second row is left wheel commands
        - **robot_pose_opt** (*ndarray*) - 3*N ndarray, where first row is x coordinate, the second row is y coordinate, and the third row is yaw angle
        - **t** (*list*) - timestamps.

        """

        start_time, end_time, duration = self.experiment_duration()
        wheel_cmd_clipped, robot_pose_clipped = self.get_actuated_interval(self.wheel_cmd, self.robot_pose, start_time, end_time)
        wheel_cmd_exec_rs = self.resampling(wheel_cmd_clipped, robot_pose_clipped)

        wheel_cmd_exec_sel = self.select_interval(wheel_cmd_exec_rs, self.DISCARD_FIRST, self.DISCARD_LAST)
        robot_pose_sel = self.select_interval(robot_pose_clipped, self.DISCARD_FIRST, self.DISCARD_LAST)

        # at this point the times should be synced so select time from either of them
        # assert(wheel_cmd_exec_sel['timestamp'] == robot_pose_opt['timestamp'])
        t = wheel_cmd_exec_sel['timestamp']
        wheel_cmd_exec_opt = self.u_adapter(wheel_cmd_exec_sel)
        robot_pose_opt = self.x_adapter(robot_pose_sel)

        """
        if self.operation_mode == 'train':
            robot_pose_opt = self.filter(robot_pose_opt, [3,3,3], ["flat" , "flat", "flat"])
        """

        return wheel_cmd_exec_opt, robot_pose_opt, t

    def experiment_duration(self):
        """
        decide experiment start time, end time and duration based on the first and the last actuation commands

        Returns:
            3-element tuple containing

        - **start_time** (*float*) - first non-zero actuation comman
        - **end_time** (*float*) - last non-zero actuation command.
        - **duration** (*float*) - translation vector from robot's cf to apriltag's cf expressed in robot cf.
        """

        wheel_cmd = self.wheel_cmd

        # get index where where there is a velocity command and its respective timestamp
        actuated_i = [i for i, j in enumerate(wheel_cmd['vel_r']) if j != 0]
        actuated_t = [wheel_cmd['timestamp'][i] for i in actuated_i]

        if actuated_t == []:  # there does not exist a time instance when robot is actuated.
            rospy.logerr('DATA SET DOES NOT CONTAIN ANY ACTUATION COMMAND')

        start_time = actuated_t[0]  # first time instance when an the vehicle recieves an actuation command
        end_time = actuated_t[-1]  # last time instance when an the vehicle recieves an actuation command. notice that this is not necessarily the stopping instance as the vehicle will keep on moving due to inertia.
        duration =  end_time - start_time

        return start_time, end_time, duration

    def get_actuated_interval(self, wheel_cmd_exec, robot_pose, start_time, end_time):
        """
        clips the wheel commands and pose measurements to [start_time, end_time] closed interval.

        it uses wheel_cmd_exec as it is the actual command that is send to the vehicle.
        reference: https://github.com/duckietown/Software/blob/9cae95e41d1672f86a10bba86fca430e73af2431/catkin_ws/src/05-teleop/dagu_car/src/wheels_driver_node.py

        Args:
            wheel_cmd_exec (dict): input commands to the vehicle.
            robot_pose (dict): pose of the vehicle.
            start_time (float): experiment start [sec].
            end_time (float): experiment end [sec].

        Returns:
            2-element tuple containing

        - **wheel_cmd_exec** (*dict*) - veh_R_world, rotation matrix that expresses world_p in robot cf.
        - **robot_pose** (*dict*)- translation vector from robot's cf to apriltag's cf expressed in robot cf.

        """

        # extract the pose detections that fall into active experiment time.
        while robot_pose['timestamp'][0] < start_time:
            for key in robot_pose.keys():
                del robot_pose[key][0]
        while robot_pose['timestamp'][-1] > end_time:
            for key in robot_pose.keys():
                del robot_pose[key][-1]

        # eliminates the case when 0 input applied at the beginning of the experiment
        while wheel_cmd_exec['timestamp'][0] < start_time:
            for key in wheel_cmd_exec.keys():
                del wheel_cmd_exec[key][0]
        # eliminates the case when 0 input applied at the end of the experiment
        while wheel_cmd_exec['timestamp'][-1] > end_time:
            for key in wheel_cmd_exec.keys():
                del wheel_cmd_exec[key][-1]

        return wheel_cmd_exec, robot_pose

    def resampling(self, wheel_cmd_exec, robot_pose):
        """
        resamples wheel commands at robot_pose time instances by linearly interpolation, i.e.
        for every pose measurement instance t_pose, the function estimates the wheel commands at time t_pose.

        currently, also shifts the measurements to t = 0.

        Args:
            wheel_cmd_exec (dict): input commands to the vehicle.
            robot_pose (dict): pose of the vehicle.

        Returns:
            wheel_cmd_exec_rs (dict): resampled input commands.
        """

        cmd_right = wheel_cmd_exec['vel_r']
        cmd_left = wheel_cmd_exec['vel_l']

        cmd_timestamp = wheel_cmd_exec['timestamp']
        robot_pose_timestamp = robot_pose['timestamp']

        # shift the timestamps such that first wheel command corresponds to t = 0.
        # note the assumption that all position measurements before the wheel command is removed.
        t0_wheel = cmd_timestamp[0]
        cmd_timestamp = [t - t0_wheel for t in cmd_timestamp]
        robot_pose_timestamp = [t - t0_wheel for t in robot_pose_timestamp]

        # actual resampling.
        # note that this resampling method can handle variable data acquisition frequency. (previous methods assumed data comes at a fixed 30 hz rate.)
        cmd_right_rs = np.interp(robot_pose_timestamp, cmd_timestamp, cmd_right)
        cmd_left_rs = np.interp(robot_pose_timestamp, cmd_timestamp, cmd_left)
        """
        if self.DEBUG_MODE and not self.TEST_MODE:
            # Create a trace
            plot1 = go.Scatter(
                x=cmd_timestamp,
                y=cmd_right,
                name='right wheel commands'
            )

            plot2 = go.Scatter(
                x=robot_pose_timestamp,
                y=cmd_right_rs,
                name='resampled right wheel commands'
            )


            plot3 = go.Scatter(
                x=cmd_timestamp,
                y=cmd_left,
                name='left wheel commands'
            )

            plot4 = go.Scatter(
                x=robot_pose_timestamp,
                y=cmd_left_rs,
                name='resampled left wheel commands'
            )

            data_right = [plot1, plot2]
            opy.plot(data_right)

            data_left = [plot3, plot4]
            opy.plot(data_left)
        """
        # create a copy of the original wheel_cmd_exec dictionary
        wheel_cmd_exec_rs = copy.deepcopy(wheel_cmd_exec)
        # assign resampled wheel commands and the corresponding timestamps to the new dictionary.
        wheel_cmd_exec_rs['vel_r'] = cmd_right_rs
        wheel_cmd_exec_rs['vel_l'] = cmd_left_rs
        wheel_cmd_exec_rs['timestamp'] = robot_pose_timestamp

        return wheel_cmd_exec_rs

    def load_bag(self, input_bag, top_wheel_cmd_exec, top_robot_pose):
        """
        generates dictionaries for  by reading the content available in their respective topics.
        as a convention each function takes in a topic name, and returns the parameter dictionary.

        :return:
        """
        wheel_cmd_exec = self.get_wheels_command(input_bag, top_wheel_cmd_exec)
        robot_pose = self.get_robot_pose(input_bag, top_robot_pose)

        return wheel_cmd_exec, robot_pose

    def get_wheels_command(self, input_bag, topic_name):

        cmd = {
            'vel_r': [],
            'vel_l': [],
            'timestamp': [],
        }

        # Loop over the image files contained in rosbag
        for topic, msg, t in rosbag.Bag(input_bag).read_messages(topics=topic_name):
            cmd['vel_l'].append(msg.vel_left)
            cmd['vel_r'].append(msg.vel_right)
            cmd['timestamp'].append(t.to_sec())

        return cmd

    def get_robot_pose(self, input_bag, topic_name):

        pose = {
            'px': [],'py': [],'pz': [],
            'rx':[],'ry':[],'rz':[],
            'timestamp': []
        }

        # Loop over the image files contained in rosbag
        for topic, msg, t in rosbag.Bag(input_bag).read_messages(topics=topic_name):
            pose['px'].append(msg.posx)
            pose['py'].append(msg.posy)
            pose['pz'].append(msg.posz)
            pose['rx'].append(msg.rotx)
            pose['ry'].append(msg.roty)
            pose['rz'].append(msg.rotz)
            pose['timestamp'].append(t.to_sec())
        return pose

    @staticmethod
    def u_adapter(u_dict):
        v_l_r = row(np.array(u_dict['vel_l']))
        v_r_r = row(np.array(u_dict['vel_r']))
        return np.vstack([v_r_r, v_l_r])

    @staticmethod
    def x_adapter(x_dict):
        px = row(np.array(x_dict['px']))
        py = row(np.array(x_dict['py']))
        rz = row(np.array(x_dict['rz']))

        return np.vstack([px, py, rz])

    @staticmethod
    def select_interval(dict, discard_first, discard_last):
        for key, val in dict.items():
            dict[key] = dict[key][discard_first:-discard_last]
        return dict

    def filter(self, robot_pose_opt, flen_array, filter_type):
            from calibration.plotting_utils import multiplot

            (x_pos , y_pos, yaw_pos) = [robot_pose_opt[i,:] for i in range(3)] #unpack position measurements
            (flen_x, flen_y, flen_yaw) = [flen_array[i] for i in range(3)] #unpack filter lengths
            (ftype_x, ftype_y, ftype_yaw) = [filter_type[i] for i in range(3)] #unpack filter types

            # apply filters
            xpos_filt = smooth(x_pos, window_len=flen_x, window=ftype_x)
            ypos_filt = smooth(y_pos, window_len=flen_y, window=ftype_y)
            yaw_pos_filt = smooth(yaw_pos, window_len=flen_yaw, window=ftype_yaw)

            # construct filtered output
            robot_pose_opt_filt = np.zeros((3, xpos_filt.shape[0]))
            robot_pose_opt_filt[0,:] = xpos_filt
            robot_pose_opt_filt[1,:] = ypos_filt
            robot_pose_opt_filt[2,:] = yaw_pos_filt

            # plot original and filtered signals on the same pot
            multiplot(states_list=[robot_pose_opt, robot_pose_opt_filt],
                      experiment_name_list=['Original Signal', 'Filtered Signal'],
                      mode='single_view',
                      plot_title = self.exp_name + ' filtering')

            return robot_pose_opt_filt


def smooth(x, window_len=1, window='hanning'):
    """
    adapted from:
    smooth the data using a window with requested size.

    This method is based on the convolution of a scaled window with the signal.
    The signal is prepared by introducing reflected copies of the signal
    (with the window size) in both ends so that transient parts are minimized
    in the begining and end part of the output signal.

    input:
        x: the input signal
        window_len: the dimension of the smoothing window; should be an odd integer
        window: the type of window from 'flat', 'hanning', 'hamming', 'bartlett', 'blackman'
            flat window will produce a moving average smoothing.

    output:
        the smoothed signal

    example:

    t=linspace(-2,2,0.1)
    x=sin(t)+randn(len(t))*0.1
    y=smooth(x)

    see also:

    numpy.hanning, numpy.hamming, numpy.bartlett, numpy.blackman, numpy.convolve
    scipy.signal.lfilter

    TODO: the window parameter could be the window itself if an array instead of a string
    NOTE: length(output) != length(input), to correct this: return y[(window_len/2-1):-(window_len/2)] instead of just y.
    """

    if x.ndim != 1:
        raise ValueError, "smooth only accepts 1 dimension arrays."

    if x.size < window_len:
        raise ValueError, "Input vector needs to be bigger than window size."


    if window_len<3:
        return x


    if not window in ['flat', 'hanning', 'hamming', 'bartlett', 'blackman']:
        raise ValueError, "Window is on of 'flat', 'hanning', 'hamming', 'bartlett', 'blackman'"

    #  mirror the beginning and end of the sequence with window length -1 elements
    # if array x = [1,2,3,4,5] and window_len = 2, then s = [2, 1, 2, 3, 4, 5, 4]
    s=np.r_[x[window_len-1:0:-1], x, np.flip(x[x.size - window_len:x.size-1])]

    if window == 'flat': #moving average
        w=np.ones(window_len,'d')
    else:
        w=eval('np.'+window+'(window_len)')

    y=np.convolve(w/w.sum(),s,mode='valid')
    return y[0:x.size]
    #return y[(window_len/2-1):-(window_len/2-1)]

# UTILITY FUNCTIONS AND CLASSES

def save_pickle(object=None, save_as=None):
    if object is not {}:
        rospy.loginfo('dumping the experiment set with pickle, naming it [{}] ...'.format(save_as))
        with open(save_as, 'wb') as handle:
            pickle.dump(object, handle, protocol=pickle.HIGHEST_PROTOCOL)
    else:
        rospy.logerr('empty experiment set, please check that your bag files are not corrupted')

def load_pickle(experiment_name):
    if experiment_name is not None:
        rospy.loginfo('pickling experiment data with name [{}]'.format(experiment_name))
        with open(experiment_name, 'rb') as handle:
            experiments = pickle.load(handle)

        return experiments
    else:
        rospy.logfatal('to load data with pickle, specify the experiment name')

# To save data easily with pickle create a trivial class
class ExperimentData():
    pass

