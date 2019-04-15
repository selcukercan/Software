#!/usr/bin/env python
# system imports
import rospy
from os.path import expanduser
from os import remove, mknod

# package utilities import
from std_msgs.msg import String #Imports msg
from duckietown_msgs.msg import WheelsCmdStamped, Twist2DStamped, AprilTagDetectionArray, VehiclePoseEuler
from calibration.wheel_cmd_utils import *
from calibration.utils import is_valid_param
# Service types to be imported from rosbag_recorder package
from rosbag_recorder.srv import *

class DataCollector:
    def __init__(self):
        # namespace variables
        host_package = rospy.get_namespace()  # as defined by <group> in launch file
        self.node_name = 'data_collector'  # node name , as defined in launch file
        host_package_node = host_package + self.node_name
        self.veh = host_package.split('/')[1]

        # Initialize the node with rospy
        rospy.init_node('Command', anonymous=True, disable_signals=True)
        self.rate = rospy.Rate(30)  # 30 Hz

        # Parameters
        self.rosbag_dir = rospy.get_param("~output_rosbag_dir")
        self.use_for = is_valid_param(param_name='use_for',
                                      param_address='~use_for',
                                      valid_params=['calibration', 'verification'])

        # Publisher
        if self.use_for == 'calibration':
            topic_wheel_cmd = '/' + self.veh + "/wheels_driver_node/wheels_cmd"
            self.pub_wheels_cmd = rospy.Publisher(topic_wheel_cmd, WheelsCmdStamped, queue_size=1)
        elif self.use_for == 'verification':
            topic_car_cmd = '/' + self.veh + "/joy_mapper_node/car_cmd"
            self.pub_car_cmd = rospy.Publisher(topic_car_cmd, Twist2DStamped, queue_size=1)
        # Topics to save into rosbag
        # Output End
        sub_topic_comp_image = '/' + self.veh + '/camera_node/image/compressed'  # 30 Hz
        sub_topic_cam_info = '/' + self.veh + '/camera_node/camera_info'
        # Input End
        sub_topic_wheels_cmd_executed = '/' + self.veh + '/wheels_driver_node/wheels_cmd_executed'
        sub_topic_wheels_cmd = '/' + self.veh + '/wheels_driver_node/wheels_cmd'
        sub_topic_car_cmd = '/' + self.veh + "/joy_mapper_node/car_cmd"

        rospy.logwarn("[{}] perform data collection for {}".format(self.node_name, self.use_for))

        self.cam_topics = [sub_topic_comp_image, sub_topic_cam_info]
        self.wheel_cmd_topics = [sub_topic_wheels_cmd_executed, sub_topic_wheels_cmd]
        self.topics_to_follow = self.cam_topics + self.wheel_cmd_topics + [sub_topic_car_cmd]

        # Wait for service server - rosbag-recorder services to start
        rospy.loginfo('[Data Collector Node] BEFORE SERVICE REGISTER')
        rospy.wait_for_service('/record_topics')
        rospy.wait_for_service('/stop_recording')

        # rospy.ServiceProxy(SERVICE_NAME, SERVICE_TYPE), SERVICE_TYPE is defined under /srv of rosbag_recorder
        # call these fns with the right input/output arguments similar to local fns.
        self.topic_recorder = rospy.ServiceProxy('/record_topics', RecordTopics)
        self.recording_stop= rospy.ServiceProxy('/stop_recording', StopRecording)
        rospy.loginfo('[Data Collector Node] AFTER SERVICE REGISTER')

        self.wait_start_rosbag = 5 # wait for wait_for_rosbag seconds to make sure that the bag has started recording
        self.wait_write_rosbag = 0
        self.wait_for_parameter_server = 3

        self.map = self.exp_name_object_map()

    @staticmethod
    def exp_name_object_map():
        """ maps the input names to input classes. note that return type is an uninitialized object"""
        return {
            'ramp_up': RampUp,
            'sine': Sine,
            'sweep_sine': SweepSine,
            'step_salsa': StepSalsa,
            'step': Step,
            'circle': Circle,
            'infinity': Infinity
        }


    def exp_name_to_exp_object(self, exp_name, use_for=None):
        """
        accepts a valid experiment name and create the corresponding experiment object

        :param exp_name:
        :return: exp_object if valid experiment name, None otherwise.
        """
        if use_for == 'calibration':
            if exp_name in self.available_experiments:
                return self.map[exp_name]
            else:
                print('[{}] is not a valid experiment name for calibration, please enter a valid one '.format(exp_name))
                return None
        elif use_for == 'verification':
            if exp_name in self.available_experiments:
                return self.map[exp_name]
            else:
                print('[{}] is not a valid experiment name for verification, please enter a valid one '.format(exp_name))
                return None


    def get_valid_experiment(self, use_for=None):
        experiment_object = None
        while experiment_object == None:
            experiment_type = raw_input()
            experiment_object = self.exp_name_to_exp_object(experiment_type, use_for=use_for)
        return experiment_type, experiment_object

    def perform_experiments(self):
        DO_EXPERIMENT = "yes"

        if self.use_for == 'calibration':
            self.available_experiments = ["ramp_up", "sine", "sweep_sine", "step_salsa", "step"]
        elif self.use_for == 'verification':
            self.available_experiments = ["step", "ramp_up", "sine", "circle", "infinity"]
        else:
            pass

        ui = ExperimentUI()
        rospy.loginfo("waiting for all parameters to be set ... ")
        rospy.sleep(self.wait_for_parameter_server)

        while DO_EXPERIMENT == "yes":
            print("\nType in the name of the experiment to conduct: {}".format(str(self.available_experiments)))
            experiment_type, experiment_object_init = self.get_valid_experiment(use_for=self.use_for)
            experiment_object = experiment_object_init(mode=self.use_for)

            if experiment_type != None:
                default_param_dict = experiment_object.parameter_dict
                param_dict_request = ui.request_param_values(experiment_type, default_param_dict) # get user input to decide the set of params to be used.
                rospy.loginfo("[Data Collector Node] parameter set to be used in {} calibration: {}".format(experiment_type, str(param_dict_request)))

                if self.use_for == 'calibration':
                    wheel_cmds = experiment_object.generate_input(param_dict_request)
                elif self.use_for == 'verification':
                    reference_traj = experiment_object.generate_trajectory(param_dict_request)

                # rosbag
                rosbag_name = experiment_object.generate_experiment_label()
                rosbag_path = self.rosbag_dir + "/" + rosbag_name + ".bag"
                # start recording
                ready_to_start = self.start_recording(rosbag_name)

                if ready_to_start: # if the recording had started
                    if self.use_for == 'calibration':
                        self.send_commands(wheel_cmds)
                    elif self.use_for == 'verification':
                        self.send_reference_trajectory(reference_traj)
                else:
                    rospy.loginfo('Failed to start recording the topics, needs handling!')

                self.stop_recording(rosbag_name)

                # ask whether user wants to keep the current measurements
                rospy.loginfo('do you want to keep the current experiments bag file? (respond with yes or no)')

                user_input = raw_input()
                if user_input.strip().lower() == 'no':
                    rospy.loginfo('removing the bag file {} ...'.format(rosbag_path))
                    remove(rosbag_path)
                else:
                    rospy.loginfo('keeping the bag file {}.'.format(rosbag_path))

            rospy.loginfo("\n\nDo you want to do another experiment? (respond with yes or no)\n\n")
            user_wish = raw_input()

            DO_EXPERIMENT = user_wish.strip().lower()
        else:
            rospy.loginfo("farewell, no more experiments for now.")

    def send_commands(self, wheel_cmds_dict):
        vel_right_array = wheel_cmds_dict['right_wheel']
        vel_left_array = wheel_cmds_dict['left_wheel']

        # Put the wheel commands in a message and publish
        msg_wheels_cmd = WheelsCmdStamped()

        rospy.loginfo("starting to send commands to motors ... \n")
        for i in range(len(vel_right_array)):
            msg_wheels_cmd.header.stamp = rospy.Time.now()
            msg_wheels_cmd.vel_right = vel_right_array[i]
            msg_wheels_cmd.vel_left = vel_left_array[i]

            rospy.loginfo("sending Left Wheel: {} \t Right Wheel: {}".format(vel_left_array[i], vel_right_array[i]))
            self.pub_wheels_cmd.publish(msg_wheels_cmd)
            self.rate.sleep()
        rospy.loginfo("completed sending commands to motors \n")

    def send_reference_trajectory(self, ref_traj):
        v = ref_traj['v']
        w = ref_traj['w']

        # Put the wheel commands in a message and publish
        msg_car_cmd = Twist2DStamped()

        rospy.loginfo("starting to send reference velocities ... \n")
        for i in range(len(w)):
            msg_car_cmd.header.stamp = rospy.Time.now()
            msg_car_cmd.v = v[i]
            msg_car_cmd.omega = w[i]

            rospy.loginfo("sending v: {} \t w: {}".format(v[i], w[i]))

            self.pub_car_cmd.publish(msg_car_cmd)
            self.rate.sleep()
        rospy.loginfo("completed sending reference velocities\n")

    def topics_ready(self, topics_to_follow):
        pub_topics_and_message_type = rospy.get_published_topics('/' + self.veh)
        pub_topics = [i[0] for i in pub_topics_and_message_type]

        for top in topics_to_follow:
            if top not in pub_topics:
                return False
        return True

    def start_recording(self, rosbag_name):
        # start recording rosbag
        cam_topics_ready = self.topics_ready(self.cam_topics)
        if not cam_topics_ready:
            rospy.logfatal('[{}] {} are not published, did you roslaunch the camera-related functionality?'
                           .format(self.node_name, str(self.cam_topics)))
            rospy.signal_shutdown('missing a required topic(s)')

        record_topic_response = self.topic_recorder(rosbag_name, self.topics_to_follow)
        record_topic_response_val = record_topic_response.success

        rospy.loginfo(
            "[Data Collector Node] starting the node and waiting {} seconds to ensure rosbag is recording ...".format(
                str(self.wait_start_rosbag)))
        rospy.sleep(self.wait_start_rosbag)  # wait for the bag to start recording
        return record_topic_response_val

    def stop_recording(self, rosbag_name):
        # stop recording rosbag.
        rospy.loginfo("[Data Collector Node] waiting {} seconds to ensure all data is recorded into rosbag ...".format(
            str(self.wait_write_rosbag)))

        rospy.sleep(self.wait_write_rosbag)  # wait for the bag to record all data
        recording_stop_response = self.recording_stop(rosbag_name)
        rospy.sleep(self.wait_write_rosbag)  # wait for the bag to record all data

        print"recording_stop_response: {} ".format(recording_stop_response)

if __name__ == '__main__':
    calib = DataCollector()
    calib.perform_experiments()
