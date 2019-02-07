#!/usr/bin/env python
# system imports
import rospy
from os.path import expanduser
from os import remove

# package utilities import
from std_msgs.msg import String #Imports msg
from duckietown_msgs.msg import WheelsCmdStamped, Twist2DStamped, AprilTagDetectionArray, VehiclePoseEuler
from calibration.wheel_cmd_utils import *
# Service types to be imported from rosbag_recorder package
from rosbag_recorder.srv import *

class DataCollector:
    def __init__(self):
        # namespace variables
        host_package = rospy.get_namespace()  # as defined by <group> in launch file
        node_name = 'data_collector'  # node name , as defined in launch file
        host_package_node = host_package + node_name
        veh = host_package.split('/')[1]

        # Initialize the node with rospy
        rospy.init_node('Command', anonymous=True)
        self.frequency = 30

        # Publisher
        publisher= '/' + veh + "/wheels_driver_node/wheels_cmd"
        self.pub_wheels_cmd = rospy.Publisher(publisher, WheelsCmdStamped, queue_size=1)

        # Topics to save into rosbag
        # Output End
        sub_topic_pose_in_world_frame =  '/' + veh + '/apriltags2_ros/publish_detections_in_local_frame/tag_detections_local_frame'
        sub_topic_rect_image = '/' + veh + '/camera_node/image/rect' # 4-5 Hz
        sub_topic_comp_image = '/' + veh + '/camera_node/image/compressed' # 30 Hz
        sub_topic_cam_info = '/' + veh +'/camera_node/camera_info'
        # Input End
        sub_topic_wheels_cmd_executed = '/' + veh + '/wheels_driver_node/wheels_cmd_executed'
        sub_topic_wheels_cmd = '/' + veh + '/wheels_driver_node/wheels_cmd'

        self.topics_to_follow = [sub_topic_pose_in_world_frame, sub_topic_rect_image, sub_topic_comp_image, sub_topic_cam_info, sub_topic_wheels_cmd_executed, sub_topic_wheels_cmd]

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

    def exp_name_to_exp_object(self, exp_name):
        """
        accepts a valid experiment name and create the corresponding experiment object

        :param exp_name:
        :return: exp_object if valid experiment name, None otherwise.
        """
        if exp_name == "ramp_up":
            return RampUp()
        elif exp_name == "sine":
            return Sine()
        elif exp_name == "sweep_sine":
            return SweepSine()
        elif exp_name == "step_salsa":
            return StepSalsa()
        elif exp_name == "step":
            return Step()
        else:
            print('[{}] is not a valid experiment name, please enter a valid one '.format(exp_name))
            return None

    def get_valid_experiment(self):
        experiment_object = None
        while experiment_object == None:
            experiment_type = raw_input()
            experiment_object = self.exp_name_to_exp_object(experiment_type)
        return experiment_type, experiment_object

    def perform_experiments(self):

        DO_EXPERIMENT = "yes"
        available_experiments = ["ramp_up", "sine", "sweep_sine", "step_salsa", "step"]
        ui = ExperimentUI()

        while DO_EXPERIMENT == "yes":
            print("\nType in the name of the experiment to conduct: {}".format(str(available_experiments)))
            experiment_type, experiment_object = self.get_valid_experiment()

            if experiment_type != None:
                #ramp_up = RampUp()
                default_param_dict = experiment_object.parameter_dict
                param_dict_request = ui.request_param_values(experiment_type, default_param_dict) # get user input to decide the set of params to be used.
                rospy.loginfo("[Data Collector Node] parameter set to be used in {} calibration: {}".format(experiment_type, str(param_dict_request)))

                wheel_cmds = experiment_object.generate_input(param_dict_request)
                rosbag_name = experiment_object.generate_experiment_label()


                # start recording rosbag
                record_topic_response = self.topic_recorder(rosbag_name, self.topics_to_follow)
                record_topic_response_val = record_topic_response.success

                rospy.loginfo("[Data Collector Node] starting the node and waiting {} seconds to ensure rosbag is recording ...".format(str(self.wait_start_rosbag)))
                rospy.sleep(self.wait_start_rosbag)  # wait for the bag to start recording


                if record_topic_response_val == True: # if the recording had started
                    self.send_commands(wheel_cmds)
                else:
                    rospy.loginfo('Failed to start recording the topics, needs handling!')

                # stop recording rosbag.
                rospy.loginfo("[Data Collector Node] waiting {} seconds to ensure all data is recorded into rosbag ...".format(str(self.wait_write_rosbag)))
                rospy.sleep(self.wait_write_rosbag)  # wait for the bag to record all data
                recording_stop_response = self.recording_stop(rosbag_name)

                # ask whether user wants to keep the current measurements
                rospy.loginfo('do you want to keep the current experiments bag file? (respond with yes or no)')
                user_input = raw_input()
                bag_file_path = expanduser("~") + "/" + rosbag_name + ".bag"
                if user_input.strip().lower() == 'no':
                    rospy.loginfo('removing the bag file {} ...'.format(bag_file_path))
                    remove(bag_file_path)
                else:
                    rospy.loginfo('keeping the bag file {}.'.format(bag_file_path))

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

        for i in range(len(vel_right_array)):
            msg_wheels_cmd.header.stamp = rospy.Time.now()
            msg_wheels_cmd.vel_right = vel_right_array[i]
            msg_wheels_cmd.vel_left = vel_left_array[i]

            #rospy.loginfo("Left Wheel: {} \t Right Wheel: {}".format(vel_left_array[i], vel_right_array[i]))
            self.pub_wheels_cmd.publish(msg_wheels_cmd)
            rospy.sleep(1/float(self.frequency))

if __name__ == '__main__':
    calib = DataCollector()
    calib.perform_experiments()
