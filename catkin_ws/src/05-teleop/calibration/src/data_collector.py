#!/usr/bin/env python
import rospy
from rosbag_recorder.srv import *
import rosnode
from std_msgs.msg import String #Imports msg
from duckietown_msgs.msg import WheelsCmdStamped, Twist2DStamped, AprilTagDetectionArray, VehiclePoseEuler
from calibration.wheel_cmd_utils import *
from os.path import expanduser
from os import remove

# Service types to be imported from rosbag_recorder package
from rosbag_recorder.srv import RecordTopics, StopRecording

class DataCollector:
    def __init__(self):
        host_package = rospy.get_namespace()  # as defined by <group> in launch file
        node_name = 'data_collector'  # node name , as defined in launch file
        host_package_node = host_package + node_name
        veh = host_package.split('/')[1]

        # Initialize the node with rospy
        rospy.init_node('Command', anonymous=True)
        self.frequency = 30
        # Publisher
        publisher=rospy.get_param("~veh")+"/wheels_driver_node/wheels_cmd"
        self.pub_wheels_cmd = rospy.Publisher(publisher,WheelsCmdStamped,queue_size=1)

        # Subscribers
        # Local Pose Info
        sub_topic_pose_in_world_frame =  '/' + veh + '/apriltags2_ros/publish_detections_in_local_frame/tag_detections_local_frame'
        print sub_topic_pose_in_world_frame
        #self.sub_pose_in_world_frame = rospy.Subscriber(sub_topic_pose_in_world_frame, VehiclePoseEuler, self.cbLocalPoseInfo)
        self.topics_to_follow = [sub_topic_pose_in_world_frame]
        """
        # Rectified Image
        sub_topic_rect_image = '/' + veh + '/camera_node/image/rect'
        self.sub_pose_in_world_frame = rospy.Subscriber(sub_topic_pose_in_world_frame, VehiclePoseEuler, self.cbLocalPoseInfo)
        sensor_msgs / Image
        """
        rospy.loginfo('BEFORE SERVICE REGISTER')
        # Wait for service server - rosbag-recorder services to start
        rospy.wait_for_service('/record_topics')
        rospy.wait_for_service('/stop_recording')
        # rospy.ServiceProxy(SERVICE_NAME, SERVICE_TYPE), SERVICE_TYPE is defined under /srv of rosbag_recorder
        # call these fns with the right input/output arguments similar to local fns.
        self.topic_recorder = rospy.ServiceProxy('/record_topics', RecordTopics)
        self.recording_stop= rospy.ServiceProxy('/stop_recording', StopRecording)
        rospy.loginfo('AFTER SERVICE REGISTER')

        self.wait_for_rosbag = 5


    def cbLocalPoseInfo(self):
        print 'IN HERE'

    def perform_experiments(self):
        DO_EXPERIMENT = "yes"
        rosbag_name = "deneme_delete"
        available_experiments = ["ramp_up", "sine"]
        ui = ExperimentUI()

        while DO_EXPERIMENT == "yes":
            print "Type the experiment type you want to do: {}".format(str(available_experiments))
            experiment_type = raw_input()

            if experiment_type == "ramp_up":
                ramp_up = RampUp()
                default_param_dict = ramp_up.parameter_dict
                param_dict_request = ui.request_param_values(experiment_type, default_param_dict) # get user input to decide the set of params to be used.
                print "[Calibration INFO] parameter set to be used in {} calibration: {}".format(experiment_type, str(param_dict_request))
                wheel_cmds = ramp_up.generate_input(param_dict_request)

                # start recording rosbag
                record_topic_response = self.topic_recorder(rosbag_name, self.topics_to_follow)
                record_topic_response_val = record_topic_response.success
                rospy.loginfo("XXXX {} , {} XXXX".format(str(record_topic_response.success), type(record_topic_response.success)))

                rospy.loginfo("[Publish Control] starting the node and waiting {} seconds to ensure rosbag is recording".format(str(self.wait_for_rosbag)))
                rospy.sleep(self.wait_for_rosbag)  # wait for the bag to start recording

                if record_topic_response_val == True: # if the recording had started
                    self.send_commands(wheel_cmds)
                else:
                    rospy.loginfo('Failed to start recording the topics, needs handling')

                # stop recording rosbag.
                recording_stop_response = self.recording_stop(rosbag_name)

                #ask whether user wants to keep the current measurements
                print 'do you want to keep the current experiment .bag? [respond with yes or no]'
                user_input = raw_input()
                bag_file_path = expanduser("~") + "/" + rosbag_name
                if user_input.strip().lower() == 'no':
                    print 'removing the bag file {}'.format(bag_file_path)
                    remove(bag_file_path)
                else:
                    print 'keeping the bag file {}'.format(bag_file_path)

            elif experiment_type == "sine":
                sine = Sine()
                default_param_dict = sine.parameter_dict
                param_dict_request = ui.request_param_values(experiment_type, default_param_dict)
                print "[Calibration INFO] parameter set to be used in {} calibration: {}".format(experiment_type, str(param_dict_request))
                wheel_cmds = sine.generate_input(param_dict_request)
                self.send_commands(wheel_cmds)
            else:
                print "NOT A VALID EXPERIMENT OPTION"

            print "\n\nDo you want to do another experiment? [respond with yes or no]\n\n"
            user_wish = raw_input()

            DO_EXPERIMENT = user_wish.strip().lower()
        else:
            print "farewell, no more experiments for now."

    def send_commands(self, wheel_cmds_dict):
        vel_right_array = wheel_cmds_dict['right_wheel']
        vel_left_array = wheel_cmds_dict['left_wheel']

        # Put the wheel commands in a message and publish
        msg_wheels_cmd = WheelsCmdStamped()

        for i in range(len(vel_right_array)):
            msg_wheels_cmd.header.stamp = rospy.Time.now()
            msg_wheels_cmd.vel_right = vel_right_array[i]
            msg_wheels_cmd.vel_left = vel_left_array[i]

            rospy.loginfo("Left Wheel: {} \t Right Wheel: {}".format(vel_left_array[i], vel_right_array[i]))
            self.pub_wheels_cmd.publish(msg_wheels_cmd)
            rospy.sleep(1/self.frequency)

if __name__ == '__main__':
    calib = DataCollector()
    calib.perform_experiments()

    #os.system("rosnode kill /record")
