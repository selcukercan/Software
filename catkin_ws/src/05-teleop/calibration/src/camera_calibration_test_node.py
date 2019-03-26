#!/usr/bin/env python
# system imports
import rospy

from os import remove
# package utilities import
from std_msgs.msg import String #Imports msg
from duckietown_msgs.msg import WheelsCmdStamped, Twist2DStamped, AprilTagDetectionArray, VehiclePoseEulerArray
from calibration.wheel_cmd_utils import *
from calibration.utils import get_package_root
from duckietown_utils.yaml_wrap import yaml_load_file
# Service types to be imported from rosbag_recorder package
from rosbag_recorder.srv import *

ground_truth = {
    "c3_00": {"posx": -0.329, "posy": 0.012, "rotz": 0.0}
}

class CameraCalibrationTest:
    def __init__(self):
        # namespace variables
        host_package = rospy.get_namespace()  # as defined by <group> in launch file
        self.node_name = 'camera_calibration_test_node'  # node name , as defined in launch file
        host_package_node = host_package + self.node_name
        self.veh = host_package.split('/')[1]

        # Initialize the node with rospy
        rospy.init_node('CameraCalibrationTest', anonymous=True, disable_signals=True)
        self.rate = rospy.Rate(30)  # 30 Hz

        # Parameters
        self.rosbag_dir = rospy.get_param("~output_rosbag_dir")

        # Topics to save into rosbag
        # Output End
        sub_topic_comp_image = '/' + self.veh + '/camera_node/image/compressed'
        sub_topic_cam_info = '/' + self.veh + '/camera_node/camera_info'
        sub_topic_rect = '/' + self.veh + '/camera_node/image/rect'
        sub_topic_tag_detections = '/' + self.veh + '/tag_detections'
        sub_topic_tag_detections_image = '/' + self.veh + '/tag_detections_image/compressed'
        sub_topic_local_pose = "/" + self.veh + '/apriltags2_ros/publish_detections_in_local_frame/tag_detections_array_local_frame'

        # Subscriber(s)
        self.sub_at_local = rospy.Subscriber(sub_topic_local_pose, VehiclePoseEulerArray, self.cb_at_local_pose, queue_size=1)

        self.topics_to_follow = [sub_topic_comp_image, sub_topic_cam_info, sub_topic_rect,
        sub_topic_tag_detections, sub_topic_tag_detections_image, sub_topic_local_pose]

        # Wait for service server - rosbag-recorder services to start
        rospy.loginfo('[Data Collector Node] BEFORE SERVICE REGISTER')
        rospy.wait_for_service('/record_topics')
        rospy.wait_for_service('/stop_recording')

        # rospy.ServiceProxy(SERVICE_NAME, SERVICE_TYPE), SERVICE_TYPE is defined under /srv of rosbag_recorder
        # call these fns with the right input/output arguments similar to local fns.
        self.topic_recorder = rospy.ServiceProxy('/record_topics', RecordTopics)
        self.recording_stop= rospy.ServiceProxy('/stop_recording', StopRecording)
        rospy.loginfo('[Data Collector Node] AFTER SERVICE REGISTER')

        self.wait_start_rosbag = 1 # wait for wait_for_rosbag seconds to make sure that the bag has started recording
        self.wait_write_rosbag = 0

        self.started_recording = False
        #self.stoped_recording = True
        self.recieved_at_position_estimates = []
        self.number_of_images_to_process = 10
        self.at_local_i = 0
        self.continue_experiment = True

    def cb_at_local_pose(self, msg):
        if self.started_recording:
            self.at_local_i += 1
            # record positions until reciving enough messages
            if self.at_local_i <= self.number_of_images_to_process:
                self.recieved_at_position_estimates.append(msg)

                rospy.loginfo("[{}] recieved at local pose {}".format(self.node_name, self.at_local_i))
            else:
                self.continue_experiment = False
        else:
            pass
            #rospy.loginfo("[{}] neglecting the pose info before recording had started".format(self.node_name))

    def get_valid_experiment(self):
        experiment_name = None
        while experiment_name == None:
            experiment_name = raw_input()
            if experiment_name not in self.available_experiments:
                print('[{}] is not a valid experiment name, please use one of the following: {}'.format(experiment_name, self.available_experiments))
                experiment_name = None
        return experiment_name

    def perform_experiments(self):
        DO_EXPERIMENT = "yes"
        self.available_experiments = ["c3_00"]

        ui = ExperimentUI()

        while DO_EXPERIMENT == "yes":
            # welcoming
            print("\nType in the name of the camera verification experiment to conduct: {}".format(str(self.available_experiments)))
            experiment_name = self.get_valid_experiment()
            experiment_conf = ground_truth[experiment_name]

            # rosbag
            rosbag_name = self.generate_experiment_label(experiment_name)
            rosbag_path = self.rosbag_dir + "/" + rosbag_name + ".bag"

            # start recording
            ready_to_start = self.start_recording(rosbag_name)
            if ready_to_start: # if the recording had started
                self.started_recording = True
                rospy.loginfo('Successfully started recording the topics!')
            else:
                rospy.logfatal('Failed to start recording the topics, needs handling!')

            # experiment will be executed until enough measurements are received
            while self.continue_experiment:rospy.sleep(1)

            # finaly stop the recording
            self.stop_recording(rosbag_name)

            # ask whether user wants to keep the current measurements
            rospy.loginfo('do you want to keep the current experiments bag file? (respond with yes or no)')
            user_input = raw_input()
            if user_input.strip().lower() == 'no':
                rospy.loginfo('removing the bag file {} ...'.format(rosbag_path))
                remove(rosbag_path)
            else:
                rospy.loginfo('keeping the bag file {}'.format(rosbag_path))

            self.evaluate_measurements(experiment_name)
            rospy.loginfo("\n\nDo you want to run another camera verification experiment? (respond with yes or no)\n\n")
            user_wish = raw_input()

            DO_EXPERIMENT = user_wish.strip().lower()

        else:
            rospy.loginfo("farewell, no more experiments for now.")

    def evaluate_measurements(self, experiment_name):
        verdict = {
        "posx": None,
        "posy": None,
        "rotz": None
        }

        trans_prec, rot_prec, at_id = self.load_test_config()
        avg_posx, avg_posy, avg_rotz = self.calculate_averages(at_id)

        posx_res = self.passes(avg_posx, ground_truth[experiment_name]["posx"], trans_prec)
        posy_res = self.passes(avg_posy, ground_truth[experiment_name]["posy"], trans_prec)
        rotz_res = self.passes(avg_rotz, ground_truth[experiment_name]["rotz"], rot_prec)

        rospy.loginfo("posx: {} posy: {} rotz: {}".format(posx_res, posy_res, rotz_res))
        rospy.loginfo("avg_posx: {} avg_posy: {} avg_rotz: {}".format(avg_posx, avg_posy, avg_rotz))

    @staticmethod
    def passes(x_meas, x_ground_truth, x_threshold):
        if abs(x_meas - x_ground_truth) < x_threshold:
            return True
        return False

    def calculate_averages(self, at_id):
        posx = 0
        posy = 0
        rotz = 0
        numb_meas = len(self.recieved_at_position_estimates)

        for i, at_array in enumerate(self.recieved_at_position_estimates):
            # allocate an AprilTag 65 for the camera_calibration test
            # this avoids possible breaking in case multiple apriltags are in the field of view.
            at = [at_cand for at_cand in at_array.local_pose_list if at_cand.id == at_id]
            posx += at[0].posx
            posy += at[0].posy
            rotz += at[0].rotz
        avg_posx = posx / numb_meas
        avg_posy = posy / numb_meas
        avg_rotz = rotz / numb_meas

        return avg_posx, avg_posy, avg_rotz

    def load_test_config(self):
        from os.path import join
        package_root = get_package_root("calibration")
        meta_conf = yaml_load_file(join(package_root,"meta_config.yaml"))
        cam_cal_test_ver = meta_conf["camera_calibration_test_config_version"]
        cam_cal_test_conf = yaml_load_file(join(package_root,"configs", "camera_calibration", "camera_calibration_test_" + str(cam_cal_test_ver) + ".yaml"))

        translational_precision = cam_cal_test_conf["translational_precision"]
        rotational_precision = cam_cal_test_conf["rotational_precision"]
        apriltags_id = cam_cal_test_conf["use_apriltag_id"]

        return translational_precision, rotational_precision, apriltags_id

    def generate_experiment_label(self, exp_name):
        import datetime
        now = datetime.datetime.now().strftime("%Y_%m_%d_%H_%M")
        experiment_name_base = exp_name + "_"+ now
        return experiment_name_base


    def start_recording(self, rosbag_name):
        # start recording rosbag
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
        self.stoped_recording = False

if __name__ == '__main__':
    camera_calibration_test = CameraCalibrationTest()
    camera_calibration_test.perform_experiments()
