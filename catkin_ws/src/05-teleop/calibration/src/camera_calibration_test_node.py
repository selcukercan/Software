#!/usr/bin/env python
# system imports
import rospy
import os
from shutil import move
# package utilities import
from std_msgs.msg import String #Imports msg
from duckietown_msgs.msg import WheelsCmdStamped, Twist2DStamped, AprilTagDetectionArray, VehiclePoseEulerArray
from calibration.wheel_cmd_utils import *
from calibration.utils import get_package_root, get_hostname, get_cpu_info, create_time_label, get_software_version, safe_create_dir,\
    copy_calibrations_folder, copy_folder, pack_results, rad
from duckietown_utils.yaml_wrap import yaml_load_file, yaml_write_to_file
# Service types to be imported from rosbag_recorder package
from rosbag_recorder.srv import *

"""
TODO: 

* The new apriltag pp will return in radians. Be sure to adjust the code,
* For each new apriltag add them to ground-truth and map. (ground truth will come from Rafael.

"""
ground_truth = {
    "d1_p40": {"posx": 0.40, "posy": 0.10, "rotz": rad(40.0)},
    "d3_00": {"posx": 0.40, "posy": 0.00, "rotz": 0.0},
    "d5_n40": {"posx": 0.40, "posy": -0.10, "rotz": rad(40.0)}
}

map_default = {
    83: "d5_n40",
    84: "d3_00",
    85: "d1_p40"
}

experiment_map_dict = {
    "default": {
        "map": map_default
    }
}
class CameraCalibrationTest:
    def __init__(self):
        # namespace variables
        host_package = rospy.get_namespace()  # as defined by <group> in launch file
        self.package_name = "calibration"
        self.node_name = 'camera_calibration_test_node'  # node name , as defined in launch file
        host_package_node = host_package + self.node_name
        #self.veh = host_package.split('/')[1]
        self.veh = "mete"

        # Initialize the node with rospy
        rospy.init_node('CameraCalibrationTest', anonymous=True, disable_signals=True)
        self.rate = rospy.Rate(30)  # 30 Hz
        self.time_label = create_time_label()

        # Parameters
        param_results_dir = "/" + self.veh + "/calibration/camera_calibration_test_node/output_dir"
        self.output_dir = rospy.get_param(param_results_dir)
        self.results_dir = os.path.join(self.output_dir, self.time_label)
        self.data_dir = os.path.join(self.results_dir, "data")
        # create necessary folders to store the results
        safe_create_dir(self.results_dir)
        safe_create_dir(self.data_dir)

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
        self.conf = self.load_test_config()

        self.recieved_at_position_estimates = []
        self.allowed_at_id = self.conf["at_list"]
        self.number_of_images_to_process = self.conf["number_of_images_to_process"]

    def advertise(self):
        msg = """
        Welcome to Camera Calibration Verification Test!
        This script lets you determine if your duckiebots`s camera is properly calibrated.
        
        At this point make sure that you properly placed your duckiebot into the verification hardware.
        
        For the purposes of AIDO Suitability Test type in ** default ** when you are prompted to select 
        an experiment.
        
        The ** default ** experiment takes {} images and calculates the mean and the standard
        deviation of the error - the difference between the pose estimation by apriltag algorithm and 
        the ground truth. Then we compare these values against some predefined threshold values.
        
        The experiment is considered successfull iff the error stats remains inside these defined boundaries:
        rotational_error_tolerance: {} [degrees]
        translational_error_tolerance: {} [meters]
        rotational_std_tolerance: {} [degrees]
        translational_std_tolerance: {} [meters]
    
        The parameters describing the test can be found inside the camera_calibration_test_1.0.yaml located at
        rospack(calibration)/configs/camera_calibration/
        """.format(
            self.conf["number_of_images_to_process"],
            self.conf["rotational_error_tolerance"],
            self.conf["translational_error_tolerance"],
            self.conf["rotational_std_tolerance"],
            self.conf["translational_std_tolerance"]
        )

        print(msg)

    def cb_at_local_pose(self, msg):
        if self.started_recording:
            self.at_local_i += 1
            if self.at_local_i <= self.number_of_images_to_process: # record positions until receiving enough messages
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
        self.available_experiments = experiment_map_dict.keys()

        ui = ExperimentUI()
        self.advertise()

        while DO_EXPERIMENT == "yes":
            # welcoming
            print("\nType in the name of the camera verification experiment to conduct: {}".format(str(self.available_experiments)))
            self.experiment_name = self.get_valid_experiment()

            # rosbag-related operations
            rosbag_name = self.generate_experiment_label(self.experiment_name)
            rosbag_path = self.output_dir + "/" + rosbag_name + ".bag"

            # start recording
            ready_to_start = self.start_recording(rosbag_name)
            if ready_to_start: # if the recording had started
                self.started_recording = True
                rospy.loginfo('Successfully started recording the topics!')
            else:
                rospy.logfatal('Failed to start recording the topics, needs handling!')

            # experiment will be executed until **enough** measurements are received
            self.at_local_i = 0
            self.continue_experiment = True
            while self.continue_experiment: rospy.sleep(1)

            # finally stop the recording
            self.stop_recording(rosbag_name)

            # ask whether user wants to keep the current measurements
            rospy.loginfo('do you want to keep the current experiments bag file? (respond with yes or no)')
            user_input = raw_input()
            if user_input.strip().lower() == 'no':
                rospy.loginfo('removing the bag file {} ...'.format(rosbag_path))
                os.remove(rosbag_path)
            else:
                rospy.loginfo('keeping the bag file {}'.format(rosbag_path))
                move(rosbag_path, self.data_dir) # move the bag to data folder in which we store experiment data

            self.measurements = self.process_at_array()
            at_id_ground_truth_dict = self.get_at_locations(self.experiment_name)
            self.evaluate_measurements(self.measurements, at_id_ground_truth_dict)
            rospy.loginfo("\n\nDo you want to run another camera verification experiment? (respond with yes or no)\n\n")
            user_wish = raw_input()

            DO_EXPERIMENT = user_wish.strip().lower()
        else:
            rospy.loginfo("farewell, no more experiments for now.")
            rospy.logwarn("[{}] packing up, find the results at: {}".format(self.node_name, self.results_dir))
            self.prepare_to_leave()

    def get_at_locations(self, experiment_name):
        """
        return a dict where
            key: apriltag_id
            value: dict specifing metric tag location on the map
        """
        map_for_experiment = experiment_map_dict[experiment_name]["map"]
        at_locations = {}
        for at_id in map_for_experiment.keys():
            at_locations[at_id] = ground_truth[map_for_experiment[at_id]]
        return at_locations

    def evaluate_measurements(self, measurements, at_id_ground_truth_dict):
        """ for each apriltag decide if the measurements conform to the design requirements  """
        for at_i in measurements.keys():
            at_verdict = {}
            # calculate the mean and the standard-deviation of the error
            med_posx, std_posx = measurements[at_i].stat("px")
            med_posy, std_posy = measurements[at_i].stat("py")
            med_rotz, std_rotz = measurements[at_i].stat("rz")

            at_i_posx = at_id_ground_truth_dict[at_i]["posx"]
            at_i_posy = at_id_ground_truth_dict[at_i]["posy"]
            at_i_rotz = at_id_ground_truth_dict[at_i]["rotz"]

            # store the results inside the member dict (verdict) of each ApriltagDetection object
            # Mean Values
            (measurements[at_i].evaluation["posx_med"]["err"], measurements[at_i].evaluation["posx_med"]["pass"]) = \
                self.thresholding(med_posx, at_i_posx, self.conf["translational_error_tolerance"])
            (measurements[at_i].evaluation["posy_med"]["err"], measurements[at_i].evaluation["posy_med"]["pass"]) = \
                self.thresholding(med_posy, at_i_posy, self.conf["translational_error_tolerance"])
            (measurements[at_i].evaluation["rotz_med"]["err"], measurements[at_i].evaluation["rotz_med"]["pass"]) = \
                self.thresholding(med_rotz, at_i_rotz, self.conf["rotational_error_tolerance"])
            # Standart Deviation
            (measurements[at_i].evaluation["posx_std"]["err"], measurements[at_i].evaluation["posx_std"]["pass"]) = \
                self.thresholding(std_posx, 0, self.conf["translational_std_tolerance"])
            (measurements[at_i].evaluation["posy_std"]["err"], measurements[at_i].evaluation["posy_std"]["pass"]) = \
                self.thresholding(std_posy, 0, self.conf["translational_std_tolerance"])
            (measurements[at_i].evaluation["rotz_std"]["err"], measurements[at_i].evaluation["rotz_std"]["pass"]) = \
                self.thresholding(std_rotz, 0, self.conf["rotational_std_tolerance"])

    def get_error_stat(self):
        error_stat = {}
        for at_i in self.measurements:
            at_i_eval_dict = self.measurements[at_i].evaluation
            error_stat[at_i] = at_i_eval_dict
        return error_stat

    def get_verdict(self):
        for at_i in self.measurements.keys():
            at_i_eval_dict = self.measurements[at_i].evaluation
            #print(at_i_eval_dict)
            for at_pose in at_i_eval_dict.keys():
                #rospy.logwarn("at pose: {}\ndict: {}".format(at_pose, at_i_eval_dict[at_pose]))
                verdict = at_i_eval_dict[at_pose]["pass"]
                if verdict == False:
                    return False
        return True

    def generate_report(self):
        # base-report content, entries are valid for both validation and optimization operations
        yaml_dict = {
            'config_version': self.conf["config_file_version"],
            'hostname': get_hostname(),
            'platform': get_cpu_info(),
            'experiment_time': self.time_label,
            'verdict': self.get_verdict(),
            'measurements': self.get_error_stat(),
            'experiment_name': self.experiment_name
        }

        # create the report file
        report = os.path.join(self.results_dir, 'camera_test_report.yaml')
        os.mknod(report)
        # write the content into the report
        yaml_write_to_file(yaml_dict, report)

    @staticmethod
    def thresholding(x_meas, x_ground_truth, x_threshold):
        err = abs(x_meas - x_ground_truth)
        if err < x_threshold:
            return err.item(), True
        return err.item(), False

    def copy_and_update_config(self):
        """ update the config file with the latest commit id, see get_software_version for further info"""
        commit_id = get_software_version(get_package_root(self.package_name))
        if commit_id is not False:
            self.conf["config_file_version"] = commit_id
        yaml_write_to_file(self.conf, os.path.join(self.results_dir, "config.yaml"))

    def process_at_array(self):
        """ generate a dictionary with
        key: apriltag_id
        value: an AprilTagDetection instance
        """
        seen_at = {}
        for i, at_array_i in enumerate(self.recieved_at_position_estimates):
            for at in at_array_i.local_pose_list: #detections in the field of view
                if at.id in self.allowed_at_id: #disregard unexpected tags
                    if at.id not in seen_at:
                        at_i = AprilTagDetection(id=at.id)
                        seen_at[at.id] = at_i
                    seen_at[at.id].add("px", at.posx)
                    seen_at[at.id].add("py", at.posy)
                    seen_at[at.id].add("rz", at.rotz)
        return seen_at

    def load_test_config(self):
        from os.path import join
        package_root = get_package_root(self.package_name)
        meta_conf = yaml_load_file(join(package_root,"meta_config.yaml"))
        cam_cal_test_ver = meta_conf["camera_calibration_test_config_version"]
        cam_cal_test_conf = yaml_load_file(join(package_root,"configs", "camera_calibration", "camera_calibration_test_" + str(cam_cal_test_ver) + ".yaml"))
        return cam_cal_test_conf

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

    def prepare_to_leave(self):
        # copy camera calibration files
        copy_calibrations_folder(self.results_dir)
        # copy updated config file
        self.copy_and_update_config()
        # generate report
        self.generate_report()
        # create a zip from the resulting folder
        pack_results(self.results_dir)

class AprilTagDetection():
    def __init__(self, id=None, size=None):
        self.pose = {
            'px': [],'py': [],'pz': [],
            'rx':[],'ry':[],'rz':[],
            'timestamp': []
        }

        # whether at is within the allowed tolerance
        self.evaluation = {
        "posx_med": {"pass": False, "err": None},
        "posy_med": {"pass": False, "err": None},
        "rotz_med": {"pass": False, "err": None},
        "posx_std": {"pass": False, "err": None},
        "posy_std": {"pass": False, "err": None},
        "rotz_std": {"pass": False, "err": None}
        }

        self.id = id
        self.size = size

    def add(self, pose_key , pose_val):
        self.pose[pose_key].append(pose_val)

    def stat(self, pose_key):
        from numpy import average, std
        return average(self.pose[pose_key]), std(self.pose[pose_key])


if __name__ == '__main__':
    camera_calibration_test = CameraCalibrationTest()
    camera_calibration_test.perform_experiments()