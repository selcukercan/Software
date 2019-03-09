#!/usr/bin/env python
# system imports
import rospy
import rosbag
from os.path import expanduser
from os import remove
from collections import deque
from threading import Lock
from shutil import copy
# package utilities import
from std_msgs.msg import String, Bool
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import WheelsCmdStamped, Twist2DStamped, AprilTagDetectionArray, LanePose, VehiclePoseEuler
from calibration.wheel_cmd_utils import *
from calibration.time_sync_utils import time_sync

class MeasurementBuffer:
    def __init__(self):
        # namespace variables
        host_package = rospy.get_namespace()  # as defined by <group> in launch file
        self.node_name = 'measurement_buffer'  # node name , as defined in launch file
        host_package_node = host_package + self.node_name
        self.veh = host_package.split('/')[1]

        # Initialize the node with rospy
        rospy.init_node('MeasurementBuffer', anonymous=True)

        active_methods = ['lane_filter']
        self.method_objects = self.construct_localization_method_objects(active_methods)

        # Parameters
        # determine we work synchronously or asynchronously, where asynchronous is the default
        # mode of operation. synchronous operation is benefitial when post-processing the recorded
        # experiment data. For example it is beneficial when only compressed image is available from the experiment and we want to
        # pass exach image through a localization pipeline (compressed_image -> decoder -> rectification -> apriltags_detection -> to_local_pose)
        # to extract the pose in world frame
        self.synchronous_mode = rospy.get_param('/' + self.veh + '/calibration/measurement_buffer/operation_mode')

        if self.synchronous_mode:
            rospy.logwarn('[publish_detections_in_local_frame] operating in synchronous mode')
            # wait until the message_count has been set by the buffer node
            while not rospy.has_param("/" + self.veh + "/buffer_node/message_count"):
                rospy.sleep(1)
                rospy.loginfo("[{}] waiting for buffer node to set message_count".format(self.node_name))

            # read the messages from the buffer node
            self.total_msg_count = rospy.get_param(param_name="/" + self.veh + "/buffer_node/message_count")
            rospy.logwarn("TOTAL_MSG_COUNT: {}".format(self.total_msg_count))
            # request image after processing of a single image is completed
            self.pub_topic_image_request = "/" + self.veh + "/" + self.node_name + "/" + "image_requested"
            self.pub_image_request = rospy.Publisher(self.pub_topic_image_request, Bool, queue_size=1)

            # get the input rosbags, and name of the output bag we wish the create
            input_bag = rospy.get_param(param_name= host_package_node + "/input_rosbag")
            self.output_bag = rospy.get_param(param_name= host_package_node + "/output_rosbag")

            # wrap bag file operations with a lock as rospy api is not threat-safe.
            self.lock = Lock()
            self.lock.acquire()
            copy(input_bag, self.output_bag)
            self.lock.release()

            self.numb_written_images = 0
            self.wrote_all_images = False
        else:
            rospy.logwarn('[measurement_buffer] operating in asynchronous mode')

        """
        # ROS Parameters
        self.pm_send_status = "/" + self.veh + "/buffer_node/process_status"
        self.send_status = self.setupParam(self.pm_send_status,0)
        self.pm_message_count = "/" + self.veh + "/buffer_node/message_count"
        self.message_count = self.setupParam(self.pm_message_count, -1) # set to a non-positive number for ease of debugging

        # Rosbag related parameters
        path_to_bag = rospy.get_param(param_name= host_package_node + "/input_rosbag")
        self.cur_image_i = 0
        self.send_out_all_images = False

        # Read the compressed image topic from the bag
        topicname = "/" + self.veh + "/camera_node/image/compressed"
        self.input_rosbag = rosbag.Bag(path_to_bag)
        self.total_msg_count = self.input_rosbag.get_message_count(topicname)
        self.view_generator = self.input_rosbag.read_messages(topics=topicname) # create a rosbag generator

        rospy.set_param(self.pm_message_count, self.total_msg_count)

        init_wait = 5
        rospy.loginfo("[{}] Waiting for {} seconds to ensure all other processes have launched".format(self.node_name, init_wait))
        rospy.sleep(init_wait)

        self.pub_single_compressed_image() # send the first image
        """

    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def construct_localization_method_objects(self, active_methods):
        method_objects = {}
        for method in active_methods:
            if method == 'apriltag':
                obj = LocalizationMethod(method_name=method,
                sub_top='/' + self.veh + '/apriltags2_ros/publish_detections_in_local_frame/tag_detections_local_frame',
                pub_top='/' + self.veh + '/calibration/measurement_buffer/tag_detections_local_frame')
                obj.set_sub(topic_type=VehiclePoseEuler, callback_fn=self.cb_apriltag)
                obj.set_pub(topic_type=VehiclePoseEuler, queue_size=1)
            elif method == 'lane_filter':
                obj = LocalizationMethod(method_name=method,
                sub_top='/' + self.veh + '/lane_filter_node/lane_pose',
                pub_top='/' + self.veh + '/calibration/measurement_buffer/lane_pose')
                obj.set_sub(topic_type=LanePose, callback_fn=self.cb_lane_filter)
                obj.set_pub(topic_type=LanePose, queue_size=1)
            else:
                rospy.logwarn('[{}] invalid localization method requested'.format(self.node_name))
            method_objects[method] = obj
        return method_objects

    def form_message(self):
        pass

    def cb_lane_filter(self, msg):
        self.method_objects['lane_filter'].add(msg)
        ready = self.check_ready_to_publish()

        if ready: self.pub_messages()

    def cb_apriltag(self, msg):
        self.method_objects['apriltag'].add(msg)

        ready = self.check_ready_to_publish()
        if ready: self.pub_messages()


        if self.synchronous_mode:
            # save the message to the bag file that contains compressed_images
            self.lock.acquire()
            output_rosbag = rosbag.Bag(self.output_bag, 'a') # open bag to write
            output_rosbag.write(self.method_objects['apriltag'].sub_top, msg)
            output_rosbag.close()
            self.lock.release()
            rospy.loginfo("[{}] wrote image {}".format(self.node_name, self.numb_written_images))

            self.numb_written_images += 1

            if self.numb_written_images == self.total_msg_count:
                time_sync(self.output_bag, self.veh)

    def check_ready_to_publish(self):
        localization_method_dict = self.method_objects
        for method_name in localization_method_dict.keys():
            if not localization_method_dict[method_name].ready():
                return False
        return True

    def pub_messages(self):
        localization_method_dict = self.method_objects
        for method_name in localization_method_dict.keys():
            method_obj = localization_method_dict[method_name]
            method_obj._publish()
        # request a new image from "buffer.py"
        req_msg = Bool(True)
        self.pub_image_request.publish(req_msg)

class LocalizationMethod():
    def __init__(self, method_name=None, sub_top=None, pub_top=None):
        self.method_name = method_name
        self.sub_top = sub_top
        self.pub_top = pub_top
        self.sub = None
        self.pub = None
        self.hist = deque()

    def _publish(self):
        self.pub.publish(self.hist.pop())
    def ready(self):
        if len(self.hist) == 0:
            return False
        else:
            return True
    def set_sub(self, topic_type=None, callback_fn=None):
        self.sub=rospy.Subscriber(self.sub_top, topic_type, callback_fn, queue_size=None)
    def set_pub(self, topic_type=None, queue_size=None):
        self.pub=rospy.Publisher(self.pub_top, topic_type, queue_size=queue_size)
    def add(self, msg):
        self.hist.append(msg)
if __name__ == '__main__':
    measurement_buffer = MeasurementBuffer()
    rospy.spin()
