#!/usr/bin/env python
# system imports
import rospy
import rosbag
import os
from collections import deque
from threading import Lock
from shutil import copy
# package utilities import
from std_msgs.msg import String, Bool
from duckietown_msgs.msg import LanePose, VehiclePoseEulerArray, RosbagInfo
from calibration.wheel_cmd_utils import *
from calibration.time_sync_utils import time_sync
from calibration.utils import safe_create_dir

class MeasurementBuffer:
    def __init__(self):
        # namespace variables
        self.host_package = rospy.get_namespace()  # as defined by <group> in launch file
        self.node_name = 'measurement_buffer'  # node name , as defined in launch file
        host_package_node = self.host_package + self.node_name
        self.veh = self.host_package.split('/')[1]

        # Initialize the node with rospy
        rospy.init_node('MeasurementBuffer', anonymous=True)

        available_methods = ['apriltag', 'lane_filter']
        active_methods = self.set_active_methods(available_methods)
        self.method_objects = self.construct_localization_method_objects(active_methods)

        sub_topic_rosbag_info = '/' + self.veh + '/buffer_node/rosbag_info'
        self.sub_processed_image = rospy.Subscriber(sub_topic_rosbag_info, RosbagInfo, self.cb_rosbag_info, queue_size=1)

        # Parameters
        # determine we work synchronously or asynchronously, where asynchronous is the default
        # mode of operation. synchronous operation is benefitial when post-processing the recorded
        # experiment data. For example it is beneficial when only compressed image is available from the experiment and we want to
        # pass exach image through a localization pipeline (compressed_image -> decoder -> rectification -> apriltags_detection -> to_local_pose)
        # to extract the pose in world frame
        self.synchronous_mode = rospy.get_param('/' + self.veh + '/calibration/measurement_buffer/operation_mode')
        input_folder = rospy.get_param('/' + self.veh + '/calibration/measurement_buffer/input_path')
        if self.synchronous_mode:
            rospy.logwarn('[publish_detections_in_local_frame] operating in synchronous mode')

            # request image after processing of a single image is completed
            self.pub_topic_image_request = "/" + self.veh + "/" + self.node_name + "/" + "image_requested"
            self.pub_image_request = rospy.Publisher(self.pub_topic_image_request, Bool, queue_size=1)

            self.pub_topic_processed_bag = "/" + self.veh + "/" + self.node_name + "/" + "bag_requested"
            self.pub_bag_request = rospy.Publisher(self.pub_topic_processed_bag, Bool, queue_size=1)

            # create a directory to to save the post-processed files
            self.output_path = safe_create_dir(os.path.join(input_folder, 'post_processed'))
        else:
            rospy.logwarn('[measurement_buffer] operating in asynchronous mode')

    def cb_rosbag_info(self, msg):
        # unpack the message content
        self.total_msg_count = msg.message_count
        input_bag = msg.rosbag_path

        # output to be placed under 'post_processed' with the same name as input
        self.output_bag = os.path.join(self.output_path, os.path.basename(input_bag))

        self.lock = Lock() #wrap bag file operations with a lock as rospy api is not threat-safe
        self.lock.acquire()
        copy(input_bag, self.output_bag) # copy the input bag
        self.lock.release()

        # keep track of number of images that are added to the output rosbag
        self.numb_written_images = 0

    def set_active_methods(self, available_methods):
        active_methods = []

        if rospy.get_param('/calibration_use_apriltags'):
            active_methods.append('apriltag')
        if rospy.get_param('/calibration_use_lane_filter'):
            active_methods.append('lane_filter')
        return active_methods


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
                sub_top='/' + self.veh + '/apriltags2_ros/publish_detections_in_local_frame/tag_detections_array_local_frame',
                pub_top='/' + self.veh + '/calibration/measurement_buffer/tag_detections_array_local_frame')
                obj.set_sub(topic_type=VehiclePoseEulerArray, callback_fn=self.cb_apriltag)
                obj.set_pub(topic_type=VehiclePoseEulerArray, queue_size=1)
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

    def cb_lane_filter(self, msg):
        self.method_objects['lane_filter']._add(msg)

        if self.check_ready_to_publish():
            self.pub_messages()
            self.write_bag()

    def cb_apriltag(self, msg):
        self.method_objects['apriltag']._add(msg)
        if self.check_ready_to_publish():
            self.pub_messages()
            self.write_bag()

    def write_bag(self):
        # save the message to the bag file that contains compressed_images
        self.lock.acquire()
        output_rosbag = rosbag.Bag(self.output_bag, 'a') # open bag to write
        for method in self.method_objects.keys():
            method_object = self.method_objects[method]
            output_rosbag.write(method_object.sub_top, method_object.last_publish)
        output_rosbag.close()
        self.lock.release()
        self.numb_written_images += 1
        rospy.loginfo("[{}] wrote image {}".format(self.node_name, self.numb_written_images))

        if self.numb_written_images == self.total_msg_count:
            time_sync_lock = Lock()
            time_sync_lock.acquire()
            time_sync(self.output_bag, self.veh)
            time_sync_lock.release()
            # request a new bag from "buffer.py"
            req_bag = Bool(True)
            self.pub_bag_request.publish(req_bag)

    def check_ready_to_publish(self):
        localization_method_dict = self.method_objects
        for method_name in localization_method_dict.keys():
            if not localization_method_dict[method_name]._ready():
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
        self.last_publish = None

    def _publish(self):
        self.last_publish = self.hist.pop()
        self.pub.publish(self.last_publish)
    def _ready(self):
        if len(self.hist) == 0:
            return False
        else:
            return True
    def set_sub(self, topic_type=None, callback_fn=None):
        self.sub=rospy.Subscriber(self.sub_top, topic_type, callback_fn, queue_size=None)
    def set_pub(self, topic_type=None, queue_size=None):
        self.pub=rospy.Publisher(self.pub_top, topic_type, queue_size=queue_size)
    def _add(self, msg):
        self.hist.append(msg)


if __name__ == '__main__':
    measurement_buffer = MeasurementBuffer()
    rospy.spin()
