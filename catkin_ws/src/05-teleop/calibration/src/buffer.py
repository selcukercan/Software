#!/usr/bin/env python
# system imports
import rospy
import rosbag
import os

# package utilities import
from std_msgs.msg import String, Bool
from sensor_msgs.msg import CompressedImage, CameraInfo
from duckietown_msgs.msg import WheelsCmdStamped, Twist2DStamped, AprilTagDetectionArray
from duckietown_msgs.msg import VehiclePoseEuler
from calibration.wheel_cmd_utils import *

class Buffer:
    def __init__(self):
        # namespace variables
        host_package = rospy.get_namespace()  # as defined by <group> in launch file
        self.node_name = 'buffer'  # node name , as defined in launch file
        host_package_node = host_package + self.node_name
        self.veh = host_package.split('/')[1]

        # Initialize the node with rospy
        rospy.init_node('Buffer', anonymous=True)

        # Subscriber
        sub_topic_image_request = '/' + self.veh + '/measurement_buffer/image_requested'
        self.sub_processed_image = rospy.Subscriber(sub_topic_image_request, Bool, self.cb_recieved_ack, queue_size=1)
        sub_topic_processed_bag = '/' + self.veh + '/measurement_buffer/bag_requested'
        self.sub_processed_bag = rospy.Subscriber(sub_topic_processed_bag, Bool, self.cb_bag_req, queue_size=1)

        # Publisher
        pub_topic_compressed_image= '/' + self.veh + "/buffer_node/image/compressed"
        pub_topic_camera_info= '/' + self.veh + "/buffer_node/camera_info"
        pub_topic_wheels_cmd_executed= '/' + self.veh + '/wheels_driver_node/wheels_cmd_executed'

        self.pub_compressed_image = rospy.Publisher(pub_topic_compressed_image, CompressedImage, queue_size=1)
        self.pub_camera_info = rospy.Publisher(pub_topic_camera_info, CameraInfo, queue_size=1)
        self.pub_wheels_cmd_exec = rospy.Publisher(pub_topic_wheels_cmd_executed, WheelsCmdStamped, queue_size=1)

        # Read the compressed image topic from the bag
        self.image_topicname = "/" + self.veh + "/camera_node/image/compressed"
        self.wheels_cmd_topicname = "/" + self.veh + '/wheels_driver_node/wheels_cmd_executed'
        # Read the cam_info topic from the bag
        self.cam_info_topicname = "/" + self.veh + "/camera_node/camera_info"

        init_wait = 5
        rospy.loginfo(
            "[{}] Waiting for {} seconds to ensure all other processes have launched".format(self.node_name, init_wait))
        rospy.sleep(init_wait)

        # Rosbags to related parameters
        self.path_to_bags = rospy.get_param(param_name= host_package_node + "/input_path")
        self.bags_list = os.listdir(self.path_to_bags)
        rospy.logwarn(self.bags_list)
        self.bag_i = 0

        # Start processing one bagin the directory
        self.process_single_bag()

    def cb_bag_req(self, msg):
        if msg == True:
            rospy.loginfo('[{}] recieved a callback from bag_request with value TRUE'.format(self.node_name))
            self.process_single_bag()
        else:
            rospy.loginfo('[{}] recieved a callback from bag_request with value FALSE'.format(self.node_name))

    def process_single_bag(self):
        bag_name = self.bags_list.pop()
        self.bag_i += 1
        rospy.loginfo('\n\n[{}] started processing bag number: {} \tname: {}\n\n'.format(self.node_name, self.bag_i, bag_name))

        # Information about bag content
        path_to_bag = os.path.join(self.path_to_bags, bag_name)
        self.input_rosbag = rosbag.Bag(path_to_bag)
        rospy.logwarn(path_to_bag)
        self.total_msg_count = self.input_rosbag.get_message_count(self.image_topicname)
        self.view_generator = self.input_rosbag.read_messages(topics=[self.image_topicname, self.wheels_cmd_topicname])  # create a rosbag generator

        # ROS Parameters
        self.pm_send_status = "/" + self.veh + "/buffer_node/process_status"
        self.send_status = self.setupParam(self.pm_send_status,0)
        self.pm_message_count = "/" + self.veh + "/buffer_node/message_count"
        self.message_count = self.setupParam(self.pm_message_count, -1) # set to a non-positive number for ease of debugging

        self.cur_image_i = 0
        self.send_out_all_images = False

        # Generate a cam_info view
        self.view_generator_cam_info = self.input_rosbag.read_messages(topics=self.cam_info_topicname) # create a rosbag generator

        rospy.set_param(self.pm_message_count, self.total_msg_count)

        self.pub_single_compressed_image() # send the first image
        self.publish_single_cam_info() # publish camera info only once // necessary for for lane filter by ground_projection_node

    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def cb_recieved_ack(self, msg):
        if self.send_out_all_images == False:
            rospy.loginfo('[{}] recieved new image request'.format(self.node_name))
            self.pub_single_compressed_image()
        else:
            rospy.logfatal("[{}] must not be here 0".format(self.node_name))

    def pub_single_compressed_image(self):
        """ responsible for publishing one compressed image
        it transverses the rosbag wrt timestamps.
        publishes the wheel commands till it finds the next compressed image as required by model-based lane filter to estimate the velocities.
        """

        if self.cur_image_i < self.total_msg_count: # if not all the messages are sent yet.
            # http://docs.ros.org/jade/api/rosbag/html/python/rosbag.bag.BagMessage-class.html
            # view_output class : <class 'rosbag.bag.BagMessage'>, has return value (topic_name, msg, time_stamp of the message)
            view_output = next(self.view_generator)
            while view_output.topic != self.image_topicname:
                if view_output.topic == self.wheels_cmd_topicname:
                    msg = view_output.message  # message content of the view is what we want to publish
                    rospy.loginfo("[{}] publishing wheels cmd".format(self.node_name))
                    self.pub_wheels_cmd_exec.publish(msg)  # publish the message
                else:
                    rospy.logfatal("[{}] must not be here 1".format(self.node_name))
                view_output = next(self.view_generator) # generate the next view
            else:
                if view_output.topic == self.image_topicname:
                    # message generation
                    msg = CompressedImage() # create a compressed image object to publish
                    msg = view_output.message # message content of the view is what we want to publish
                    rospy.loginfo("[{}] publishing image {}".format(self.node_name, self.cur_image_i))
                    self.pub_compressed_image.publish(msg) # publish the message
                    self.cur_image_i += 1
                else:
                    rospy.logfatal("[{}] must not be here 2".format(self.node_name))
        else: # after sending all messages close the bag
            if self.send_out_all_images == False:
                rospy.loginfo('[{}] send all the messages'.format(self.node_name))
                self.input_rosbag.close()
                self.send_out_all_images = True
                self.send_status = rospy.set_param(self.pm_send_status,1)
            else:
                rospy.logfatal("[{}] must not be here 3".format(self.node_name))

    def publish_single_cam_info(self):
        view_output = next(self.view_generator_cam_info) # view_output class : <class 'rosbag.bag.BagMessage'>, has return value (topic_name, msg, time_stamp of the message)
        msg = view_output.message # message content of the view is what we want to publish
        rospy.loginfo("[{}] publishing camera info".format(self.node_name))
        self.pub_camera_info.publish(msg) # publish the message

if __name__ == '__main__':
    buffer = Buffer()
    rospy.spin()
