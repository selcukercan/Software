#!/usr/bin/env python
# system imports
import rospy
import rosbag
import os

# package utilities import
from std_msgs.msg import String, Bool
from sensor_msgs.msg import CompressedImage, CameraInfo
from duckietown_msgs.msg import WheelsCmdStamped, Twist2DStamped, AprilTagDetectionArray
from duckietown_msgs.msg import VehiclePoseEuler, RosbagInfo
from calibration.wheel_cmd_utils import *
from calibration.utils import get_files_in_dir

class Buffer:
    def __init__(self):
        # namespace variables
        self.host_package = rospy.get_namespace()  # as defined by <group> in launch file
        self.node_name = 'buffer'  # node name , as defined in launch file
        host_package_node = self.host_package + self.node_name
        self.veh = self.host_package.split('/')[1]

        # Initialize the node with rospy
        rospy.init_node('Buffer', anonymous=True)

        # Subscribers
        sub_topic_image_request = '/' + self.veh + '/measurement_buffer/image_requested'
        self.sub_processed_image = rospy.Subscriber(sub_topic_image_request, Bool, self.cb_recieved_ack, queue_size=1)
        sub_topic_processed_bag = '/' + self.veh + '/measurement_buffer/bag_requested'
        self.sub_processed_bag = rospy.Subscriber(sub_topic_processed_bag, Bool, self.cb_bag_req, queue_size=1)

        # Publishers
        pub_topic_compressed_image = '/' + self.veh + "/buffer_node/image/compressed"
        pub_topic_camera_info = '/' + self.veh + "/buffer_node/camera_info"
        pub_topic_wheels_cmd_executed = '/' + self.veh + '/wheels_driver_node/wheels_cmd_executed'
        pub_topic_rosbag_info = '/' + self.veh + '/buffer_node/rosbag_info'

        self.pub_compressed_image = rospy.Publisher(pub_topic_compressed_image, CompressedImage, queue_size=1)
        self.pub_camera_info = rospy.Publisher(pub_topic_camera_info, CameraInfo, queue_size=1)
        self.pub_wheels_cmd_exec = rospy.Publisher(pub_topic_wheels_cmd_executed, WheelsCmdStamped, queue_size=1)
        self.pub_rosbag_info = rospy.Publisher(pub_topic_rosbag_info, RosbagInfo, queue_size=1)

        # Read the compressed image topic from the bag
        self.image_topicname = "/" + self.veh + "/camera_node/image/compressed"
        self.wheels_cmd_topicname = "/" + self.veh + '/wheels_driver_node/wheels_cmd_executed'
        # Read the cam_info topic from the bag
        self.cam_info_topicname = "/" + self.veh + "/camera_node/camera_info"

        # Be cautios and wait a bit to ensure other processes are started, may not be necessary.
        init_wait = 5
        rospy.loginfo(
            "[{}] Waiting for {} seconds to ensure all other processes have launched".format(self.node_name, init_wait))
        rospy.sleep(init_wait)

        # Rosbags-related parameters
        self.path_to_bags = rospy.get_param(param_name= host_package_node + "/input_path")
        self.bags_list = get_files_in_dir(self.path_to_bags)
        rospy.loginfo("[{}] bags found under {}:  {}".format(self.node_name, self.path_to_bags, str(self.bags_list)))
        self.bag_i = 0

        # Start processing one bagin the directory
        self.process_single_bag()

    def cb_bag_req(self, msg):
        if msg.data == True:
            rospy.loginfo('[{}] recieved a callback from bag_request with value TRUE'.format(self.node_name))
            self.process_single_bag()
        else:
            rospy.loginfo('[{}] recieved a callback from bag_request with value FALSE'.format(self.node_name))

    def process_single_bag(self):
        if len(self.bags_list) != 0:
            bag_name = self.bags_list.pop()
            self.bag_i += 1

            rospy.loginfo('\n\n[{}] started processing bag number: {} \tname: {}\n\n'.format(self.node_name, self.bag_i, bag_name))

            path_to_bag = os.path.join(self.path_to_bags, bag_name)

            # initiate a rosbag object
            self.input_rosbag = rosbag.Bag(path_to_bag)
            self.view_generator = self.input_rosbag.read_messages(topics=[self.image_topicname, self.wheels_cmd_topicname])
            self.view_generator_cam_info = self.input_rosbag.read_messages(topics=self.cam_info_topicname)

            # create/publish message to be recieved by measurement_buffer node
            msg_rosbag_info = RosbagInfo()
            msg_rosbag_info.message_count = self.total_msg_count = self.input_rosbag.get_message_count(self.image_topicname)
            msg_rosbag_info.rosbag_path = path_to_bag
            self.pub_rosbag_info.publish(msg_rosbag_info)

            # keep track of the number of sent images
            self.cur_image_i = 0
            self.send_out_all_images = False

            self.pub_single_compressed_image() # send the first image
            self.publish_single_cam_info() # publish camera info only once // necessary for for lane filter by ground_projection_node
        else:
            rospy.loginfo('[{}] finished processing all {} bags'.format(self.node_name,self.bag_i))

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
