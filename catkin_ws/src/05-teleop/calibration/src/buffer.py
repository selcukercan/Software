#!/usr/bin/env python
# system imports
import rospy
import rosbag
from os.path import expanduser
from os import remove

# package utilities import
from std_msgs.msg import String, Bool
from sensor_msgs.msg import CompressedImage
from duckietown_msgs.msg import WheelsCmdStamped, Twist2DStamped, AprilTagDetectionArray
from duckietown_msgs.msg import VehiclePoseEuler
from calibration.wheel_cmd_utils import *
# Service types to be imported from rosbag_recorder package
from rosbag_recorder.srv import *
from collections import deque

class Buffer:
    def __init__(self):
        # namespace variables
        host_package = rospy.get_namespace()  # as defined by <group> in launch file
        node_name = 'buffer_node'  # node name , as defined in launch file
        host_package_node = host_package + node_name
        self.veh = host_package.split('/')[1]

        # Initialize the node with rospy
        rospy.init_node('Buffer', anonymous=True)

        # Subscriber
        sub_topic_image_process_status = '/' + self.veh + '/publish_detections_in_local_frame/image_requested' # 30 Hz
        self.sub_proccesed_image = rospy.Subscriber(sub_topic_image_process_status, Bool, self.cb_recieved_ack, queue_size=1)
        #self.recieved_pub_image_request = False

        # Publisher
        pub_topic_compressed_image= '/' + self.veh + "/buffer_node/image/compressed"
        self.pub_compressed_image = rospy.Publisher(pub_topic_compressed_image, CompressedImage, queue_size=1)

        # Rosbag related parameters
        path_to_bag = "/home/selcuk/input.bag"
        self.cur_image_i = 0

        topicname = "/" + self.veh + "/camera_node/image/compressed"
        self.input_rosbag = rosbag.Bag(path_to_bag)
        self.total_msg_count = self.input_rosbag.get_message_count(topicname)
        self.view_generator = self.input_rosbag.read_messages(topics=topicname) # create a rosbag generator object

        self.pub_single_compressed_image() # send first image
        self.cur_image_i += 1


    def cb_recieved_ack(self, msg):
        if msg.data == True:
            print("done processing, send a new image")
            self.pub_single_compressed_image()
        else:
            print("waiting for image processing to complete...")

    def pub_single_compressed_image(self):
        if self.cur_image_i < self.total_msg_count: # if not all the messages are sent yet.
            view_output = next(self.view_generator) # view_output class : <class 'rosbag.bag.BagMessage'>, has return value (topic_name, msg, time_stamp)

            # message generation
            msg = CompressedImage()
            msg.data = view_output[1]
            msg.header.stamp = view_output[2]
            msg.header.frame_id = self.cur_image_i # is this assignment a problem?

            print "Publishing image with stamp: {}".format(msg.header.stamp)
            self.pub_compressed_image.publish(msg) # publish the message
            self.cur_image_i += 1
        else: # after sending all messages close the bag
            rospy.loginfo('processed all the messages, now closing the bag.')
            self.input_rosbag.close()

if __name__ == '__main__':
    buffer = Buffer()
    rospy.spin()
