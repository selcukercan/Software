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
        sub_topic_image_request = '/' + self.veh + '/publish_detections_in_local_frame/image_requested' # 30 Hz
        self.sub_proccesed_image = rospy.Subscriber(sub_topic_image_request, Bool, self.cb_recieved_ack, queue_size=1)
        #self.recieved_pub_image_request = False

        # Publisher
        pub_topic_compressed_image= '/' + self.veh + "/buffer_node/image/compressed"
        self.pub_compressed_image = rospy.Publisher(pub_topic_compressed_image, CompressedImage, queue_size=1)

        # ROS Parameters
        self.paramname = "/" + self.veh + "/buffer_node/process_status"
        self.processing_status = self.setupParam(self.paramname,0)

        # Rosbag related parameters
        #path_to_bag = "/home/selcuk/input.bag"
        path_to_bag = rospy.get_param(param_name= host_package_node + "/input_rosbag")
        print path_to_bag + "*******************"
        self.cur_image_i = 0
        self.finished_processing = False
        # Read the compressed image topic from the bag
        topicname = "/" + self.veh + "/camera_node/image/compressed"
        self.input_rosbag = rosbag.Bag(path_to_bag)
        self.total_msg_count = self.input_rosbag.get_message_count(topicname)
        self.view_generator = self.input_rosbag.read_messages(topics=topicname) # create a rosbag generator

        init_wait = 3
        rospy.loginfo("[{}] Waiting for {} seconds to ensure all other processes have luanched".format(self.node_name, init_wait))
        rospy.sleep(init_wait)

        self.pub_single_compressed_image() # send the first image


    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def cb_recieved_ack(self, msg):
        if self.finished_processing == False:
            print("recieved new image request, and have more to sent")
            self.pub_single_compressed_image()
        else:
            print("!!!!!!!!!! SHOULD NOT BE HERE 1")

    def pub_single_compressed_image(self):
        if self.cur_image_i < self.total_msg_count: # if not all the messages are sent yet.
            print self.cur_image_i
            view_output = next(self.view_generator) # view_output class : <class 'rosbag.bag.BagMessage'>, has return value (topic_name, msg, time_stamp of the message)

            # message generation
            msg = CompressedImage() # create a compressed image object to publish
            msg = view_output.message # message content of the view is what we want to publish
            rospy.loginfo("[{}] publishing image {}".format(self.node_name, self.cur_image_i))
            self.pub_compressed_image.publish(msg) # publish the message
            self.cur_image_i += 1
        else: # after sending all messages close the bag
            if self.finished_processing == False:
                rospy.loginfo('[{}] processed all the messages, now closing the bag.'.format(self.node_name))
                self.processing_status = rospy.set_param(self.paramname,1)
                self.input_rosbag.close()
                self.finished_processing = True
            else:
                print("!!!!!!!!!! SHOULD NOT BE HERE 2 ")

if __name__ == '__main__':
    buffer = Buffer()
    rospy.spin()
