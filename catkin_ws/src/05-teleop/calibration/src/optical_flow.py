#!/usr/bin/env python
# system imports
import rospy
import rosbag
import cv2
from RLOFLib import rlof
import numpy as np
import time
from os.path import expanduser
from os import remove

# package utilities import
from std_msgs.msg import String, Bool
from sensor_msgs.msg import CompressedImage

# set rlof parameters
parameters = {  "SolverType" : "ST_BILINEAR",
                "SupportRegionType" : "SR_CROSS",
                "maxLevel" : 9,
                "maxIter" : 30,
                "largeWinSize" : 21,
                "smallWinSize"  : 9,
                "HampelNormS0"  : 3.2,
                "HampelNormS1": 7.0 ,
				"segmentationThreshold": 25 ,
				"RansacReprojThresholdPercentil": 10.0 ,
				"minEigenvalue": 0.0001 ,
				"useIlluminationModel": True ,
				"useGlobalMotionPrior": False
				}

class OpticalFlow:
    def __init__(self):
        # namespace variables
        host_package = rospy.get_namespace()  # as defined by <group> in launch file
        self.node_name = 'buffer'  # node name , as defined in launch file
        host_package_node = host_package + self.node_name
        self.veh = host_package.split('/')[1]

        # Initialize the node with rospy
        rospy.init_node('OpticalFlow', anonymous=True)
        """
        # Subscriber
        sub_topic_image = '/' + self.veh + '/camera_node/image/rect'
        self.sub_processed_image = rospy.Subscriber(sub_topic_image_request, Bool, self.cb_recieved_ack, queue_size=1)
        #self.recieved_pub_image_request = False

        # Publisher
        pub_topic_optical_flow_field = '/' + self.veh + "/optical_flow_node/optical_flow_field"
        self.pub_compressed_image = rospy.Publisher(pub_topic_optical_flow_field, CompressedImage, queue_size=1)

        # ROS Parameters
        self.pm_send_status = "/" + self.veh + "/buffer_node/process_status"
        self.send_status = self.setupParam(self.pm_send_status,0)
        self.pm_message_count = "/" + self.veh + "/buffer_node/message_count"
        self.message_count = self.setupParam(self.pm_message_count, -1) # set to a non-positive number for ease of debugging

        # Rosbag related parameters
        #path_to_bag = "/home/selcuk/input.bag"
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
        """
        self.calculate_optical_flow() # send the first image


    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def calculate_optical_flow(self):
        prevImg = cv2.imread('../Doc/ErnstReuter1.png')
        currImg = cv2.imread('../Doc/ErnstReuter2.png')

        rlofProc = rlof.RLOFEstimator()
        rlofProc.set_param(parameters["SolverType"],
                           parameters["SupportRegionType"],
                           parameters["maxLevel"], parameters["maxIter"],
                           parameters["largeWinSize"],
                           parameters["smallWinSize"],
                           parameters["HampelNormS0"],
                           parameters["HampelNormS1"],
                           parameters["segmentationThreshold"],
                           parameters["RansacReprojThresholdPercentil"],
                           parameters["minEigenvalue"],
                           parameters["useIlluminationModel"],
                           parameters["useGlobalMotionPrior"])

        # prepare pointlist
        (h, w) = (prevImg.shape[0], prevImg.shape[1])
        a = np.meshgrid(np.arange(9, w, 10), np.arange(9, h, 10))
        prevPoints = np.vstack((a[0].ravel(), a[1].ravel())).transpose().astype(np.float32).copy()

        # sparse optical flow estimation e.g. to compute a grid of motion vectors
        start = time.time();
        currPoints = rlofProc.sparse_flow(prevImg, currImg, prevPoints)
        end = time.time();
        print("\nSparse Optical Flow Estimation\n")
        print("#Features = " + str(prevPoints.shape[0]))
        print("Runtime[sec] = " + str(end - start));

        # draw sparse motion vectors
        sparseFlowImg = prevImg.copy()
        for i, (new, old) in enumerate(zip(currPoints, prevPoints)):
            a, b = new.ravel()
            c, d = old.ravel()
            if (a >= 0 and a < prevImg.shape[1] and b >= 0 and b < prevImg.shape[0]):
                sparseFlowImg = cv2.line(sparseFlowImg, (a, b), (c, d), (0, 255, 0), 1)

        # write results
        cv2.imwrite("SparseFlow.png", sparseFlowImg)


if __name__ == '__main__':
    buffer = OpticalFlow()
    rospy.spin()
