#!/usr/bin/env python
# system imports
import rospy
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, MultiArrayLayout
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from scipy.optimize import minimize
from RLOFLib import rlof
import numpy as np
import time

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
        self.node_name = 'optical_flow'  # node name , as defined in launch file
        host_package_node = host_package + self.node_name
        self.veh = host_package.split('/')[1]

        # initialize the node with rospy
        rospy.init_node('OpticalFlow', anonymous=True)

        # msg to cv bridge
        self.bridge = CvBridge()
        self.prev_cv_image = None
        self.image_id = 0

        # subscriber
        sub_topic_rect_image = '/' + self.veh + '/camera_node/image/rect'
        self.sub_rect_image = rospy.Subscriber(sub_topic_rect_image, Image, self.cbImage, queue_size=1)
        #self.recieved_pub_image_request = False

        # Publisher
        pub_topic_optical_flow_field = '/' + self.veh + "/optical_flow_node/optical_flow_field"
        self.pub_of_val= rospy.Publisher(pub_topic_optical_flow_field, Float64MultiArray, queue_size=1)

        """
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

    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def cbImage(self, msg):
        current_cv_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
        #rospy.loginfo("XXXXXXXXXXXX")
        #rospy.logwarn(type(self.prev_cv_image))
        #rospy.loginfo("XXXXXXXXXXXX")
        if not isinstance(self.prev_cv_image, np.ndarray):
            self.prev_cv_image= current_cv_image
        of = self.calculate_optical_flow(self.prev_cv_image, current_cv_image)
        self.pub_of_val.publish(self.ros_from_np_array(of))
        self.prev_cv_image = current_cv_image
        return of

    def calculate_optical_flow(self, prevImg, currImg):
        #rospy.logwarn(os.getcwd())
        #prevImg = cv2.imread('/home/selcuk/fSoftware/catkin_ws/src/05-teleop/calibration/include/RLOFLib/Doc/ErnstReuter1.png')
        #currImg = cv2.imread('/home/selcuk/fSoftware/catkin_ws/src/05-teleop/calibration/include/RLOFLib/Doc/ErnstReuter2.png')

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
        start = time.time()
        currPoints = rlofProc.sparse_flow(prevImg, currImg, prevPoints)
        end = time.time()
        self.image_id += 1
        rospy.loginfo("[{}] sparse optical flow for image: {} features: {} runtime[sec]: {}"
                      .format(self.node_name, self.image_id, str(prevPoints.shape[0]), str(end - start)))

        """
        # draw sparse motion vectors
        sparseFlowImg = prevImg.copy()
        for i, (new, old) in enumerate(zip(currPoints, prevPoints)):
            a, b = new.ravel()
            c, d = old.ravel()
            if (a >= 0 and a < prevImg.shape[1] and b >= 0 and b < prevImg.shape[0]):
                sparseFlowImg = cv2.line(sparseFlowImg, (a, b), (c, d), (0, 255, 0), 1)

        # write results
        cv2.imwrite("SparseFlow.png", sparseFlowImg)
        """
        return currPoints

    def flow_model(self, x, p):
        Ty, Tz, wx = p


    def cost_function(self, p, flow_model, F_meas):
        cost = None
        return cost


    def nonlinear_optimization(self, current_image):
        result = minimize(self.cost_function, self.p0, args=(self.flow_model(), current_image), bounds=self.bounds)
    @staticmethod
    def ros_from_np_array(data):
        if data.shape == ():
            msg = 'I do not know how to convert this: \n%s\n%s' % (data.dtype, data)
            raise NotImplementedError(msg)
        dims = []
        for i, size in enumerate(data.shape):
            label = 'dim%d' % i
            stride = 0
            dims.append(MultiArrayDimension(label=label, size=size, stride=stride))
        layout = MultiArrayLayout(dim=dims)

        d = list(data.flatten())
        msg = Float64MultiArray(data=d, layout=layout)
        return msg

if __name__ == '__main__':
    buffer = OpticalFlow()
    rospy.spin()
