#!/usr/bin/env python
# system imports
import rospy
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, MultiArrayLayout
import os
from cv_bridge import CvBridge
import cv2
from scipy.optimize import minimize
from RLOFLib import rlof
import numpy as np
import time
#from duckietown_utils.yaml_wrap import yaml_load_file
#from calibration.utils import get_workspace_param

# package utilities import
from std_msgs.msg import String, Bool
from sensor_msgs.msg import CompressedImage, Image

# set rlof parameters
parameters = {  "SolverType" : "ST_BILINEAR",
                "SupportRegionType" : "SR_CROSS",
                "maxLevel" : 9,
                "maxIter" : 50,
                "largeWinSize" : 21,
                "smallWinSize"  : 9,
                "HampelNormS0"  : 3.2,
                "HampelNormS1": 7.0,
				"segmentationThreshold": 50 ,
				"RansacReprojThresholdPercentil": 25.0 ,
				"minEigenvalue": 0.0001 ,
				"useIlluminationModel": True ,
				"useGlobalMotionPrior": False
				}

class OpticalFlow:
    def __init__(self):
        # namespace variables
        host_package = rospy.get_namespace()  # as defined by <group> in launch file
        self.node_name = 'optical_flow'  # node name , as defined in launch file

        #self.conf = yaml_load_file(get_workspace_param("path_to_config_file"), plain_yaml=True)

        # flow-control parameters
        DEBUG = True

        # namespace variables
        if not DEBUG:
            host_package = rospy.get_namespace()  # as defined by <group> in launch file
        else:
            robot_name_from_conf = "mete"
            host_package = "/" + robot_name_from_conf + "/calibration/"

        self.veh = host_package.split('/')[1]
        # initialize the node with rospy
        rospy.init_node('OpticalFlow', anonymous=True)

        # msg to cv bridge
        self.bridge = CvBridge()
        self.prev_cv_image = None
        self.image_id = 0

        self.image_type = "compressed"

        # subscriber
        if self.image_type == "rectified":
            sub_topic_image = '/' + self.veh + '/camera_node/image/rect'
            self.sub_image = rospy.Subscriber(sub_topic_image, Image, self.cbImage, queue_size=1)
        elif self.image_type == "compressed":
            sub_topic_image = '/' + self.veh + '/buffer_node/image/compressed'
            self.sub_image = rospy.Subscriber(sub_topic_image, CompressedImage, self.cbImage, queue_size=1)

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
        if self.image_type == "rectified":
            current_cv_image_or = self.bridge.imgmsg_to_cv2(msg, "rgb8")
        elif self.image_type == "compressed":
            np_arr = np.fromstring(msg.data, np.uint8)
            current_cv_image_or = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        current_cv_image = self.crop_image2(current_cv_image_or, 0, 480, 0, 640) # h_min, h_max, w_min, w_max
        # h = 0 en ust, w = en sol

        if not isinstance(self.prev_cv_image, np.ndarray):
            self.prev_cv_image= current_cv_image
        #rospy.logwarn(type(current_cv_image))
        of = self.calculate_optical_flow(self.prev_cv_image, current_cv_image)
        self.pub_of_val.publish(self.ros_from_np_array(of))
        self.prev_cv_image = current_cv_image

        p = [0.01, 0.01, 0.005, 0.10]
        self.cost_function(p, self.flow_model, current_cv_image, of)
        return of

    def calculate_optical_flow(self, prevImg, currImg):
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
        (h, w) = (prevImg.shape[0], prevImg.shape[1]) # h: 480, w: 640
        #print("height: {} weight: {}".format(h, w))
        a = np.meshgrid(np.arange(0, w, 9), np.arange(0, h, 9)) # from 9 till w increments of 10
        print("a type: {} len a: {}".format(type(a), len(a)))
        print("a0 type: {} shape a0: {}".format(type(a[0]), a[0].shape))
        print(a[0])
        print("a1 type: {} shape a1: {}".format(type(a[1]), a[1].shape))

        prevPoints = np.vstack((a[0].ravel(), a[1].ravel())).transpose().astype(np.float32).copy()
        a_ravel = a[0].ravel()
        print("a_ravel prevPoints: {}".format(a_ravel.shape))
        print("shape prevPoints: {}".format(prevPoints.shape))

        # sparse optical flow estimation e.g. to compute a grid of motion vectors
        start = time.time()
        currPoints = rlofProc.sparse_flow(prevImg, currImg, prevPoints)
        print("currPoints type: {} currPoints shape: {}".format(type(currPoints), currPoints.shape))

        end = time.time()
        self.image_id += 1
        rospy.loginfo("[{}] sparse optical flow for image: {} features: {} runtime[sec]: {}"
                      .format(self.node_name, self.image_id, str(prevPoints.shape[0]), str(end - start)))

        # draw sparse motion vectors
        sparseFlowImg = prevImg.copy()
        for i, (new, old) in enumerate(zip(currPoints, prevPoints)):
            a, b = new.ravel()
            c, d = old.ravel()
            if (a >= 0 and a < prevImg.shape[1] and b >= 0 and b < prevImg.shape[0]):
                sparseFlowImg = cv2.line(sparseFlowImg, (a, b), (c, d), (0, 255, 0), 1)

        # write results
        #cv2.imwrite("/home/selcuk/sparse_flow_{}.png".format(self.image_id), sparseFlowImg)
        cv2.imwrite("/home/selcuk/of_test/lab_ramp_up/comp/sparse_flow_{}.png".format(self.image_id), sparseFlowImg)

        return currPoints

    """
    @staticmethod
    def crop_image(image, h_min, h_max, w_min, w_max):
        # Get size
        H,W,_channel_size = image.shape

        # Compute indices
        h_start = int(np.floor(H * h_min))
        h_end = int(np.ceil(H * h_max))
        w_start = int(np.floor(W * w_min))
        w_end = int(np.ceil(W * w_max))
        #rospy.logwarn("hs: {} he: {} ws: {} we: {}".format(h_start,h_end,w_start,w_end))

        # Crop image
        image_cropped = image[h_start:h_end, w_start:w_end]
        # Return cropped image
        return image_cropped
    """

    @staticmethod
    def crop_image2(image, h_min, h_max, w_min, w_max):
        # Get size
        H,W,_channel_size = image.shape

        #rospy.logwarn("hs: {} he: {} ws: {} we: {}".format(h_start,h_end,w_start,w_end))

        # Crop image
        image_cropped = image[h_min:h_max, w_min:w_max]
        # Return cropped image
        return image_cropped

    @staticmethod
    def flow_model(x, p):
        # x pixels -> N by M np grid
        x_px, y_px = x
        f = 350
        Ty, Tz, wx, Z = p
        u_x = (Tz * x_px) / Z + (wx * x_px * y_px) / f # px/s
        u_y = (Tz * y_px - Ty * f) / Z + (wx * f) + (wx * y_px ** 2) / f # px/s

        return u_x, u_y

    def cost_function(self, p, flow_model, image, F_meas):
        F_model_pred = self.flow_model(image, p)
        err = F_model_pred - F_meas
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
