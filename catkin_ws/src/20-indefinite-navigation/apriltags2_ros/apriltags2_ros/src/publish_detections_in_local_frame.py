#!/usr/bin/env python
import rospy
import rosbag
import numpy as np
from std_msgs.msg import Bool
from apriltags2_ros.msg import AprilTagDetectionArray
from duckietown_msgs.msg import VehiclePoseEuler, VehiclePoseEulerArray
from apriltags2_ros_post_process.rotation_utils import *

class ToLocalPose:
    def __init__(self):
        """
        listens to pose estimation returned by apriltag2_ros node and converts it
        into robot pose expressed in the global frame
        """
        host_package = rospy.get_namespace() # as defined by <group> in launch file
        self.node_name = 'publish_detections_in_local_frame' # node name , as defined in launch file
        host_package_node = host_package + self.node_name
        self.veh = host_package.split('/')[1]

        # initialize the node
        rospy.init_node('publish_detections_in_local_frame_node', anonymous=False)
        rospy.sleep(2) # to ensure that the the rosparam service is initialized before the values requested below (an observed issue)


        # Parameters
        # determine we work synchronously or asynchronously, where asynchronous is the default
        # mode of operation. synchronous operation is benefitial when post-processing the recorded
        # experiment data. For example it is beneficial when only compressed image is available from the experiment and we want to
        # pass exach image through a localization pipeline (compressed_image -> decoder -> rectification -> apriltags_detection -> to_local_pose)
        # to extract the pose in world frame
        self.synchronous_mode = rospy.get_param(param_name="/operation_mode")

        # Publisher
        # single tag
        #self.pub_topic_name = host_package_node + '/tag_detections_local_frame'
        #self.pub_detection_in_robot_frame = rospy.Publisher(self.pub_topic_name ,VehiclePoseEuler,queue_size=1)
        # tag array
        self.pub_multiple_tag = host_package_node + '/tag_detections_array_local_frame'
        self.pub_detection_in_robot_frame_array = rospy.Publisher(self.pub_multiple_tag ,VehiclePoseEulerArray,queue_size=1)

        # Subscriber
        sub_topic_name =  '/' + self.veh + '/tag_detections'
        self.sub_img = rospy.Subscriber(sub_topic_name, AprilTagDetectionArray, self.cbDetection)

        self.image_id = 0 # keep the image counter

    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def cbDetection(self, msg):
        veh_pose_euler_array_msg = VehiclePoseEulerArray()
        self.image_id += 1
        if (len(msg.detections) > 0):  # non-emtpy detection message
            veh_pose_euler_array_msg.local_pose_list = []

            for i in range(len(msg.detections)):
                # unpack the position and orientation returned by apriltags2 ros
                t_msg = msg.detections[i].pose.pose.pose.position
                q_msg = msg.detections[i].pose.pose.pose.orientation
                # print msg.detections[i].size
                tag_id_msg = msg.detections[i].id[0]
                tag_size_msg = msg.detections[i].size[0]

                # convert the message content into a numpy array as robot_pose_in_world_frame requires so.
                t = np.array([t_msg.x, t_msg.y, t_msg.z])
                q = np.array([q_msg.x, q_msg.y, q_msg.z, q_msg.w])

                # express relative rotation of the robot wrt the global frame.
                world_R_veh, world_t_veh = vehTworld(q, t)
                veh_feaXYZ_world = rotation_matrix_to_euler(world_R_veh)

                # convert from numpy float to standart python float to be written into the message
                world_t_veh = world_t_veh.tolist()
                veh_feaXYZ_world = veh_feaXYZ_world.tolist()

                # form message to publish
                veh_pose_euler_msg = VehiclePoseEuler()
                veh_pose_euler_msg.header.stamp = rospy.Time.now()
                # position
                veh_pose_euler_msg.posx = world_t_veh[0]
                veh_pose_euler_msg.posy = world_t_veh[1]
                veh_pose_euler_msg.posz = world_t_veh[2]
                # orientation
                veh_pose_euler_msg.rotx = veh_feaXYZ_world[0]
                veh_pose_euler_msg.roty = veh_feaXYZ_world[1]
                veh_pose_euler_msg.rotz = veh_feaXYZ_world[2]
                # size of the Apriltag
                veh_pose_euler_msg.size = tag_size_msg
                # id of the Apriltag
                veh_pose_euler_msg.id = tag_id_msg

                veh_pose_euler_array_msg.local_pose_list.append(veh_pose_euler_msg)

            veh_pose_euler_array_msg.at_detected = True
        else:
            rospy.loginfo("[{}] no apriltags detected in image {}".format(self.node_name, self.image_id))
            veh_pose_euler_array_msg.at_detected = False
        # publish the at array message
        self.pub_detection_in_robot_frame_array.publish(veh_pose_euler_array_msg)


if __name__ == '__main__':
    to_local_pose = ToLocalPose()
    rospy.spin()
