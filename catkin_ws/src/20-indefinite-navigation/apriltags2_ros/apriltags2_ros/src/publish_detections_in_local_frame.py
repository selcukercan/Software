#!/usr/bin/env python
import rospy
import rosbag
import numpy as np
from threading import Lock
from shutil import copy
from std_msgs.msg import Bool
from apriltags2_ros.msg import AprilTagDetectionArray
from apriltags2_ros.msg import VehiclePoseEuler
from apriltags2_ros_post_process.rotation_utils import *
from apriltags2_ros_post_process.time_sync_utils import *

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

        # Parameters
        # determine we work synchronously or asynchronously, where asynchronous is the default
        # mode of operation. synchronous operation is benefitial when post-processing the recorded
        # experiment data. For example it is beneficial when only compressed image is available from the experiment and we want to
        # pass exach image through a localization pipeline (compressed_image -> decoder -> rectification -> apriltags_detection -> to_local_pose)
        # to extract the pose in world frame
        self.synchronous_mode = rospy.get_param(param_name="/operation_mode")
        self.total_msg_count =  rospy.get_param(param_name="/" + self.veh + "/buffer_node/message_count")
        rospy.logwarn("TOTAL_MSG_COUNT: {}".format(self.total_msg_count))

        # Publisher
        self.pub_topic_image_request = "/" + self.veh + "/" + self.node_name + "/" + "image_requested"
        self.pub_image_request = rospy.Publisher(self.pub_topic_image_request, Bool, queue_size=1)

        self.pub_topic_name = host_package_node + '/tag_detections_local_frame'
        self.pub_detection_in_robot_frame = rospy.Publisher(self.pub_topic_name ,VehiclePoseEuler,queue_size=1)

        # Subscriber
        sub_topic_name =  '/' + self.veh + '/tag_detections'
        self.sub_img = rospy.Subscriber(sub_topic_name, AprilTagDetectionArray, self.cbDetection)


        if self.synchronous_mode:
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
            rospy.logwarn('INVALID MODE OF OPERATION in publish_detections_in_local_frame')

    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def cbDetection(self,msg):
        if (len(msg.detections) > 0):  # non-emtpy detection message
            #print msg
            # unpack the position and orientation returned by apriltags2 ros
            t_msg = msg.detections[0].pose.pose.pose.position
            q_msg = msg.detections[0].pose.pose.pose.orientation

            # convert the message content into a numpy array as robot_pose_in_world_frame requires so.
            t = np.array([t_msg.x, t_msg.y, t_msg.z])
            q = np.array([q_msg.x, q_msg.y, q_msg.z, q_msg.w])

            # express relative rotation of the robot wrt the global frame.
            world_R_veh, world_t_veh = worldTveh(q,t)
            veh_feaXYZ_world = rotation_matrix_to_euler(world_R_veh)

            # convert from numpy float to standart python float to be written into the message
            world_t_veh =  world_t_veh.tolist()
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

            # finally publish the message
            self.pub_detection_in_robot_frame.publish(veh_pose_euler_msg)
            rospy.loginfo('posx: {} posy:  {} rotz: {}'.format(world_t_veh[0],world_t_veh[1],veh_feaXYZ_world[2]))
            if self.synchronous_mode:
                # save the message to the bag file that contains compressed_images
                self.lock.acquire()
                output_rosbag = rosbag.Bag(self.output_bag, 'a') # open bag to write
                output_rosbag.write(self.pub_topic_name, veh_pose_euler_msg)
                output_rosbag.close()
                self.lock.release()

                rospy.loginfo("[{}] wrote image {}".format(self.node_name, self.numb_written_images))
                self.numb_written_images += 1

                # request a new image from "buffer.py"
                req_msg = Bool(True)
                self.pub_image_request.publish(req_msg)

                if self.numb_written_images == self.total_msg_count:
                    time_sync(self.output_bag, self.veh)


        else:
            rospy.loginfo("[{}] empty apriltag detection recieved".format(self.node_name, self.numb_written_images))

            if self.synchronous_mode:
                rospy.loginfo("[{}] in synchronous mode publishing VehiclePoseEuler with entries equal to 0.0".format(self.node_name,self.numb_written_images))

                # form message to publish
                veh_pose_euler_msg = VehiclePoseEuler()
                veh_pose_euler_msg.header.stamp = rospy.Time.now()
                # position
                veh_pose_euler_msg.posx = 0.
                veh_pose_euler_msg.posy = 0.
                veh_pose_euler_msg.posz = 0.
                # orientation
                veh_pose_euler_msg.rotx = 0.
                veh_pose_euler_msg.roty = 0.
                veh_pose_euler_msg.rotz = 0.

                """
                # finally publish the message
                self.pub_detection_in_robot_frame.publish(veh_pose_euler_msg)    
                """

                # save the message to the bag file that contains compressed_images
                self.lock.acquire()
                output_rosbag = rosbag.Bag(self.output_bag, 'a') # open bag to write
                output_rosbag.write(self.pub_topic_name, veh_pose_euler_msg)
                output_rosbag.close()
                self.lock.release()

                rospy.loginfo("[{}] wrote image {}".format(self.node_name, self.numb_written_images))
                self.numb_written_images += 1

                # request a new image from "buffer.py"
                req_msg = Bool(True)
                self.pub_image_request.publish(req_msg)

                if self.numb_written_images == self.total_msg_count:
                    time_sync(self.output_bag, self.veh)

if __name__ == '__main__':
    to_local_pose = ToLocalPose()
    rospy.spin()
