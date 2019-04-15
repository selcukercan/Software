#!/usr/bin/env python
import unittest, rosunit, rospy
from calibration.data_preperation_utils import DataPreparation
import rospy, rosbag
import numpy as np
from os.path import join

class DataPreperationUtils(unittest.TestCase):
    def setup(self):
        veh_name = "testbot"
        node_name = "test_data_preperation_utils"

        robot_name = rospy.get_param('/' + join(veh_name, node_name, "veh"))
        path = rospy.get_param('/' + join(veh_name, node_name, "path"))

        input_bag = path + robot_name + "_calibration.bag"

        # topics of interest
        top_wheel_cmd_exec = "/" + "mete" + "/wheels_driver_node/wheels_cmd_executed"
        top_robot_pose = "/" + "mete" + "/apriltags2_ros/publish_detections_in_local_frame/tag_detections_local_frame"

        self.data_raw = DataPreparation(input_bag=input_bag, top_wheel_cmd_exec=top_wheel_cmd_exec, top_robot_pose=top_robot_pose)
        self.data_raw.TEST_MODE = True

    def test_resampling_output_dimension(self):
        """
        tests whether resampled left and right wheel commands have the same dimension as robot pose measurements.
        """
        print 'INSIDE TEST 1'
        self.setup()
        do = self.data_raw
        wheel_cmd_rs = do.resampling(do.wheel_cmd, do.robot_pose )

        self.assertEqual(len(do.robot_pose['px']), len(wheel_cmd_rs['vel_r']))


if __name__ == '__main__':
    rosunit.unitrun('calibration', 'test_data_preperation_utils', DataPreperationUtils)
