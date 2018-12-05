#!/usr/bin/env python
import rospy
import rosnode
from std_msgs.msg import String #Imports msg
from duckietown_msgs.msg import WheelsCmdStamped, Twist2DStamped

from calibration.wheel_cmd_utils import *

class Calibration:
    def __init__(self):
        # Initialize the node with rospy
        rospy.init_node('Command', anonymous=True)
        self.frequency = 30
        # Create publisher
        publisher=rospy.get_param("~veh")+"/wheels_driver_node/wheels_cmd"
        self.pub_wheels_cmd = rospy.Publisher(publisher,WheelsCmdStamped,queue_size=1)

        wait_for_rosbag = 5
        rospy.loginfo("[Publish Control] starting the node and waiting {} seconds to ensure rosbag is recording".format(str(wait_for_rosbag)))
        rospy.sleep(wait_for_rosbag)  # wait for the bag to start recording

    def perform_experiments(self):
        DO_EXPERIMENT = "yes"
        available_experiments = ["ramp_up", "sine"]
        ui = ExperimentUI()

        while DO_EXPERIMENT == "yes":
            print "Type the experiment type you want to do: {}".format(str(available_experiments))
            experiment_type = raw_input()

            if experiment_type == "ramp_up":
                ramp_up = RampUp()
                default_param_dict = ramp_up.parameter_dict
                param_dict_request = ui.request_param_values(experiment_type, default_param_dict) # get user input to decide the set of params to be used.
                print "[Calibration INFO] parameter set to be used in {} calibration: {}".format(experiment_type, str(param_dict_request))
                wheel_cmds = ramp_up.generate_input(param_dict_request)
                self.send_commands(wheel_cmds)
            elif experiment_type == "sine":
                sine = Sine()
                default_param_dict = sine.parameter_dict
                param_dict_request = ui.request_param_values(experiment_type, default_param_dict)
                print "[Calibration INFO] parameter set to be used in {} calibration: {}".format(experiment_type, str(param_dict_request))
                wheel_cmds = sine.generate_input(param_dict_request)
                self.send_commands(wheel_cmds)
            else:
                print "NOT A VALID EXPERIMENT OPTION"

            print "\n\nDo you want to do another experiment? [respond with yes or no]\n\n"
            user_wish = raw_input()

            DO_EXPERIMENT = user_wish.strip().lower()
        else:
            print "farewell, no more experiments for now."

    def send_commands(self, wheel_cmds_dict):
        vel_right_array = wheel_cmds_dict['right_wheel']
        vel_left_array = wheel_cmds_dict['left_wheel']

        # Put the wheel commands in a message and publish
        msg_wheels_cmd = WheelsCmdStamped()

        for i in range(len(vel_right_array)):
            msg_wheels_cmd.header.stamp = rospy.Time.now()
            msg_wheels_cmd.vel_right = vel_right_array[i]
            msg_wheels_cmd.vel_left = vel_left_array[i]

            rospy.loginfo("Left Wheel: {} \t Right Wheel: {}".format(vel_left_array[i], vel_right_array[i]))
            self.pub_wheels_cmd.publish(msg_wheels_cmd)
            rospy.sleep(1/self.frequency)

if __name__ == '__main__':
    calib = Calibration()
    calib.perform_experiments()

    #os.system("rosnode kill /record")
