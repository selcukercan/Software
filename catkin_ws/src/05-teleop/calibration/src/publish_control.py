#!/usr/bin/env python
import rospy
import rosnode
from std_msgs.msg import String #Imports msg
from duckietown_msgs.msg import WheelsCmdStamped, Twist2DStamped
from math import sin, pi, cos
import os
from calibration.wheel_cmd_utils import *

class Calibration:
    def __init__(self):
        # Initialize the node with rospy
        rospy.init_node('Command', anonymous=True)
        self.frequency = 30
        # Create publisher
        publisher=rospy.get_param("~veh")+"/wheels_driver_node/wheels_cmd"
        self.pub_wheels_cmd = rospy.Publisher(publisher,WheelsCmdStamped,queue_size=1)
        DO_EXPERIMENT = "yes"

        while DO_EXPERIMENT == "yes":
            print "Please, enter experiment name: "
            experiment_type = raw_input()

            print "Do you want to make another experiment? [please respond with yes or no]"
            user_wish = raw_input()
            DO_EXPERIMENT = user_wish.strip().lower()
            print "See [{}]".format(DO_EXPERIMENT)
        else:
            
            print "thanks, no more experiments for now."


    def sendCommand(self, vel_right, vel_left):
        # Put the wheel commands in a message and publish
        msg_wheels_cmd = WheelsCmdStamped()
        msg_wheels_cmd.header.stamp = rospy.Time.now()
        msg_wheels_cmd.vel_right =vel_right
        msg_wheels_cmd.vel_left =vel_left
        self.pub_wheels_cmd.publish(msg_wheels_cmd)

        rospy.sleep(1/self.frequency)

if __name__ == '__main__':
    rospy.sleep(5) #wait for the bag to start recording
    calib=Calibration()
    #os.system("rosnode kill /record")
