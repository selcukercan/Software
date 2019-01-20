#!/usr/bin/env python
# system imports
import rospy

# package utilities import
from std_msgs.msg import String #Imports msg
from duckietown_msgs.msg import WheelsCmdStamped, Twist2DStamped, AprilTagDetectionArray, VehiclePoseEuler
from calibration.wheel_cmd_utils import *

class TestRun:
    def __init__(self):
        # namespace variables
        host_package = rospy.get_namespace()  # as defined by <group> in launch file
        node_name = 'data_collector'  # node name , as defined in launch file
        host_package_node = host_package + node_name
        veh = host_package.split('/')[1]

        # Initialize the node with rospy
        rospy.init_node('Command', anonymous=True)
        self.rate = rospy.Rate(30) # 30 Hz

        # Publisher
        publisher= '/' + veh + "/joy_mapper_node/car_cmd"
        self.pub_car_cmd = rospy.Publisher(publisher, Twist2DStamped, queue_size=1)

    def exp_name_to_exp_object(self, exp_name):
        """
        accepts a valid experiment name and create the corresponding experiment object

        :param exp_name:
        :return: exp_object if valid experiment name, None otherwise.
        """
        if exp_name == "ramp_up":
            return RampUp()
        elif exp_name == "sine":
            return Sine()
        else:
            rospy.loginfo('[{}] is not a valid experiment name'.format(exp_name))
            return None


    def perform_experiments(self):
        DO_TEST = "yes"

        #available_ref_traj = ["straight"]
        #ui = ExperimentUI()

        while DO_TEST == "yes":
            #rospy.loginfo("\nType the input command sequence you want to generate for testing: {}".format(str(available_experiments)))
            #experiment_type = raw_input()
            #experiment_object = self.exp_name_to_exp_object(experiment_type)
            """
            if experiment_type != None:
                #ramp_up = RampUp()
                default_param_dict = experiment_object.parameter_dict
                param_dict_request = ui.request_param_values(experiment_type, default_param_dict) # get user input to decide the set of params to be used.
                rospy.loginfo("[Data Collector Node] parameter set to be used in {} calibration: {}".format(experiment_type, str(param_dict_request)))

                wheel_cmds = experiment_object.generate_input(param_dict_request)
            """

            self.straight()
                
            rospy.loginfo("\n\nDo you want run another test round? (respond with yes or no)\n\n")
            user_wish = raw_input()

            DO_TEST = user_wish.strip().lower()
        else:
            rospy.loginfo("farewell, done with testing.")

    def send_forward_ref_velocities(self, car_cmd_dict):
        v_x = car_cmd_dict['vx']
        w = car_cmd_dict['w']


        # Put the wheel commands in a message and publish
        msg_car_cmd = Twist2DStamped()
        msg_car_cmd.header.stamp = rospy.Time.now()
        msg_car_cmd.v = v_x
        msg_car_cmd.omega = w

        self.pub_car_cmd.publish(msg_car_cmd)
        rospy.sleep('')

    def straight(self):
        for n in range(0,10):
            self.sendCommand(0.2, 0)
            rospy.sleep(0.5)
        self.sendCommand(0, 0)

    def sendCommand(self, v, w):
        # Put the wheel commands in a message and publish
        msg_car_cmd = Twist2DStamped()
        msg_car_cmd.header.stamp = rospy.Time.now()
        msg_car_cmd.v =v
        msg_car_cmd.omega =w
        self.pub_car_cmd.publish(msg_car_cmd )

if __name__ == '__main__':
    calib = TestRun()
    calib.perform_experiments()

    #os.system("rosnode kill /record")
