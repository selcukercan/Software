#!/usr/bin/env python
import rospy
from duckietown_msgs.msg import WheelsCmdStamped, Twist2DStamped, BoolStamped
from duckietown_msgs.srv import SetValueRequest, SetValueResponse, SetValue
from std_srvs.srv import EmptyRequest, EmptyResponse, Empty
from numpy import *
import yaml
import time
import os.path
from duckietown_utils import get_duckiefleet_root
from duckietown_utils.yaml_wrap import yaml_load_file

# Inverse Kinematics Node
# Author: Robert Katzschmann, Shih-Yuan Liu

class InverseKinematicsNode(object):
    def __init__(self):
        # Get node name and vehicle name
        self.node_name = rospy.get_name()
        self.veh_name = self.node_name.split("/")[1]

        # Model Type
        self.model_type = self.load_model()
        rospy.logwarn('Using Model Type: [{}]'.format(self.model_type))

        # Select Model Function
        self.inv_model = self.select_model()
        # Read parameters from the yaml file and write them to ROS parameter server
        self.readParamFromFile()
        # Set local variables (class variables) by reading the values from ROS parameters server
        self.setModelParams()

        self.v_max = 999.0     # TODO: Calculate v_max !
        self.omega_max = 999.0     # TODO: Calculate v_max !

        # Prepare services
        self.setModelServices()

        # Setup the publisher and subscriber
        self.sub_car_cmd = rospy.Subscriber("~car_cmd", Twist2DStamped, self.car_cmd_callback)
        self.sub_actuator_limits_received = rospy.Subscriber("~actuator_limits_received", BoolStamped, self.updateActuatorLimitsReceived, queue_size=1)
        self.pub_wheels_cmd = rospy.Publisher("~wheels_cmd", WheelsCmdStamped, queue_size=1)
        self.pub_actuator_limits = rospy.Publisher("~actuator_limits", Twist2DStamped, queue_size=1)

        self.msg_actuator_limits = Twist2DStamped()
        self.msg_actuator_limits.v = self.v_max     # TODO: Calculate v_max !
        self.msg_actuator_limits.omega = self.omega_max     # TODO: Calculate omega_max !
        self.actuator_limits_received = False
        self.pub_actuator_limits.publish(self.msg_actuator_limits)

        rospy.loginfo("[%s] Initialized.", self.node_name)
        self.printValues()

    def load_model(self):
        # load the model-type as set by the launch file, default value: False
        model_type = rospy.get_param('~model')
        if model_type == False:
            from calibration.utils import get_baseline_config
            conf = yaml_load_file(get_baseline_config(package_name="calibration"))
            return conf["model"]
        else:
            return model_type

    def readParamFromFile(self):
        # Check file existence
        fname = self.getFilePath(self.veh_name)

        # Use default.yaml if file doesn't exsit
        if not os.path.isfile(fname):
            rospy.logwarn("[%s] %s does not exist. Using default.yaml." %(self.node_name,fname))
            fname = self.getFilePath("default")

        with open(fname, 'r') as in_file:
            try:
                yaml_dict = yaml.load(in_file)
            except yaml.YAMLError as exc:
                rospy.logfatal("[%s] YAML syntax error. File: %s fname. Exc: %s" %(self.node_name, fname, exc))
                rospy.signal_shutdown()
                return

        # Set parameters using value in yaml file
        if yaml_dict is None:
            # Empty yaml file
            return

        # select the parameter list that corresponds to the chosen model
        model_param_list = self.getModelParamList()

        for param_name in model_param_list:
            param_value = yaml_dict.get(param_name)
            if param_name is not None:
                rospy.set_param("~"+param_name, param_value)
            else:
                # Skip if not defined, use default value instead.
                pass

    def getModelParamList(self):
        if self.model_type == 'gt':
            return ["gain", "trim", "baseline", "k", "radius", "limit"]
        elif self.model_type == 'kinematic_drive':
            return ["dr", "dl", "L"]
        elif self.model_type == "input_dependent_kinematic_drive":
            return self.model_object.param_ordered_list


    def setModelParams(self):
        # common parameters
        self.limit = self.setup_parameter("~limit", 1.0)
        self.limit_max = 1.0
        self.limit_min = 0.0

        if self.model_type == 'gt':
            # Set local variable by reading parameters
            self.gain = self.setup_parameter("~gain", 0.6)
            self.trim = self.setup_parameter("~trim", 0.0)
            self.baseline = self.setup_parameter("~baseline", 0.1)
            self.radius = self.setup_parameter("~radius", 0.0318)
            self.k = self.setup_parameter("~k", 27.0)
        elif self.model_type == 'kinematic_drive':
            self.dr = self.setup_parameter("~dr", 1)
            self.dl = self.setup_parameter("~dl", 1)
            self.L = self.setup_parameter("~L", 1)
        elif self.model_type == 'input_dependent_kinematic_drive':
            pass
        else:
            rospy.logfatal('Model name {} is not a valid one, failed to set parameters in setModelParams'.format(self.model_type))
            rospy.signal_shutdown()
            return

    def setModelServices(self):
        # Common services
        self.srv_save = rospy.Service("~save_calibration", Empty, self.cbSrvSaveCalibration)

        if self.model_type == 'gt':
            self.srv_set_gain = rospy.Service("~set_gain", SetValue, self.cbSrvSetGain)
            self.srv_set_trim = rospy.Service("~set_trim", SetValue, self.cbSrvSetTrim)
            self.srv_set_baseline = rospy.Service("~set_baseline", SetValue, self.cbSrvSetBaseline)
            self.srv_set_radius = rospy.Service("~set_radius", SetValue, self.cbSrvSetRadius)
            self.srv_set_k = rospy.Service("~set_k", SetValue, self.cbSrvSetK)
            self.srv_set_limit = rospy.Service("~set_limit", SetValue, self.cbSrvSetLimit)
        elif self.model_type == 'kinematic_drive':
            self.srv_set_dr = rospy.Service("~set_dr", SetValue, self.cbSrvSetDR)
            self.srv_set_dl = rospy.Service("~set_dl", SetValue, self.cbSrvSetDL)
            self.srv_set_L = rospy.Service("~set_L", SetValue, self.cbSrvSetL)
        elif self.model_type == "input_dependent_kinematic_drive":
            pass
        else:
            rospy.logfatal('Model name {} is not a valid one, failed to set services in setModelServices'.format(model_type))

    def getFilePath(self, name):
        if self.model_type == 'gt':
            return (get_duckiefleet_root()+'/calibrations/kinematics/' + name + ".yaml")
        elif self.model_type == 'kinematic_drive':
            if name != 'default':
                return (get_duckiefleet_root()+'/calibrations/kinematics/' + name + "_" + self.model_type + ".yaml")
            else:
                rospy.logfatal('\n\nyou must run the calibration script first to generate the model parameters.\n\n')
        elif self.model_type == 'input_dependent_kinematic_drive':
            if name != 'default':
                return (get_duckiefleet_root()+'/calibrations/kinematics/' + name + "_" + self.model_type + ".yaml")
            else:
                rospy.logfatal('\n\nyou must run the calibration script first to generate the model parameters.\n\n')

    def updateActuatorLimitsReceived(self, msg_actuator_limits_received):
        self.actuator_limits_received = msg_actuator_limits_received.data

    def saveCalibration(self):
        # Write to yaml
        data = {
            "calibration_time": time.strftime("%Y-%m-%d-%H-%M-%S"),
            "gain": self.gain,
            "trim": self.trim,
            "baseline": self.baseline,
            "radius": self.radius,
            "k": self.k,
            "limit": self.limit,
        }

        # Write to file
        file_name = self.getFilePath(self.veh_name)
        with open(file_name, 'w') as outfile:
            outfile.write(yaml.dump(data, default_flow_style=False))
        # Printout
        self.printValues()
        rospy.loginfo("[%s] Saved to %s" %(self.node_name, file_name))

    # [Start] Kinematic Drive Model Services
    def cbSrvSetDR(self, req):
        self.dr = req.value
        self.printValues()
        self.msg_actuator_limits.v = self.v_max     # TODO: Calculate v_max !
        self.msg_actuator_limits.omega = self.omega_max     # TODO: Calculate omega_max !
        self.pub_actuator_limits.publish(self.msg_actuator_limits)
        return SetValueResponse()

    def cbSrvSetDL(self, req):
        self.dl = req.value
        self.printValues()
        self.msg_actuator_limits.v = self.v_max     # TODO: Calculate v_max !
        self.msg_actuator_limits.omega = self.omega_max     # TODO: Calculate omega_max !
        self.pub_actuator_limits.publish(self.msg_actuator_limits)
        return SetValueResponse()

    def cbSrvSetL(self, req):
        self.L = req.value
        self.printValues()
        self.msg_actuator_limits.v = self.v_max     # TODO: Calculate v_max !
        self.msg_actuator_limits.omega = self.omega_max     # TODO: Calculate omega_max !
        self.pub_actuator_limits.publish(self.msg_actuator_limits)
        return SetValueResponse()
    # [End] Kinematic Drive Model Services

    # [Start] GT Model Services
    def cbSrvSaveCalibration(self, req):
        self.saveCalibration()
        return EmptyResponse()

    def cbSrvSetGain(self, req):
        self.gain = req.value
        self.printValues()
        self.msg_actuator_limits.v = self.v_max     # TODO: Calculate v_max !
        self.msg_actuator_limits.omega = self.omega_max     # TODO: Calculate omega_max !
        self.pub_actuator_limits.publish(self.msg_actuator_limits)
        return SetValueResponse()

    def cbSrvSetTrim(self, req):
        self.trim = req.value
        self.printValues()
        self.msg_actuator_limits.v = self.v_max     # TODO: Calculate v_max !
        self.msg_actuator_limits.omega = self.omega_max     # TODO: Calculate omega_max !
        self.pub_actuator_limits.publish(self.msg_actuator_limits)
        return SetValueResponse()

    def cbSrvSetBaseline(self, req):
        self.baseline = req.value
        self.printValues()
        self.msg_actuator_limits.v = self.v_max     # TODO: Calculate v_max !
        self.msg_actuator_limits.omega = self.omega_max     # TODO: Calculate omega_max !
        self.pub_actuator_limits.publish(self.msg_actuator_limits)
        return SetValueResponse()

    def cbSrvSetRadius(self, req):
        self.radius = req.value
        self.printValues()
        self.msg_actuator_limits.v = self.v_max     # TODO: Calculate v_max !
        self.msg_actuator_limits.omega = self.omega_max     # TODO: Calculate omega_max !
        self.pub_actuator_limits.publish(self.msg_actuator_limits)
        return SetValueResponse()

    def cbSrvSetK(self, req):
        self.k = req.value
        self.printValues()
        self.msg_actuator_limits.v = self.v_max     # TODO: Calculate v_max !
        self.msg_actuator_limits.omega = self.omega_max     # TODO: Calculate omega_max !
        self.pub_actuator_limits.publish(self.msg_actuator_limits)
        return SetValueResponse()

    def cbSrvSetLimit(self, req):
        self.limit = self.setLimit(req.value)
        self.printValues()
        self.msg_actuator_limits.v = self.v_max     # TODO: Calculate v_max !
        self.msg_actuator_limits.omega = self.omega_max     # TODO: Calculate omega_max !
        self.pub_actuator_limits.publish(self.msg_actuator_limits)
        return SetValueResponse()

    def setLimit(self, value):
        if value > self.limit_max:
            rospy.logwarn("[%s] limit (%s) larger than max at %s" % (self.node_name, value, self.limit_max))
            limit = self.limit_max
        elif value < self.limit_min:
            rospy.logwarn("[%s] limit (%s) smaller than allowable min at %s" % (self.node_name, value, self.limit_min))
            limit = self.limit_min
        else:
            limit = value
        return limit
    # [End] GT Model Services

    def printValues(self):
        if self.model_type == 'gt':
            rospy.loginfo("[%s] gain: %s trim: %s baseline: %s radius: %s k: %s limit: %s" % (self.node_name, self.gain, self.trim, self.baseline, self.radius, self.k, self.limit))
        elif self.model_type == 'kinematic_drive':
            rospy.loginfo("[%s] dr: %s dl: %s L: %s " % (self.node_name, self.dr, self.dl, self.L))

    def car_cmd_callback(self, msg_car_cmd):
        if not self.actuator_limits_received:
            self.pub_actuator_limits.publish(self.msg_actuator_limits)

        v_ref = msg_car_cmd.v
        w_ref = msg_car_cmd.omega

        if self.model_type == "input_dependent_kinematic_drive":
            inputs = self.inv_model(v_ref = v_ref, w_ref= w_ref ,
                                    V_r_init = 0.5, V_l_init = 0.5,
                                    semi_wheel_distance=self.semi_baseline_length)
            u_r = inputs[0]
            u_l = inputs[1]
        else:
            u_r, u_l = self.inv_model(v_ref, w_ref)

        # limiting output to limit, which is 1.0 for the duckiebot
        u_r_limited = max(min(u_r, self.limit), -self.limit)
        u_l_limited = max(min(u_l, self.limit), -self.limit)

        # Put the wheel commands in a message and publish
        msg_wheels_cmd = WheelsCmdStamped()
        msg_wheels_cmd.header.stamp = msg_car_cmd.header.stamp
        msg_wheels_cmd.vel_right = u_r_limited
        msg_wheels_cmd.vel_left = u_l_limited
        self.pub_wheels_cmd.publish(msg_wheels_cmd)

    def gt(self, v_ref, w_ref):
        # assuming same motor constants k for both motors
        k_r = self.k
        k_l = self.k

        # adjusting k by gain and trim
        k_r_inv = (self.gain + self.trim) / k_r
        k_l_inv = (self.gain - self.trim) / k_l

        omega_r = (v_ref + 0.5 * w_ref * self.baseline) / self.radius
        omega_l = (v_ref - 0.5 * w_ref * self.baseline) / self.radius

        # conversion from motor rotation rate to duty cycle
        # u_r = (gain + trim) (v + 0.5 * omega * b) / (r * k_r)
        u_r = omega_r * k_r_inv
        # u_l = (gain - trim) (v - 0.5 * omega * b) / (r * k_l)
        u_l = omega_l * k_l_inv

        return u_r, u_l

    def kinematic_drive(self, v_ref, w_ref):
        u_r = 1.0 / (2 * self.dr) * v_ref + self.L / (2 * self.dr) * w_ref
        u_l = 1.0 / (2 * self.dl) * v_ref - self.L / (2 * self.dl) * w_ref
        return u_r, u_l

    def select_model(self):
        if self.model_type == 'gt':
            return self.gt
        elif self.model_type == 'kinematic_drive':
            return self.kinematic_drive
        elif self.model_type == "input_dependent_kinematic_drive":
            from calibration.model_library import model_generator
            from calibration.utils import get_valid_drive_constants

            self.model_object = model_generator("input_dependent_kinematic_drive")

            duty_cycle_right, drive_constant_right, duty_cycle_left, drive_constant_left, self.semi_baseline_length = \
                get_valid_drive_constants(self.veh_name, self.model_object)

            print duty_cycle_right
            print drive_constant_right
            print duty_cycle_left
            print drive_constant_left
            print self.semi_baseline_length

            # set the interpolation utilities inside the model object
            #self.model_object.linear_interp_drive_constants(duty_cycle_right, drive_constant_right, duty_cycle_left, drive_constant_left)
            self.model_object.fit_to_exponential_model_drive_constants(duty_cycle_right, drive_constant_right, duty_cycle_left,
                                                            drive_constant_left)
            return self.model_object.inverse_model

    def setup_parameter(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        # Write to parameter server for transparency
        rospy.set_param(param_name, value)
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value

if __name__ == '__main__':
    rospy.init_node('inverse_kinematics_node', anonymous=False)
    inverse_kinematics_node = InverseKinematicsNode()
    rospy.spin()
