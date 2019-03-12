#!/usr/bin/env python
from cv_bridge import CvBridge
from duckietown_msgs.msg import SegmentList, LanePose, BoolStamped, Twist2DStamped, FSMState, WheelsCmdStamped
from duckietown_utils.instantiate_utils import instantiate
import numpy as np
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String
import json

class LaneFilterNode(object):

    def __init__(self):
        host_package = rospy.get_namespace()  # as defined by <group> in launch file
        self.node_name = 'lane_filter_node'  # node name , as defined in launch file
        host_package_node = host_package + self.node_name
        self.veh = host_package.split('/')[1]

        self.active = True
        self.filter = None
        self.updateParams(None)

        self.t_last_update = rospy.get_time()
        self.velocity = Twist2DStamped()

        self.d_median = []
        self.phi_median = []
        self.latencyArray = []

        # Define Constants
        self.curvature_res = self.filter.curvature_res

        # Set parameters to server
        rospy.set_param('~curvature_res', self.curvature_res) #Write to parameter server for transparancy

        self.pub_in_lane    = rospy.Publisher("~in_lane",BoolStamped, queue_size=1)
        # Subscribers
        self.sub = rospy.Subscriber("~segment_list", SegmentList, self.processSegments, queue_size=1)
        self.sub_change_params = rospy.Subscriber("~change_params", String, self.cbChangeParams)

        operation_mode = rospy.get_param("/operation_mode", 0)

        if operation_mode:
            from calibration.model_library import model_generator
            from calibration.utils import cautious_read_param_from_file
            rospy.logwarn("[{}] operating in model-based velocity prediction mode".format(self.node_name))
            # Publisher
            top_wheel_cmd_exec = "/" + self.veh + "/wheels_driver_node/wheels_cmd_executed"
            self.sub_model_velocity = rospy.Subscriber(top_wheel_cmd_exec, WheelsCmdStamped, self.modelBasedVelocityUpdate)

            # construct a model by specifying which model to use
            model_type = "kinematic_drive"
            measurement_coordinate_frame = "polar"

            self.model_object = model_generator(model_type, measurement_coordinate_frame)
            self.model_params = cautious_read_param_from_file(self.veh, self.model_object)
        else:
            self.sub_velocity = rospy.Subscriber("~car_cmd", Twist2DStamped, self.updateVelocity)

        # Publishers
        self.pub_lane_pose = rospy.Publisher("~lane_pose", LanePose, queue_size=1)
        self.pub_belief_img = rospy.Publisher("~belief_img", Image, queue_size=1)
        self.pub_ml_img = rospy.Publisher("~ml_img", Image, queue_size=1)
        self.pub_entropy    = rospy.Publisher("~entropy",Float32, queue_size=1)

        # FSM
        self.sub_switch = rospy.Subscriber("~switch",BoolStamped, self.cbSwitch, queue_size=1)
        self.sub_fsm_mode = rospy.Subscriber("~fsm_mode", FSMState, self.cbMode, queue_size=1)
        self.active = True

        # timer for updating the params
        self.timer = rospy.Timer(rospy.Duration.from_sec(1.0), self.updateParams)

    def modelBasedVelocityUpdate(self, msg):
        # model(self, t, x, u, p) t and x are not required for model prediction and only required for SysId purposes.
        u = (msg.vel_right, msg.vel_left)
        x_dot = self.model_object.model(None, None, u, self.model_params) # returns  [m/s, deg/s]
        self.velocity.v = x_dot[0]
        self.velocity.omega = x_dot[1] * (np.pi/180)

    def cbChangeParams(self, msg):
        data = json.loads(msg.data)
        params = data["params"]
        reset_time = data["time"]
        # Set all paramters which need to be updated
        for param_name in params.keys():
            param_val = params[param_name]
            params[param_name] = eval("self.filter." + str(param_name))
            exec("self.filter." + str(param_name) + "=" + str(param_val))

        # Sleep for reset time
        rospy.sleep(reset_time)

        # Reset parameters to old values
        for param_name in params.keys():
            param_val = params[param_name]
            exec("self.filter." + str(param_name) + "=" + str(param_val))


    def updateParams(self, event):
        if self.filter is None:
            c = rospy.get_param('~filter')
            assert isinstance(c, list) and len(c) == 2, c

            self.loginfo('new filter config: %s' % str(c))
            #rospy.logwarn('c0: {}\nc1: {}'.format(c[0], c[1]))
            #c[0] : lane_filter.LaneFilterHistogram
            #c[1] : parameters ..
            self.filter = instantiate(c[0], c[1])

    def cbSwitch(self, switch_msg):
        self.active = switch_msg.data


    def processSegments(self,segment_list_msg):
        # Get actual timestamp for latency measurement
        timestamp_now = rospy.Time.now()


        if not self.active:
            return

        # Step 0: get values from server
        if (rospy.get_param('~curvature_res') is not self.curvature_res):
            self.curvature_res = rospy.get_param('~curvature_res')
            self.filter.updateRangeArray(self.curvature_res)

        # Step 1: predict
        current_time = rospy.get_time()
        dt = current_time - self.t_last_update

        v = self.velocity.v
        w = self.velocity.omega
        rospy.logwarn('[lane_filter] dt: {} v_ref: {} w_ref: {}'.format(dt, v, w))

        self.filter.predict(dt=dt, v=v, w=w)
        self.t_last_update = current_time

        # Step 2: update

        self.filter.update(segment_list_msg.segments)

        # Step 3: build messages and publish things
        [d_max, phi_max] = self.filter.getEstimate()
        # print "d_max = ", d_max
        # print "phi_max = ", phi_max


        max_val = self.filter.getMax()
        in_lane = max_val > self.filter.min_max
        # build lane pose message to send
        lanePose = LanePose()
        lanePose.header.stamp = segment_list_msg.header.stamp
        lanePose.d = d_max[0]
        lanePose.phi = phi_max[0]
        lanePose.in_lane = in_lane
        # XXX: is it always NORMAL?
        lanePose.status = lanePose.NORMAL


        if self.curvature_res > 0:
            lanePose.curvature = self.filter.getCurvature(d_max[1:], phi_max[1:])



        # publish the belief image
        bridge = CvBridge()
        belief_img = bridge.cv2_to_imgmsg(np.array(255 * self.filter.beliefArray[0]).astype("uint8"), "mono8")
        belief_img.header.stamp = segment_list_msg.header.stamp

        self.pub_lane_pose.publish(lanePose)
        self.pub_belief_img.publish(belief_img)



        # Latency of Estimation including curvature estimation
        estimation_latency_stamp = rospy.Time.now() - timestamp_now
        estimation_latency = estimation_latency_stamp.secs + estimation_latency_stamp.nsecs/1e9
        self.latencyArray.append(estimation_latency)

        if (len(self.latencyArray) >= 20):
            self.latencyArray.pop(0)

        # print "Latency of segment list: ", segment_latency
        # print("Mean latency of Estimation:................. %s" % np.mean(self.latencyArray))

        # TODO (1): see above, method does not exist
        #self.pub_belief_img.publish(belief_img)

        # also publishing a separate Bool for the FSM
        in_lane_msg = BoolStamped()
        in_lane_msg.header.stamp = segment_list_msg.header.stamp
        in_lane_msg.data = True #TODO change with in_lane
        self.pub_in_lane.publish(in_lane_msg)



    def cbMode(self, msg):
        return #TODO adjust self.active

    def updateVelocity(self,twist_msg):
        rospy.logwarn("\n\n\n\n EVER HERE ?\n\n\n\n")
        self.velocity = twist_msg

    def onShutdown(self):
        rospy.loginfo("[LaneFilterNode] Shutdown.")


    def loginfo(self, s):
        rospy.loginfo('[%s] %s' % (self.node_name, s))


if __name__ == '__main__':
    rospy.init_node('lane_filter', anonymous=False)
    lane_filter_node = LaneFilterNode()
    rospy.on_shutdown(lane_filter_node.onShutdown)
    rospy.spin()
