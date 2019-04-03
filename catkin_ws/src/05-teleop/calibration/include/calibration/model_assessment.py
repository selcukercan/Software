"""
A library to faciliate model assessment procedure. It includes assessment rules (or logic)
"""

import rospy
from calibration.utils import get_param_from_config_file

use_assessment = get_param_from_config_file("assessment_rule")

def nsap_threshold_based(nsap_error):
    nsap_threshold = get_param_from_config_file("nsap_threshold")
    if nsap_error > nsap_threshold:
        return "FAIL"
    return "PASS"

def end_point_prediction(long_error, heading_error):
    longitudinal_distance_threshold = get_param_from_config_file("longitudinal_distance_threshold")
    heading_angle_error_threshold = get_param_from_config_file("heading_angle_error_threshold")

    if (long_error > longitudinal_distance_threshold) or (heading_error > heading_angle_error_threshold):
        return "FAIL"
    return "PASS"

def assesment_rule():
    if use_assessment == "nsap_threshold":
        return nsap_threshold_based
    elif use_assessment == "end_positon_prediction":
        return end_point_prediction
    else:
        rospy.logfatal("[{}] {} is not a valid model assessment method".format("model_assessment", use_assessment))
