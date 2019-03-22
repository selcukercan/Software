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

def assesment_rule():
    if use_assessment == "nsap_threshold":
        return nsap_threshold_based
    else:
        rospy.logfatal("[{}] {} is not a valid model assessment method".format("model_assessment", use_assessment))
