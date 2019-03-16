import rosbag
import rospy

from shutil import copy
from os.path import splitext, isfile
from os import remove

def time_sync(input_bag, veh_name):
    """
    performs time-syncing of the compressed images and the localization estimates of different methods, currently
    AprilTag Algorithm and Lane Filter Algorithm are supported.

    remarks:
    * this procedure is required since rosbag write uses the clock time.
    """

    rospy.loginfo("[{}] started time-sync function".format('time-sync'))

    # back up and output rosbag names
    input_base = splitext(input_bag)[0]
    input_backup_bag = input_base + "_backup.bag"
    output_bag = input_base + "_pp.bag"

    # create a backup of the original file
    copy(input_bag, input_backup_bag)

    # topics that we would like to time-sync
    top_compressed_image = "/" + veh_name + "/camera_node/image/compressed"
    top_tag_detections = "/" + veh_name + "/apriltags2_ros/publish_detections_in_local_frame/tag_detections_array_local_frame"
    top_lane_filter = "/" + veh_name + "/lane_filter_node/lane_pose"

    # create lists to store the timestamps of compressed images and the localization estimates.
    t_compressed = []
    t_at = []
    t_lf = []

    # record the ros times of compressed_image and tag_detections topics into a list.
    for topic, msg, t in rosbag.Bag(input_bag).read_messages():
        if topic == top_compressed_image:
            t_compressed.append(t)
        elif topic == top_tag_detections:
            t_at.append(t)
        elif topic == top_lane_filter:
            t_lf.append(t)

    # make sure they are in ascending order. (note: this might not be necessary)
    t_compressed.sort()
    t_at.sort()
    t_lf.sort()

    # t_tag_detections: keys, t_compressed: values
    map_comp_at = dict(zip(t_at, t_compressed))
    map_comp_lf = dict(zip(t_lf, t_compressed))


    # read the values from the input bag and replace the ros time of the tag_detections
    # to match to that of the compressed_image. Note the output is written only when an
    # apriltag is detected.
    with rosbag.Bag(output_bag, 'w') as outbag:
        for topic, msg, t in rosbag.Bag(input_bag).read_messages():
            if topic == top_tag_detections:
                try:
                    if msg.at_detected:
                        outbag.write(topic, msg, map_comp_at[t])
                        rospy.logwarn("[time_sync] FULL april tag detection array recieved")
                    else:
                        rospy.logwarn("[time_sync] empty april tag detection array recieved")
                except:
                    pass
            elif topic == top_lane_filter:
                outbag.write(topic, msg, map_comp_lf[t])
            else:
                outbag.write(topic, msg, t)

    # Finally if the output bag is successfully generated, cleanup by removing the input bag and its backup.
    if isfile(output_bag):
        remove(input_bag)
        remove(input_backup_bag)

    rospy.loginfo('finished time-syncing, file is at {}'.format(output_bag))


'''
def non_empty_tag_detection(msg):
    """ write to bag if there exist at least one non-empty tag-detection"""
    tag_list = msg.local_pose_list
    if len(tag_list ) != 0:
        for tag_i in range(len(tag_list)):
            if tag_list[tag_i].at_detected:
                return True
            return False
    else:
        rospy.logwarn("[time_sync] empty april tag detection array recieved")

'''

'''
# is used when VehiclePoseEuler is published, not VehiclePoseEulerArray. Note 
def non_empty_tag_detection_single_tag(msg):
    """ write to bag if there exist at least one non-empty tag-detection"""
    els = [px, py, pz, rx, ry, rz] = [msg.posx, msg.posy, msg.posz, msg.rotx, msg.roty, msg.rotz]
    for el in els:
        if el != 0.:
            return True
    return False
'''