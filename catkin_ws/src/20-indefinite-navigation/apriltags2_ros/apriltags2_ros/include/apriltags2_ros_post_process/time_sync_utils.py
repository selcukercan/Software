import rosbag
from shutil import copy
import rospy

def time_sync(input_bag, veh_name):

    rospy.loginfo("[{}] started time-sync function".format('time-sync'))

    input_base = input_bag.split('.')[0]
    input_backup_bag = input_base + "_backup.bag"
    output_bag = input_base + "_synced.bag"

    copy(input_bag, input_backup_bag)


    t_compressed = []
    t_tag_detectons = []

    top_compressed_image = "/" + veh_name + "/camera_node/image/compressed"
    top_tag_detections = "/" + veh_name + "/apriltags2_ros/publish_detections_in_local_frame/tag_detections_local_frame"

    # record the ros times of compressed_image and tag_detections topics into a list
    for topic, msg, t in rosbag.Bag(input_bag).read_messages():
        if topic == top_compressed_image:
            t_compressed.append(t)
        elif topic == top_tag_detections:
            t_tag_detectons.append(t)

    # make sure they are in ascending order
    t_compressed.sort()
    t_tag_detectons.sort()

    # t_tag_detections: keys, t_compressed: values
    my_dict = dict(zip(t_tag_detectons, t_compressed))

    # read the values from the input bag and replace the ros time of the tag_detections
    # to match to that of the compressed_image.
    with rosbag.Bag(output_bag, 'w') as outbag:
        for topic, msg, t in rosbag.Bag(input_bag).read_messages():
            if topic == top_tag_detections:
                try:
                    if non_empty_tag_detection(msg):
                        outbag.write(topic, msg, my_dict[t])
                except:
                    pass
            else:
                outbag.write(topic, msg, t)

    rospy.loginfo('finished time-syncing, file is at {}'.format(output_bag))

def non_empty_tag_detection(msg):
    els = [px, py, pz, rx, ry, rz] = [msg.posx, msg.posy, msg.posz, msg.rotx, msg.roty, msg.rotz]
    for el in els:
        if el != 0.:
            return True
    return False
