#!/usr/bin/env python2

from rosbag_recorder.srv import *
import rospy
import psutil
import subprocess
import signal
from os.path import expanduser

pidDict = {}
veh = None
save_path = None

def recordTopics(req):
	global pidDict

	command = "rosbag record -b 512 -O " + save_path + "/" + req.name
	print "Recording to bag named %s. Topics:"%(req.name)
	for t in req.topics:
		print t
		command += " " + t
	pidDict[req.name] = subprocess.Popen(command, stdin=subprocess.PIPE, shell=True, cwd="/tmp/")

	return RecordTopicsResponse(True)

def stopRecording(req):
	global pidDict

	if req.name in pidDict:
		print "Stop recording to bag named %s"%(req.name)
		p = pidDict[req.name]
		process = psutil.Process(p.pid)
		for subProcess in process.children(recursive=True):
			subProcess.send_signal(signal.SIGINT)
		p.wait()
	else:
		print "No current recording with name %s!"%req.name
	return StopRecordingResponse(True)

def rosbagRecorder():
	global save_path, veh
	veh = rospy.get_param("/rosbag_recorder_server/veh")
	save_path =  rospy.get_param("/rosbag_recorder_server/output_rosbag_dir")
	rospy.loginfo("[rosbagRecorder] i will save results for vehicle [{}] to [{}]".format(veh, save_path))

	rospy.init_node('~rosbag_recorder_server')
	recordServ = rospy.Service('record_topics', RecordTopics, recordTopics)
	stopServ = rospy.Service('stop_recording', StopRecording, stopRecording)
	print "Ready to record topics"
	rospy.spin()

if __name__ == "__main__":
	"""
	# namespace variables
	host_package = rospy.get_namespace()  # as defined by <group> in launch file
	node_name = 'rosbag_recorder_server'  # node name , as defined in launch file
	host_package_node = host_package + node_name
	veh = host_package.split('/')[1]
	param_save_path = host_package_node + "/"  + "output_rosbag_dir"
	"""
	rosbagRecorder()
