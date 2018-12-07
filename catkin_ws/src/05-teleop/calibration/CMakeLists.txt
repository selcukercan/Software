cmake_minimum_required(VERSION 2.8.3)
project(calibration)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  duckietown_msgs
  std_msgs
  dagu_car
  rosbag_recorder
)

catkin_python_setup()

# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

