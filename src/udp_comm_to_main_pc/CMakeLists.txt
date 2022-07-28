cmake_minimum_required(VERSION 2.8.3)
project(udp_comm_to_main_pc)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
)

# catkin_python_setup()

catkin_package()