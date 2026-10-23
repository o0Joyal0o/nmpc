cmake_minimum_required(VERSION 2.8.3)
project(nmpc_swarm_demo)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  scripts/central_controller.py
  scripts/moving_obstacles.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY urdf launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
