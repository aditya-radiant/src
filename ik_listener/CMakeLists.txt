cmake_minimum_required(VERSION 3.0.2)
project(ik_listener)


find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES ik_listener
  CATKIN_DEPENDS
  rospy
  std_msgs
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

#executable 
install(PROGRAMS
    src/node/ik_listener.py
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)