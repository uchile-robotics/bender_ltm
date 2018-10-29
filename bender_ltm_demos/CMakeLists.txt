cmake_minimum_required(VERSION 2.8.3)
project(bender_ltm_demos)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  std_srvs
  uchile_msgs
  uchile_srvs
)

catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
catkin_package(
  # INCLUDE_DIRS 
  # LIBRARIES
  # CATKIN_DEPENDS 
  # DEPENDS
)
