cmake_minimum_required(VERSION 2.8.3)
project(bender_ltm_plugins)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)


## Find catkin macros and libraries
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/")
find_package(catkin REQUIRED
    roscpp
    std_srvs
    std_msgs
    uchile_srvs
    uchile_msgs
    geometry_msgs
    sensor_msgs
    message_generation
    cv_bridge
    ltm
    pluginlib
)

# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

# Generate messages in the 'msg' folder
add_message_files(
    FILES
    HumanEntity.msg
)

## Generate services in the 'srv' folder
add_service_files(
    FILES
    HumanEntitySrv.srv
)

# Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
    sensor_msgs
    ltm
)


################################################
## Declare ROS dynamic reconfigure parameters ##
################################################
## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )


###################################
## catkin specific configuration ##
###################################
catkin_package(
    INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include include
    LIBRARIES
    CATKIN_DEPENDS roscpp std_msgs std_srvs uchile_srvs uchile_msgs geometry_msgs sensor_msgs message_runtime cv_bridge ltm
    DEPENDS
)

###########
## Build ##
###########
include_directories(
    ${CATKIN_DEVEL_PREFIX}/include
    include
    ${catkin_INCLUDE_DIRS}
)
link_directories(${catkin_LINK_DIRS})

# ltm_plugins library
add_library(bender_ltm_plugins__ltm_plugins
    src/ltm_plugins.cpp
    src/emotion_plugin.cpp
    src/location_plugin.cpp
    src/human_entity_plugin.cpp
)
add_dependencies(bender_ltm_plugins__ltm_plugins ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(bender_ltm_plugins__ltm_plugins ${catkin_LIBRARIES} ${Boost_LIBRARIES})
