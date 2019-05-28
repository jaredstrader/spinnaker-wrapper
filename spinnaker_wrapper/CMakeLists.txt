cmake_minimum_required(VERSION 2.8.7)

project(spinnaker_wrapper)

find_package(catkin REQUIRED COMPONENTS
	message_generation
	message_runtime
	roscpp
	rospy
	roslib
	std_msgs
	geometry_msgs
	cv_bridge
	image_transport
)

find_package(OpenCV REQUIRED)

# Generate messages in the 'msg' folder
# add_message_files(
#   FILES
# )

## Generate services in the 'srv' folder
# add_service_files(
# 	FILES
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
	DEPENDENCIES
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
	INCLUDE_DIRS include
	CATKIN_DEPENDS message_generation message_runtime roscpp rospy std_msgs geometry_msgs
)

add_definitions("-std=gnu++11")

###########
## Build ##
###########

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	/usr/include/spinnaker
)

## Add library
add_library(spinnaker_wrapper src/spinnaker_wrapper.cpp)
target_link_libraries(spinnaker_wrapper ${catkin_LIBRARIES}
	Spinnaker
	Spinnaker_C
)

## Declare a C++ executable
add_executable(acquisition_node src/acquisition_node.cpp)
add_dependencies(acquisition_node spinnaker_wrapper_generate_messages_cpp)
target_link_libraries(acquisition_node 
	${catkin_LIBRARIES}
	${OpenCV_LIBS}
	Spinnaker
	Spinnaker_C
	spinnaker_wrapper
)

