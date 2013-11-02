cmake_minimum_required(VERSION 2.8.3)
project(hubo_hose_task_planner_msgs)


## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp message_generation genmsg octomap_msgs std_msgs hubo_planning_common hubo_robot_msgs geometry_msgs)

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
file(GLOB message_source "msg/*.msg")
LIST(SORT message_source)
foreach(message_src_file ${message_source})
	get_filename_component(message_base ${message_src_file} NAME)
	message(STATUS "Adding message ${message_base}" )
	add_message_files(DIRECTORY msg FILES ${message_base})
endforeach(message_src_file)


## Generate services in the 'srv' folder
file(GLOB service_source "srv/*.srv")
LIST(SORT service_source)
foreach(service_src_file ${service_source})
	get_filename_component(service_base ${service_src_file} NAME)
	message(STATUS "Adding service ${service_base}" )
	add_service_files(DIRECTORY srv FILES ${service_base})
endforeach(service_src_file)

## Actually generate the language-specific message and service files
generate_messages(DEPENDENCIES std_msgs geometry_msgs octomap_msgs hubo_planning_common hubo_robot_msgs)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   CATKIN_DEPENDS message_generation std_msgs geometry_msgs octomap_msgs hubo_planning_common hubo_robot_msgs)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/groovy/api/catkin/html/adv_user_guide/variables.html

## Mark executables and/or libraries for installation
#install(TARGETS hubo_drc_vision_msgs
#	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#	)

## Mark cpp header files for installation
#install(DIRECTORY include/${PROJECT_NAME}/
#	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#	FILES_MATCHING PATTERN "*.h"
#	PATTERN ".svn" EXCLUDE
#	)
