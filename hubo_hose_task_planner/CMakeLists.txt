cmake_minimum_required(VERSION 2.8.3)
project(hubo_hose_task_planner)
#set(BUILD_FLAGS "-std=c++0x -Wreturn-type -Wno-enum-compare")

# Add additional CMake files
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs geometry_msgs tf tf_conversions eigen_conversions hubo_hose_task_planner_msgs)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs tf tf_conversions eigen_conversions hubo_hose_task_planner_msgs)

###########
## Build ##
###########

#add_definitions(-DQT_NO_KEYWORDS)

## Specify additional locations of header files
## Your package locations should be listed before other locations
#include_directories(include ${catkin_INCLUDE_DIRS})
#link_libraries(${catkin_LIBRARIES})

# Collect the source, node and test files
# file(GLOB lib_source "src/libraries/*.cpp")
# list(SORT lib_source)
# file(GLOB nodes_source "src/nodes/*.cpp")
# list(SORT nodes_source)
# file(GLOB plugins_source "src/plugins/*.cpp")
# list(SORT plugins_source)
# file(GLOB unit_tests_source "src/tests/unit/*.cpp")
# list(SORT unit_tests_source)
# file(GLOB system_tests_source "src/tests/system/*.cpp")
# list(SORT system_tests_source)
# file(GLOB include_source "include/hubo_manipulation_planner/*.h")
# list(SORT include_source)
# qt4_wrap_cpp(MOC_FILES include/hubo_manipulation_planner/hubo_planning_panel.h)

# ## Declare a cpp library
# add_library(${PROJECT_NAME} SHARED ${lib_source} ${plugins_source} ${include_source} ${MOC_FILES})
# set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS ${BUILD_FLAGS})

## Allow Testing
#enable_testing()

# Build unit tests
# message(STATUS "\n-- UNIT TESTS: ")
# add_custom_target(check COMMAND ${CMAKE_CTEST_COMMAND})
# foreach(utest_src_file ${unit_tests_source})
# 	get_filename_component(test_base ${utest_src_file} NAME_WE)
# 	message(STATUS "Adding test ${test_base}" )
# 	add_executable(${test_base} "src/tests/test_units.cpp" ${utest_src_file})
# 	add_test(${test_base} ${EXECUTABLE_OUTPUT_PATH}/${test_base})
# 	#catkin_add_gtest(${test_base} src/tests/unit/TestParameterizedObject.cpp)
# 	target_link_libraries(${test_base} ${PROJECT_NAME})
# 	set_target_properties(${test_base} PROPERTIES COMPILE_FLAGS ${BUILD_FLAGS})
# 	add_dependencies(tests ${test_base})
# 	add_dependencies(check ${test_base})
# endforeach(utest_src_file)

# Build executables uncomment for c++ files
# message(STATUS "\n-- ROS NODES: ")
# foreach(node_src_file ${nodes_source})
# 	get_filename_component(node_base ${node_src_file} NAME_WE)
# 	message(STATUS "Adding ROS Node ${node_base}" )
# 	add_executable(${node_base} ${node_src_file} ${include_source})
# 	target_link_libraries(${node_base} ${PROJECT_NAME})
# 	add_dependencies(${node_base} ${PROJECT_NAME}_gencpp)
# 	set_target_properties(${node_base} PROPERTIES COMPILE_FLAGS ${BUILD_FLAGS})
# endforeach(node_src_file)

# Build system tests
# message(STATUS "\n-- ROS TESTING NODES: ")
# foreach(test_node_src_file ${system_tests_source})
# 	get_filename_component(test_node_base ${test_node_src_file} NAME_WE)
# 	message(STATUS "Adding ROS Node ${test_node_base}" )
# 	add_executable(${test_node_base} ${test_node_src_file})
# 	target_link_libraries(${test_node_base} ${PROJECT_NAME})
# 	add_dependencies(${test_node_base} ${PROJECT_NAME}_gencpp)
# 	set_target_properties(${test_node_base} PROPERTIES COMPILE_FLAGS ${BUILD_FLAGS})
# endforeach(test_node_src_file)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/groovy/api/catkin/html/adv_user_guide/variables.html

## Mark executables and/or libraries for installation
# install(TARGETS hubo_manipulation_planner
# 	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# 	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# 	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# 	)

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
# 	DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# 	FILES_MATCHING PATTERN "*.h"
# 	PATTERN ".svn" EXCLUDE
# 	)


