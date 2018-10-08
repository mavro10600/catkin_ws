cmake_minimum_required(VERSION 2.8.3)
project(dynamic_tutorials)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  message_generation
  roscpp
  rospy
  rosgraph_msgs
  std_msgs
)


## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   NodeExampleData.msg
#   Message1.msg
#   Message2.msg
 )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
  std_msgs  # Or other packages containing msgs
 )

## Generate dynamic reconfigure parameters in the 'cfg' folder
 generate_dynamic_reconfigure_options(
cfg/Tutorials.cfg
cfg/nodeExample.cfg
cfg/vel_PID.cfg
cfg/joint_PID.cfg
cfg/wrist_PID.cfg
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
 )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} 
  CATKIN_DEPENDS dynamic_reconfigure roscpp rospy message_runtime rosgraph_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
 add_library(talker 
 src/talker.cpp
# src/vel_talker.cpp
# src/joint_talker.cpp
# src/wrist_talker.cpp
 )
 add_library(vel_talker src/vel_talker.cpp)
 add_library(joint_talker src/joint_talker.cpp)
 add_library(wrist_talker src/wrist_talker.cpp)

add_executable(server src/server.cpp)
add_executable(talker_node  src/talker_node.cpp )
add_executable(talker_node_2  src/talker_node_2.cpp )
add_executable(vel_node  src/vel_node.cpp )
add_executable(joint_node  src/joint_node.cpp )


add_dependencies(server dynamic_tutorials_gencfg)
add_dependencies(talker_node dynamic_tutorials_gencfg dynamic_tutorials_generate_messages_cpp)
add_dependencies(talker_node_2 dynamic_tutorials_gencfg dynamic_tutorials_generate_messages_cpp)
add_dependencies(vel_node dynamic_tutorials_gencfg dynamic_tutorials_generate_messages_cpp)
add_dependencies(joint_node dynamic_tutorials_gencfg dynamic_tutorials_generate_messages_cpp)


## Specify libraries to link a library or executable target against
target_link_libraries(talker ${catkin_LIBRARIES})
target_link_libraries(vel_talker ${catkin_LIBRARIES})
target_link_libraries(joint_talker ${catkin_LIBRARIES})
target_link_libraries(wrist_talker ${catkin_LIBRARIES})

target_link_libraries(server ${catkin_LIBRARIES})
target_link_libraries(talker_node ${catkin_LIBRARIES} talker)
target_link_libraries(talker_node_2 ${catkin_LIBRARIES} talker)
target_link_libraries(vel_node ${catkin_LIBRARIES} vel_talker)
target_link_libraries(joint_node ${catkin_LIBRARIES} joint_talker)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_dynamic_tutorials.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
