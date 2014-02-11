cmake_minimum_required(VERSION 2.8.3)
project(pose_estimation_uibk)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED roscpp genmsg definitions)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS filesystem system)
find_package(Eigen REQUIRED)
find_package(PCL 1.7 REQUIRED)

set(POSE_ESTIMATION_EXT /home/pacman/Code/pacman/poseEstimation)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   msg.srv
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#  DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
#   sensor_msgs
#   geometry_msgs
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#   INCLUDE_DIRS include
#  LIBRARIES lwr_test
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${PCL_INCLUDE_DIRS}

  #/home/krzysztof/Dokumenty/Birmingham_research/pacman_poseEstimation/include
   ${POSE_ESTIMATION_EXT}/include




)

add_definitions(
  ${PCL_DEFINITIONS}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
 # /home/crosales/Code/poseEstimation/build
  ${POSE_ESTIMATION_EXT}/build
)

## Declare a cpp library
#add_library(pose_estimation
#   src/ParametersPoseEstimation.cpp 
#   src/I_SegmentedObjects.cpp
#)

## Declare a cpp executable
add_executable(pose_estimation_node src/pose_estimation_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(pose_estimation_node definitions_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(pose_estimation_node
   ${catkin_LIBRARIES} ${boost_LIBRARIES} ${PCL_LIBRARIES} pose_estimation  pcl_3d_rec_framework
)


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
# install(TARGETS lwr_test lwr_test_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_lwr_test.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
