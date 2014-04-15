cmake_minimum_required(VERSION 2.8.3)
project(trajectory_planner_moveit)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED roscpp moveit_msgs sensor_msgs eigen moveit_core geometry_msgs definitions)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

# STILL NEED TO SEE HOW TO PASS THE PACMAN ROOT FOLDER, PERHAPS SETTING AN ENV VAR?
set(PACMAN ../../../../)
set(CMAKE_CXX_FLAGS "-std=c++0x")

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

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
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
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
#  INCLUDE_DIRS include
#  LIBRARIES trajectory_planner_moveit
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
	${PACMAN}/pacman/include
	${catkin_INCLUDE_DIRS}
	/usr/include/eigen3
	)

link_directories(
	${PACMAN}/bin
	)

## Declare a cpp library
# helper classes
add_library(CartPlanner
	src/CartPlanner.cpp
	)
add_library(StatePlanner
	src/StatePlanner.cpp
	)

## Declare a cpp executable
add_executable(trajectory_planner_node 
	src/trajectory_planner_node.cpp
	)
# examples
add_executable(ping_cart_planner 
	examples/ping_cart_planner.cpp
	)
add_executable(ping_cart_planner_and_execution
	examples/ping_cart_planner_and_execution.cpp
	)
add_executable(ping_state_planner
	examples/ping_state_planner.cpp
	)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# libraries
add_dependencies(CartPlanner
	definitions_generate_messages_cpp
	)
add_dependencies(StatePlanner
	definitions_generate_messages_cpp
	)
# executables
add_dependencies(trajectory_planner_node
	CartPlanner
	StatePlanner
	definitions_generate_messages_cpp
	)

#examples
add_dependencies(ping_cart_planner
	definitions_generate_messages_cpp
	)
add_dependencies(ping_cart_planner_and_execution
	definitions_generate_messages_cpp
	)
add_dependencies(ping_state_planner
	definitions_generate_messages_cpp
	)

## Specify libraries to link a library or executable target against
# libraries
target_link_libraries(CartPlanner 
	${catkin_LIBRARIES}
	)
target_link_libraries(StatePlanner 
	${catkin_LIBRARIES}
	)
# executables
target_link_libraries(trajectory_planner_node 
	CartPlanner
	StatePlanner
	${catkin_LIBRARIES}
	)

# examples
target_link_libraries(ping_cart_planner
	${catkin_LIBRARIES}
	)
target_link_libraries(ping_cart_planner_and_execution
	${catkin_LIBRARIES}
	)
target_link_libraries(ping_state_planner
	${catkin_LIBRARIES}
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
# install(TARGETS trajectory_planner_moveit trajectory_planner_moveit_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_trajectory_planner_moveit.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)


## Specify additional locations of header files
## Your package locations should be listed before other locations