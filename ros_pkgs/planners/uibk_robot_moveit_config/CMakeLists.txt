cmake_minimum_required(VERSION 2.8.3)
project(uibk_robot_moveit_config)

find_package(catkin REQUIRED roscpp moveit_msgs sensor_msgs eigen moveit_core geometry_msgs definitions)

catkin_package()

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
	${catkin_INCLUDE_DIRS}
	/usr/include/eigen3
	)


## Declare a cpp executable
add_executable(path_planner_node 
	src/path_planner_node.cpp
	)

add_executable(ping_path_planner 
	src/ping_path_planner.cpp
	)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(path_planner_node definitions_generate_messages_cpp)
add_dependencies(ping_path_planner definitions_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(path_planner_node
  ${catkin_LIBRARIES}
  )

target_link_libraries(ping_path_planner
  ${catkin_LIBRARIES}
  )