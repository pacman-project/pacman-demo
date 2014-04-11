cmake_minimum_required(VERSION 2.8.3)
project(uibk_robot_moveit_config)

find_package(catkin REQUIRED roscpp moveit_msgs sensor_msgs eigen moveit_core geometry_msgs definitions)

catkin_package()

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
