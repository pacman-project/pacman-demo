# cmake requirements
cmake_minimum_required(VERSION 2.8)

# Example Interface
PROJECT(ExampleInterface)

# Settings
INCLUDE_DIRECTORIES(./include)

# Robot interface
SET(ROBOT_HEADERS ./include/pacman/Robot/Robot.h)

# Robot Kuka
SET(ROBOTKUKA_SOURCES ./src/pacman/RobotKuka/RobotKuka.cpp)
SET(ROBOTKUKA_HEADERS ./include/pacman/RobotKuka/RobotKuka.h)
ADD_LIBRARY(RobotKuka STATIC ${ROBOTKUKA_SOURCES} ${ROBOT_HEADERS} ${ROBOTKUKA_HEADERS})
#TARGET_LINK_LIBRARIES(RobotKuka)
INSTALL(TARGETS RobotKuka RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
INSTALL(FILES ${ROBOT_HEADERS} DESTINATION include/pacman/Robot/)
INSTALL(FILES ${ROBOTKUKA_HEADERS} DESTINATION include/pacman/RobotKuka/)

# Demo
SET(DEMO_SOURCES ./src/pacman/Demo/Demo.cpp)
ADD_EXECUTABLE(Demo ${DEMO_SOURCES})
TARGET_LINK_LIBRARIES(Demo RobotKuka)
INSTALL(TARGETS Demo RUNTIME DESTINATION bin)
