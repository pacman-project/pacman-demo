###############################################################################
#
#	Prerequisites
#
###############################################################################

# cmake requirements
cmake_minimum_required(VERSION 2.8)

# Build options have to be before PROJECT(...)
SET(CMAKE_BUILD_TYPE "Release" CACHE PATH "Current build configuration")

# PaCMan main workspace
PROJECT(pacman)

# Folders
SET_PROPERTY(GLOBAL PROPERTY USE_FOLDERS ON)

# Project parent directory
GET_FILENAME_COMPONENT(PROJECT_PARENT .. ABSOLUTE CACHE INTERNAL "Path prefix for the project parent")

# Project root directory
GET_FILENAME_COMPONENT(PROJECT_ROOT . ABSOLUTE CACHE INTERNAL "Path prefix for the project")

# Set the path where other thing should be relative to
GET_FILENAME_COMPONENT(CMAKE_SOURCE_DIR . ABSOLUTE CACHE INTERNAL "")

# Set the install directory.
SET(CMAKE_INSTALL_PREFIX /usr/local CACHE PATH "Installation prefix")

# Build options
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -DLINUX -Dlinux -DLINUX64 -D__x86_64__ -DNX64 -Wno-deprecated-declarations -std=c++0x -O3")

# Boost
FIND_PACKAGE(Boost COMPONENTS system filesystem thread)

# OpenCV
FIND_PACKAGE(OpenCV REQUIRED)

# PCL
FIND_PACKAGE(PCL REQUIRED)

ADD_DEFINITIONS(
	${OPENCV_DEFINITIONS}
	${PCL_DEFINITIONS}
)
INCLUDE_DIRECTORIES(
	${PROJECT_ROOT}/include
	${Boost_INCLUDE_DIRS}
	${OPENCV_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
)
LINK_DIRECTORIES(
	${Boost_LIBRARY_DIRS}
	${OPENCV_LIBRARIES}
	${PCL_LIBRARY_DIRS}
)

###############################################################################
#
#	Common PaCMan software
#
###############################################################################

# PaCMan main definitions and inline code
SET(PACMAN_HEADERS
	${PROJECT_ROOT}/include/pacman/PaCMan/Defs.h
)
ADD_LIBRARY(PaCMan STATIC ${PACMAN_HEADERS})
SET_TARGET_PROPERTIES(PaCMan PROPERTIES LINKER_LANGUAGE CXX)
#TARGET_LINK_LIBRARIES(PaCMan)
SET_PROPERTY(TARGET PaCMan PROPERTY PROJECT_LABEL "PaCMan")
INSTALL(FILES ${PACMAN_HEADERS} DESTINATION include/pacman/PaCMan/)

###############################################################################
#
#	Bham software
#
###############################################################################

# Expat
SET(EXPAT_INCLUDE /usr/include CACHE PATH "Path prefix for Expat include")
#MARK_AS_ADVANCED(EXPAT_INCLUDE)
SET(EXPAT_LIBRARY /usr/lib CACHE PATH "Path prefix for Expat library")
#MARK_AS_ADVANCED(EXPAT_LIBRARY)
# Freeglut
SET(FREEGLUT_INCLUDE /usr/include CACHE PATH "Path prefix for Freeglut include")
#MARK_AS_ADVANCED(FREEGLUT_INCLUDE)
SET(FREEGLUT_LIBRARY /usr/lib CACHE PATH "Path prefix for Freeglut library")
#MARK_AS_ADVANCED(FREEGLUT_LIBRARY)
# PhysX
SET(PHYSX_INCLUDE /usr/include/PhysX/v2.8.3/SDKs CACHE PATH "Path prefix for PhysX include")
#MARK_AS_ADVANCED(PHYSX_INCLUDE)
SET(PHYSX_LIBRARY /usr/lib/PhysX/v2.8.3 CACHE PATH "Path prefix for PhysX library")
#MARK_AS_ADVANCED(PHYSX_LIBRARY)
# Golem
SET(GOLEM_INCLUDE /usr/local/include CACHE PATH "Path prefix for Golem include")
#MARK_AS_ADVANCED(GOLEM_INCLUDE)
SET(GOLEM_LIBRARY /usr/local/lib CACHE PATH "Path prefix for Golem library")
#MARK_AS_ADVANCED(GOLEM_LIBRARY)
SET(GOLEM_BINARIES /usr/local/bin CACHE PATH "Path prefix for Golem binaries")
#MARK_AS_ADVANCED(GOLEM_BINARIES)
# Grasp
SET(GRASP_INCLUDE /usr/local/include CACHE PATH "Path prefix for Grasp include")
#MARK_AS_ADVANCED(GRASP_INCLUDE)
SET(GRASP_LIBRARY /usr/local/lib CACHE PATH "Path prefix for Grasp library")
#MARK_AS_ADVANCED(GRASP_LIBRARY)
SET(GRASP_BINARIES /usr/local/bin CACHE PATH "Path prefix for Grasp binaries")
#MARK_AS_ADVANCED(GRASP_BINARIES)
# National Instruments DAQmx
OPTION(BUILD_BHAM_GRASP_FTSENSOR_DAQ "Build NI DAQ F/T sensor device" NO)
MARK_AS_ADVANCED(BUILD_BHAM_GRASP_FTSENSOR_DAQ)
if(BUILD_BHAM_GRASP_FTSENSOR_DAQ)
	SET(NIDAQMX_INCLUDE "/usr/local/natinst/nidaqmxbase/include" CACHE PATH "Path prefix for National Instruments DAQmx include")
	MARK_AS_ADVANCED(NIDAQMX_INCLUDE)
	SET(NIDAQMX_LIBRARY "/usr/local/natinst/nidaqmxbase/lib" CACHE PATH "Path prefix for National Instruments DAQmx library")
	MARK_AS_ADVANCED(NIDAQMX_LIBRARY)
endif(BUILD_BHAM_GRASP_FTSENSOR_DAQ)
# PointGray
OPTION(BUILD_BHAM_GRASP_CAMERA_PGR "Build Point Gray camera device" NO)
MARK_AS_ADVANCED(BUILD_BHAM_GRASP_CAMERA_PGR)
if(BUILD_BHAM_GRASP_CAMERA_PGR)
	SET(POINTGRAY_INCLUDE /usr/include/flycapture CACHE PATH "Path prefix for PointGray include")
	MARK_AS_ADVANCED(POINTGRAY_INCLUDE)
	SET(POINTGRAY_LIBRARY /usr/lib CACHE PATH "Path prefix for PointGray library")
	MARK_AS_ADVANCED(POINTGRAY_LIBRARY)
endif(BUILD_BHAM_GRASP_CAMERA_PGR)

LINK_DIRECTORIES(
	${EXPAT_LIBRARY}
	${FREEGLUT_LIBRARY}
	${PHYSX_LIBRARY}
	${GOLEM_LIBRARY}
	${GOLEM_BINARIES}
	${GRASP_LIBRARY}
	${GRASP_BINARIES}
	${NIDAQMX_LIBRARY}
	${POINTGRAY_LIBRARY}
)
INCLUDE_DIRECTORIES(
	${EXPAT_INCLUDE}
	${FREEGLUT_INCLUDE}
	${PHYSX_INCLUDE}/Physics/include
	${PHYSX_INCLUDE}/Foundation/include
	${PHYSX_INCLUDE}/PhysXLoader/include
	${PHYSX_INCLUDE}/Cooking/include
	${GOLEM_INCLUDE}
	${GRASP_INCLUDE}
	${NIDAQMX_INCLUDE}
	${POINTGRAY_INCLUDE}
)

# Bham grasp library
SET(BHAM_GRASP_HEADERS
	${PROJECT_ROOT}/include/pacman/Bham/Grasp/Grasp.h
	${PROJECT_ROOT}/include/pacman/Bham/Grasp/GraspImpl.h
)
SET(BHAM_GRASP_SOURCES
	${PROJECT_ROOT}/src/pacman/Bham/Grasp/Grasp.cpp
)
SET(BHAM_GRASP_FILES
	${PROJECT_ROOT}/resources/Bham/Grasp.xml
)
ADD_LIBRARY(PaCManBhamGrasp STATIC ${BHAM_GRASP_HEADERS} ${BHAM_GRASP_SOURCES} ${BHAM_GRASP_FILES})
TARGET_LINK_LIBRARIES(
	PaCManBhamGrasp
	GraspGrasp
	${Boost_LIBRARIES}
	GolemDefs GolemMath GolemSys GolemTools GolemCtrl GolemPlan GolemPhys GolemPhysCtrl GolemDeviceSingleCtrl GolemDeviceMultiCtrl
	${OpenCV_LIBS}
	${PCL_LIBRARIES}
	CamcalbCalb
	GL
	GLU
	glut
	NxCharacter NxCooking PhysXCore PhysXLoader
	CamcalbMatas
)
if(BUILD_BHAM_GRASP_FTSENSOR_DAQ)
	TARGET_LINK_LIBRARIES(PaCManBhamGrasp daqft nidaqmxbase)
endif(BUILD_BHAM_GRASP_FTSENSOR_DAQ)
if(BUILD_BHAM_GRASP_CAMERA_PGR)
	TARGET_LINK_LIBRARIES(PaCManBhamGrasp flycapture)
endif(BUILD_BHAM_GRASP_CAMERA_PGR)
SET_PROPERTY(TARGET PaCManBhamGrasp PROPERTY PROJECT_LABEL "Grasp")
SET_PROPERTY(TARGET PaCManBhamGrasp PROPERTY FOLDER "Bham")
SOURCE_GROUP("Resource Files" FILES ${BHAM_GRASP_FILES})
INSTALL(TARGETS PaCManBhamGrasp RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
INSTALL(FILES ${BHAM_GRASP_HEADERS} DESTINATION include/pacman/Bham/Grasp/)
INSTALL(FILES ${BHAM_GRASP_FILES} DESTINATION bin)

# Bham grasp test
SET(BHAM_GRASP_TEST_SOURCES
	${PROJECT_ROOT}/src/pacman/Bham/GraspTest/GraspTest.cpp
)
ADD_EXECUTABLE(PaCManBhamGraspTest ${BHAM_GRASP_TEST_SOURCES})
TARGET_LINK_LIBRARIES(PaCManBhamGraspTest PaCManBhamGrasp)
SET_PROPERTY(TARGET PaCManBhamGraspTest PROPERTY PROJECT_LABEL "GraspTest")
SET_PROPERTY(TARGET PaCManBhamGraspTest PROPERTY FOLDER "Bham")
INSTALL(TARGETS PaCManBhamGraspTest RUNTIME DESTINATION bin)

###############################################################################
#
#	Innsbruck software
#
###############################################################################

# Innsbruck pose estimation library
SET(UIBK_POSE_ESTIMATION_HEADERS
	${PROJECT_ROOT}/include/pacman/UIBK/PoseEstimation/PoseEstimation.h
	${PROJECT_ROOT}/include/pacman/UIBK/PoseEstimation/PoseEstimationImpl.h
)
SET(UIBK_POSE_ESTIMATION_SOURCES
	${PROJECT_ROOT}/src/pacman/UIBK/PoseEstimation/PoseEstimation.cpp
)
SET(UIBK_POSE_ESTIMATION_FILES
	${PROJECT_ROOT}/resources/UIBK/PoseEstimation.txt
)
ADD_LIBRARY(PaCManUIBKPoseEstimation STATIC ${UIBK_POSE_ESTIMATION_HEADERS} ${UIBK_POSE_ESTIMATION_SOURCES} ${UIBK_POSE_ESTIMATION_FILES})
TARGET_LINK_LIBRARIES(
	PaCManUIBKPoseEstimation
	${Boost_LIBRARIES}
	${PCL_LIBRARIES}
)
SET_PROPERTY(TARGET PaCManUIBKPoseEstimation PROPERTY PROJECT_LABEL "PoseEstimation")
SET_PROPERTY(TARGET PaCManUIBKPoseEstimation PROPERTY FOLDER "UIBK")
SOURCE_GROUP("Resource Files" FILES ${UIBK_POSE_ESTIMATION_FILES})
INSTALL(TARGETS PaCManUIBKPoseEstimation RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
INSTALL(FILES ${UIBK_POSE_ESTIMATION_HEADERS} DESTINATION include/pacman/UIBK/PoseEstimation/)
INSTALL(FILES ${UIBK_POSE_ESTIMATION_FILES} DESTINATION bin)

# UIBK pose estimation test
SET(UIBK_POSE_ESTIMATION_TEST_SOURCES
	${PROJECT_ROOT}/src/pacman/UIBK/PoseEstimationTest/PoseEstimationTest.cpp
)
ADD_EXECUTABLE(PaCManUIBKPoseEstimationTest ${UIBK_POSE_ESTIMATION_TEST_SOURCES})
TARGET_LINK_LIBRARIES(PaCManUIBKPoseEstimationTest PaCManUIBKPoseEstimation)
SET_PROPERTY(TARGET PaCManUIBKPoseEstimationTest PROPERTY PROJECT_LABEL "PoseEstimationTest")
SET_PROPERTY(TARGET PaCManUIBKPoseEstimationTest PROPERTY FOLDER "UIBK")
INSTALL(TARGETS PaCManUIBKPoseEstimationTest RUNTIME DESTINATION bin)

###############################################################################
#
#	Pisa software
#
###############################################################################

# TODO

###############################################################################
#
#	ROS
#
###############################################################################

# TODO

