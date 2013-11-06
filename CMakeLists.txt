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

# Traget names
SET(CMAKE_RELEASE_POSTFIX "" CACHE PATH "Release postfix")
mark_as_advanced(CMAKE_RELEASE_POSTFIX)
SET(CMAKE_DEBUG_POSTFIX "DEBUG" CACHE PATH "Debug postfix")
mark_as_advanced(CMAKE_DEBUG_POSTFIX)

# Executable output directory
SET(RUNTIME_OUTPUT_DIRECTORY ${PROJECT_PARENT}/bin CACHE PATH "Executable output directory")
mark_as_advanced(RUNTIME_OUTPUT_DIRECTORY)

# Dynamic library output directory
SET(LIBRARY_OUTPUT_DIRECTORY ${PROJECT_PARENT}/bin CACHE PATH "Dynamic library output directory")
mark_as_advanced(LIBRARY_OUTPUT_DIRECTORY)

# Static library output directory
SET(ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_PARENT}/lib CACHE PATH "Static library output directory")
mark_as_advanced(ARCHIVE_OUTPUT_DIRECTORY)

SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${RUNTIME_OUTPUT_DIRECTORY})
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${LIBRARY_OUTPUT_DIRECTORY})
SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${ARCHIVE_OUTPUT_DIRECTORY})
foreach(CONFIGURATION_TYPE ${CMAKE_CONFIGURATION_TYPES})
	string(TOUPPER ${CONFIGURATION_TYPE} CONFIGURATION_TYPE)
	SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${CONFIGURATION_TYPE} ${RUNTIME_OUTPUT_DIRECTORY})
	SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY_${CONFIGURATION_TYPE} ${LIBRARY_OUTPUT_DIRECTORY})
	SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${CONFIGURATION_TYPE} ${ARCHIVE_OUTPUT_DIRECTORY})
endforeach(CONFIGURATION_TYPE CMAKE_CONFIGURATION_TYPES)

option(POST_BUILD_COPY_FILES "Copy resource files after build" YES)
mark_as_advanced(POST_BUILD_COPY_FILES)

MACRO(COPY_FILES TAR DST)
	if(POST_BUILD_COPY_FILES)
		foreach(SRC ${ARGN})
			add_custom_command(TARGET ${TAR} POST_BUILD COMMAND ${CMAKE_COMMAND} -DSRC="${SRC}" -DDST="${DST}" -P "${CMAKE_SOURCE_DIR}/copy.cmake")
		endforeach(SRC)
	endif(POST_BUILD_COPY_FILES)
ENDMACRO(COPY_FILES TAR DST)

# Build options
if (WIN32)
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  /D_CRT_SECURE_NO_DEPRECATE /Ox")
elseif (UNIX)
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -DLINUX -Dlinux -DLINUX64 -D__x86_64__ -DNX64 -Wno-deprecated-declarations -std=c++0x -O3")
endif()

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
	${PROJECT_ROOT}/include/pacman/PaCMan/PCL.h
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

if (WIN32)
	SET(PROGRAM_FILES $ENV{ProgramW6432})
	SET(X86_WIN "win64")
	SET(X86_X64 "64")
	
	if (MSVC90)
		SET(MSVC_VER "90")
	elseif(MSVC10)
		SET(MSVC_VER "100")
	elseif(MSVC11)
		SET(MSVC_VER "110")
	endif()
	
	# Expat
	SET(EXPAT_INCLUDE "${PROGRAM_FILES}/expat/include" CACHE PATH "Path prefix for Expat include")
	#MARK_AS_ADVANCED(EXPAT_INCLUDE)
	SET(EXPAT_LIBRARY "${PROGRAM_FILES}/expat/lib" CACHE PATH "Path prefix for Expat library")
	#MARK_AS_ADVANCED(EXPAT_LIBRARY)
	# Freeglut
	SET(FREEGLUT_INCLUDE "${PROGRAM_FILES}/freeglut/include" CACHE PATH "Path prefix for Freeglut include")
	#MARK_AS_ADVANCED(FREEGLUT_INCLUDE)
	SET(FREEGLUT_LIBRARY "${PROGRAM_FILES}/freeglut/lib" CACHE PATH "Path prefix for Freeglut library")
	#MARK_AS_ADVANCED(FREEGLUT_LIBRARY)
	# PhysX
	SET(PHYSX_INCLUDE "${PROGRAM_FILES}/NVIDIA Corporation/NVIDIA PhysX SDK/v2.8.4_win/SDKs" CACHE PATH "Path prefix for PhysX include")
	#MARK_AS_ADVANCED(PHYSX_INCLUDE)
	SET(PHYSX_LIBRARY "${PROGRAM_FILES}/NVIDIA Corporation/NVIDIA PhysX SDK/v2.8.4_win/SDKs/lib/${X86_WIN}" CACHE PATH "Path prefix for PhysX library")
	#MARK_AS_ADVANCED(PHYSX_LIBRARY)
	# Golem
	SET(GOLEM_INCLUDE "${PROGRAM_FILES}/Golem/include" CACHE PATH "Path prefix for Golem include")
	#MARK_AS_ADVANCED(GOLEM_INCLUDE)
	SET(GOLEM_LIBRARY "${PROGRAM_FILES}/Golem/lib" CACHE PATH "Path prefix for Golem library")
	#MARK_AS_ADVANCED(GOLEM_LIBRARY)
	SET(GOLEM_BINARIES "${PROGRAM_FILES}/Golem/bin" CACHE PATH "Path prefix for Golem binaries")
	#MARK_AS_ADVANCED(GOLEM_BINARIES)
	# Grasp
	SET(GRASP_INCLUDE "${PROGRAM_FILES}/Grasp/include" CACHE PATH "Path prefix for Grasp include")
	#MARK_AS_ADVANCED(GRASP_INCLUDE)
	SET(GRASP_LIBRARY "${PROGRAM_FILES}/Grasp/lib" CACHE PATH "Path prefix for Grasp library")
	#MARK_AS_ADVANCED(GRASP_LIBRARY)
	SET(GRASP_BINARIES "${PROGRAM_FILES}/Grasp/bin" CACHE PATH "Path prefix for Grasp binaries")
	#MARK_AS_ADVANCED(GRASP_BINARIES)
	# National Instruments DAQmx
	OPTION(BUILD_BHAM_GRASP_FTSENSOR_DAQ "Build NI DAQ F/T sensor device" YES)
	MARK_AS_ADVANCED(BUILD_BHAM_GRASP_FTSENSOR_DAQ)
	if(BUILD_BHAM_GRASP_FTSENSOR_DAQ)
	SET(NIDAQMX_INCLUDE "${PROGRAM_FILES}/National Instruments/Shared/ExternalCompilerSupport/C/include" CACHE PATH "Path prefix for National Instruments DAQmx include")
		MARK_AS_ADVANCED(NIDAQMX_INCLUDE)
		SET(NIDAQMX_LIBRARY "${PROGRAM_FILES}/National Instruments/Shared/ExternalCompilerSupport/C/lib${X86_X64}/msvc" CACHE PATH "Path prefix for National Instruments DAQmx library")
		MARK_AS_ADVANCED(NIDAQMX_LIBRARY)
	endif(BUILD_BHAM_GRASP_FTSENSOR_DAQ)
	# PointGray
	OPTION(BUILD_BHAM_GRASP_CAMERA_PGR "Build Point Gray camera device" YES)
	MARK_AS_ADVANCED(BUILD_BHAM_GRASP_CAMERA_PGR)
	if(BUILD_BHAM_GRASP_CAMERA_PGR)
		SET(POINTGRAY_INCLUDE "${PROGRAM_FILES}/Point Grey Research/FlyCapture2/include" CACHE PATH "Path prefix for PointGray include")
		MARK_AS_ADVANCED(POINTGRAY_INCLUDE)
		SET(POINTGRAY_LIBRARY "${PROGRAM_FILES}/Point Grey Research/FlyCapture2/lib${X86_X64}" CACHE PATH "Path prefix for PointGray library")
		MARK_AS_ADVANCED(POINTGRAY_LIBRARY)
	endif(BUILD_BHAM_GRASP_CAMERA_PGR)
elseif (UNIX)
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
endif()

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
if (WIN32)
	TARGET_LINK_LIBRARIES(
		PaCManBhamGrasp
		optimized GraspGrasp${CMAKE_RELEASE_POSTFIX} debug GraspGrasp${CMAKE_DEBUG_POSTFIX}
		optimized GraspApp${CMAKE_RELEASE_POSTFIX} debug GraspApp${CMAKE_DEBUG_POSTFIX}		 
		NxCharacter64 PhysXCooking64 PhysXCore64 PhysXLoader64
		expat freeglut Gdiplus
		optimized GolemDefs${CMAKE_RELEASE_POSTFIX} debug GolemDefs${CMAKE_DEBUG_POSTFIX}
		optimized GolemMath${CMAKE_RELEASE_POSTFIX} debug GolemMath${CMAKE_DEBUG_POSTFIX}
		optimized GolemSys${CMAKE_RELEASE_POSTFIX} debug GolemSys${CMAKE_DEBUG_POSTFIX}
		optimized GolemTools${CMAKE_RELEASE_POSTFIX} debug GolemTools${CMAKE_DEBUG_POSTFIX}
		optimized GolemCtrl${CMAKE_RELEASE_POSTFIX} debug GolemCtrl${CMAKE_DEBUG_POSTFIX}
		optimized GolemPlan${CMAKE_RELEASE_POSTFIX} debug GolemPlan${CMAKE_DEBUG_POSTFIX}
		optimized GolemPhys${CMAKE_RELEASE_POSTFIX} debug GolemPhys${CMAKE_DEBUG_POSTFIX}
		optimized GolemPhysCtrl${CMAKE_RELEASE_POSTFIX} debug GolemPhysCtrl${CMAKE_DEBUG_POSTFIX}
		optimized GolemDeviceSingleCtrl${CMAKE_RELEASE_POSTFIX} debug GolemDeviceSingleCtrl${CMAKE_DEBUG_POSTFIX}
		optimized GolemDeviceMultiCtrl${CMAKE_RELEASE_POSTFIX} debug GolemDeviceMultiCtrl${CMAKE_DEBUG_POSTFIX}
		optimized GolemDeviceRobotJustin${CMAKE_RELEASE_POSTFIX} debug GolemDeviceRobotJustin${CMAKE_DEBUG_POSTFIX} 
		optimized CamcalbCalb${CMAKE_RELEASE_POSTFIX} debug CamcalbCalb${CMAKE_DEBUG_POSTFIX}
		optimized CamcalbMatas${CMAKE_RELEASE_POSTFIX} debug CamcalbMatas${CMAKE_DEBUG_POSTFIX}		 
		${PCL_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS}
	)
	if(BUILD_BHAM_GRASP_FTSENSOR_DAQ)
		TARGET_LINK_LIBRARIES(
			PaCManBhamGrasp
			optimized daqft${CMAKE_RELEASE_POSTFIX} debug daqft${CMAKE_DEBUG_POSTFIX}		 
			NIDAQmx
		)
	endif(BUILD_BHAM_GRASP_FTSENSOR_DAQ)
	if(BUILD_BHAM_GRASP_CAMERA_PGR)
		TARGET_LINK_LIBRARIES(PaCManBhamGrasp FlyCapture2_v${MSVC_VER})
	endif(BUILD_BHAM_GRASP_CAMERA_PGR)
elseif (UNIX)
	TARGET_LINK_LIBRARIES(
		PaCManBhamGrasp
		${PCL_LIBRARIES}
		optimized CamcalbCalb${CMAKE_RELEASE_POSTFIX} debug CamcalbCalb${CMAKE_DEBUG_POSTFIX}
		optimized CamcalbMatas${CMAKE_RELEASE_POSTFIX} debug CamcalbMatas${CMAKE_DEBUG_POSTFIX}		 
		${OpenCV_LIBS}
		optimized GraspGrasp${CMAKE_RELEASE_POSTFIX} debug GraspGrasp${CMAKE_DEBUG_POSTFIX}
		optimized GraspApp${CMAKE_RELEASE_POSTFIX} debug GraspApp${CMAKE_DEBUG_POSTFIX}		 
		${Boost_LIBRARIES}
		optimized GolemDefs${CMAKE_RELEASE_POSTFIX} debug GolemDefs${CMAKE_DEBUG_POSTFIX}
		optimized GolemSys${CMAKE_RELEASE_POSTFIX} debug GolemSys${CMAKE_DEBUG_POSTFIX}
		optimized GolemTools${CMAKE_RELEASE_POSTFIX} debug GolemTools${CMAKE_DEBUG_POSTFIX}
		optimized GolemCtrl${CMAKE_RELEASE_POSTFIX} debug GolemCtrl${CMAKE_DEBUG_POSTFIX}
		optimized GolemPlan${CMAKE_RELEASE_POSTFIX} debug GolemPlan${CMAKE_DEBUG_POSTFIX}
		optimized GolemPhysCtrl${CMAKE_RELEASE_POSTFIX} debug GolemPhysCtrl${CMAKE_DEBUG_POSTFIX}
		optimized GolemDeviceSingleCtrl${CMAKE_RELEASE_POSTFIX} debug GolemDeviceSingleCtrl${CMAKE_DEBUG_POSTFIX}
		optimized GolemDeviceMultiCtrl${CMAKE_RELEASE_POSTFIX} debug GolemDeviceMultiCtrl${CMAKE_DEBUG_POSTFIX}
		optimized GolemDeviceRobotJustin${CMAKE_RELEASE_POSTFIX} debug GolemDeviceRobotJustin${CMAKE_DEBUG_POSTFIX}
		${CMAKE_DL_LIBS}
		optimized GolemMath${CMAKE_RELEASE_POSTFIX} debug GolemMath${CMAKE_DEBUG_POSTFIX}
		optimized GolemPhys${CMAKE_RELEASE_POSTFIX} debug GolemPhys${CMAKE_DEBUG_POSTFIX}
		GL
		GLU
		glut
		NxCharacter NxCooking PhysXCore PhysXLoader
	)
	if(BUILD_BHAM_GRASP_FTSENSOR_DAQ)
		TARGET_LINK_LIBRARIES(
			PaCManBhamGrasp
			optimized daqft${CMAKE_RELEASE_POSTFIX} debug daqft${CMAKE_DEBUG_POSTFIX}		 
			nidaqmxbase
		)
	endif(BUILD_BHAM_GRASP_FTSENSOR_DAQ)
	if(BUILD_BHAM_GRASP_CAMERA_PGR)
		TARGET_LINK_LIBRARIES(PaCManBhamGrasp flycapture)
	endif(BUILD_BHAM_GRASP_CAMERA_PGR)
endif()	
SET_PROPERTY(TARGET PaCManBhamGrasp PROPERTY RELEASE_POSTFIX ${CMAKE_RELEASE_POSTFIX})
SET_PROPERTY(TARGET PaCManBhamGrasp PROPERTY DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})
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
SET_PROPERTY(TARGET PaCManBhamGraspTest PROPERTY RELEASE_POSTFIX ${CMAKE_RELEASE_POSTFIX})
SET_PROPERTY(TARGET PaCManBhamGraspTest PROPERTY DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})
SET_PROPERTY(TARGET PaCManBhamGraspTest PROPERTY PROJECT_LABEL "GraspTest")
SET_PROPERTY(TARGET PaCManBhamGraspTest PROPERTY FOLDER "Bham")
COPY_FILES(PaCManBhamGraspTest ${RUNTIME_OUTPUT_DIRECTORY} ${BHAM_GRASP_FILES})
INSTALL(TARGETS PaCManBhamGraspTest RUNTIME DESTINATION bin)

###############################################################################
#
#	Innsbruck software
#
###############################################################################


# Innsbruck pose estimation library
SET(UIBK_POSE_ESTIMATION_EXTERNALLIB
    ${PROJECT_PARENT}/poseEstimation
)
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
SET(UIBK_POSE_ESTIMATION_UTILS
	${UIBK_POSE_ESTIMATION_EXTERNALLIB}/src/ParametersPoseEstimation.cpp
        ${UIBK_POSE_ESTIMATION_EXTERNALLIB}/src/I_SegmentedObjects.cpp	
)
INCLUDE_DIRECTORIES(
        ${UIBK_POSE_ESTIMATION_EXTERNALLIB}/include
        /home/c7031079/pcl-master/apps/3d_rec_framework/include
)
ADD_LIBRARY(PaCManUIBKPoseEstimation STATIC ${UIBK_POSE_ESTIMATION_HEADERS} ${UIBK_POSE_ESTIMATION_SOURCES} ${UIBK_POSE_ESTIMATION_FILES} ${UIBK_POSE_ESTIMATION_UTILS})
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

