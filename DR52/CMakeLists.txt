###############################################################################
#
#	Prerequisites
#
###############################################################################

# cmake requirements
cmake_minimum_required(VERSION 2.8)

# Build options have to be before PROJECT(...)
SET(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE PATH "Configuration types")
SET(CMAKE_BUILD_TYPE "Release" CACHE PATH "Current build configuration")

# PaCMan main workspace
PROJECT(pacman)

# Folders
SET_PROPERTY(GLOBAL PROPERTY USE_FOLDERS ON)

# Project parent directory
GET_FILENAME_COMPONENT(PROJECT_PARENT ../.. ABSOLUTE CACHE INTERNAL "Path prefix for the project parent")

# Project root directory
GET_FILENAME_COMPONENT(PROJECT_ROOT . ABSOLUTE CACHE INTERNAL "Path prefix for the project")

# Set the path where other thing should be relative to
GET_FILENAME_COMPONENT(CMAKE_SOURCE_DIR . ABSOLUTE CACHE INTERNAL "")

# Architecture
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
	SET(X86_64 1)
	#MESSAGE("Architecture x86 64 bit")
else(CMAKE_SIZEOF_VOID_P EQUAL 8)
	SET(X86_32 1)
	#MESSAGE("Architecture x86 32 bit")
endif(CMAKE_SIZEOF_VOID_P EQUAL 8)

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

###############################################################################
#
#	Bham software
#
###############################################################################

if (WIN32)
	if(X86_64)
		SET(PROGRAM_FILES $ENV{ProgramW6432})
		SET(X86_WIN "64")
		SET(X86_X64 "/x64")
	else(X86_64)
		SET(PROGRAM_FILES $ENV{ProgramFiles})
		SET(X86_WIN "32")
		SET(X86_X64 "")
	endif(X86_64)
	
	if (MSVC90)
		SET(MSVC_VER "90")
	elseif(MSVC10)
		SET(MSVC_VER "100")
	elseif(MSVC11)
		SET(MSVC_VER "110")
	elseif(MSVC12)
		SET(MSVC_VER "120")
	endif()
	
	# Build options
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /D_CRT_SECURE_NO_DEPRECATE")
	SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /Ox")
	SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /bigobj")
	SET(CMAKE_DLL_EXPORT_FLAGS "/DGOLEM_LIBRARY_DECLDIR_EXPORT /wd4251 /wd4275")
	
	# Library settings
	option(BUILD_DYNAMIC_LIBS "Build dynamic library versions" NO)
	mark_as_advanced(BUILD_DYNAMIC_LIBS)

	# Output
	SET(OUTPUT_DIRECTORY_POSTFIX "")

	# Set the install directory.
	SET(CMAKE_INSTALL_PREFIX "${PROGRAM_FILES}" CACHE PATH "Installation prefix")

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
	# Boost
	ADD_DEFINITIONS(-DBOOST_ALL_NO_LIB)
	SET(Boost_USE_STATIC_LIBS ON)
	SET(Boost_ALL_DYN_LINK OFF)
	FIND_PACKAGE(Boost COMPONENTS system filesystem thread)
	# OpenCV
	FIND_PACKAGE(OpenCV REQUIRED)
	# PCL
	FIND_PACKAGE(PCL REQUIRED)
elseif (UNIX)
	# Build options
	if(X86_64)
		SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -DLINUX -Dlinux -DLINUX64 -D__x86_64__ -DNX64 -Wno-deprecated-declarations -std=c++0x")
	else(X86_64)
		SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -DLINUX -Dlinux -DLINUX32 -m32 -Di386 -DNX32 -Wno-deprecated-declarations -std=c++0x")
	endif(X86_64)
	SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")
	SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -ggdb")
	SET(CMAKE_DLL_EXPORT_FLAGS " ")

	# Library settings
	option(BUILD_DYNAMIC_LIBS "Build dynamic library versions" YES)
	mark_as_advanced(BUILD_DYNAMIC_LIBS)

	# Output
	SET(OUTPUT_DIRECTORY_POSTFIX "")
	
	# Set the install directory.
	SET(CMAKE_INSTALL_PREFIX /usr/local CACHE PATH "Installation prefix")

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
	# Boost
	#ADD_DEFINITIONS(-DBOOST_ALL_NO_LIB)
	#SET(Boost_USE_STATIC_LIBS OFF)
	#SET(Boost_ALL_DYN_LINK ON)
	FIND_PACKAGE(Boost COMPONENTS system filesystem thread)
	# OpenCV
	FIND_PACKAGE(OpenCV REQUIRED)
	# PCL
	FIND_PACKAGE(PCL REQUIRED)
endif()

ADD_DEFINITIONS(
	${OPENCV_DEFINITIONS}
	${PCL_DEFINITIONS}
)
LINK_DIRECTORIES(
	${Boost_LIBRARY_DIRS}
	${OPENCV_LIBRARIES}
	${PCL_LIBRARY_DIRS}
	${EXPAT_LIBRARY}
	${FREEGLUT_LIBRARY}
	${PHYSX_LIBRARY}
	${GOLEM_LIBRARY}
	${GOLEM_BINARIES}
	${GRASP_LIBRARY}
	${GRASP_BINARIES}
)
INCLUDE_DIRECTORIES(
	${PROJECT_ROOT}/include
	${Boost_INCLUDE_DIRS}
	${OPENCV_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
	${EXPAT_INCLUDE}
	${FREEGLUT_INCLUDE}
	${PHYSX_INCLUDE}/Physics/include
	${PHYSX_INCLUDE}/Foundation/include
	${PHYSX_INCLUDE}/PhysXLoader/include
	${PHYSX_INCLUDE}/Cooking/include
	${GOLEM_INCLUDE}
	${GRASP_INCLUDE}
)

###############################################################################
#
# Bham ActiveSens library
#
###############################################################################

SET(PACMAN_BHAM_ACTIVESENS_HEADERS
	${PROJECT_ROOT}/include/pacman/Bham/ActiveSens/ActiveSens.h
)
SET(PACMAN_BHAM_ACTIVESENS_SOURCES
	${PROJECT_ROOT}/src/pacman/Bham/ActiveSens/ActiveSens.cpp
)
SET(PACMAN_BHAM_ACTIVESENS_FILES
)
if(BUILD_DYNAMIC_LIBS)
	ADD_LIBRARY(PaCManBhamActiveSens SHARED ${PACMAN_BHAM_ACTIVESENS_HEADERS} ${PACMAN_BHAM_ACTIVESENS_SOURCES} ${PACMAN_BHAM_ACTIVESENS_FILES})
else(BUILD_DYNAMIC_LIBS)
	ADD_LIBRARY(PaCManBhamActiveSens STATIC ${PACMAN_BHAM_ACTIVESENS_HEADERS} ${PACMAN_BHAM_ACTIVESENS_SOURCES} ${PACMAN_BHAM_ACTIVESENS_FILES})
endif(BUILD_DYNAMIC_LIBS)
if (WIN32)
	TARGET_LINK_LIBRARIES(
		PaCManBhamActiveSens
		optimized GraspCore${CMAKE_RELEASE_POSTFIX} debug GraspCore${CMAKE_DEBUG_POSTFIX}
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
		optimized GolemDeviceSM${CMAKE_RELEASE_POSTFIX} debug GolemDeviceSM${CMAKE_DEBUG_POSTFIX}
		optimized CamcalbCalb${CMAKE_RELEASE_POSTFIX} debug CamcalbCalb${CMAKE_DEBUG_POSTFIX}
		optimized CamcalbMatas${CMAKE_RELEASE_POSTFIX} debug CamcalbMatas${CMAKE_DEBUG_POSTFIX}		 
		${PCL_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS}
	)
elseif (UNIX)
	TARGET_LINK_LIBRARIES(
		PaCManBhamActiveSens
		${PCL_LIBRARIES}
		optimized CamcalbCalb${CMAKE_RELEASE_POSTFIX} debug CamcalbCalb${CMAKE_DEBUG_POSTFIX}
		optimized CamcalbMatas${CMAKE_RELEASE_POSTFIX} debug CamcalbMatas${CMAKE_DEBUG_POSTFIX}		 
		${OpenCV_LIBS}
		optimized GraspCore${CMAKE_RELEASE_POSTFIX} debug GraspCore${CMAKE_DEBUG_POSTFIX}
		optimized GraspGrasp${CMAKE_RELEASE_POSTFIX} debug GraspGrasp${CMAKE_DEBUG_POSTFIX}
		optimized GraspApp${CMAKE_RELEASE_POSTFIX} debug GraspApp${CMAKE_DEBUG_POSTFIX}		 
		${Boost_LIBRARIES}
		optimized GolemDefs${CMAKE_RELEASE_POSTFIX} debug GolemDefs${CMAKE_DEBUG_POSTFIX}
		optimized GolemSys${CMAKE_RELEASE_POSTFIX} debug GolemSys${CMAKE_DEBUG_POSTFIX}
		optimized GolemTools${CMAKE_RELEASE_POSTFIX} debug GolemTools${CMAKE_DEBUG_POSTFIX}
		optimized GolemCtrl${CMAKE_RELEASE_POSTFIX} debug GolemCtrl${CMAKE_DEBUG_POSTFIX}
		optimized GolemPlan${CMAKE_RELEASE_POSTFIX} debug GolemPlan${CMAKE_DEBUG_POSTFIX}
		optimized GolemPhysCtrl${CMAKE_RELEASE_POSTFIX} debug GolemPhysCtrl${CMAKE_DEBUG_POSTFIX}
		optimized GolemDeviceSM${CMAKE_RELEASE_POSTFIX} debug GolemDeviceSM${CMAKE_DEBUG_POSTFIX}
		${CMAKE_DL_LIBS}
		optimized GolemMath${CMAKE_RELEASE_POSTFIX} debug GolemMath${CMAKE_DEBUG_POSTFIX}
		optimized GolemPhys${CMAKE_RELEASE_POSTFIX} debug GolemPhys${CMAKE_DEBUG_POSTFIX}
		GL
		GLU
		glut
		NxCharacter NxCooking PhysXCore PhysXLoader
	)
endif()	
SET_PROPERTY(TARGET PaCManBhamActiveSens PROPERTY RELEASE_POSTFIX ${CMAKE_RELEASE_POSTFIX})
SET_PROPERTY(TARGET PaCManBhamActiveSens PROPERTY DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})
SET_PROPERTY(TARGET PaCManBhamActiveSens PROPERTY PROJECT_LABEL "ActiveSens")
SET_PROPERTY(TARGET PaCManBhamActiveSens PROPERTY FOLDER "Bham")
SOURCE_GROUP("Resource Files" FILES ${PACMAN_BHAM_ACTIVESENS_FILES})
INSTALL(TARGETS PaCManBhamActiveSens RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
INSTALL(FILES ${PACMAN_BHAM_ACTIVESENS_HEADERS} DESTINATION include/pacman/Bham/ActiveSens/)
INSTALL(FILES ${PACMAN_BHAM_ACTIVESENS_FILES} DESTINATION bin)

###############################################################################
#
# Bham ContactPred library
#
###############################################################################

SET(PACMAN_BHAM_CONTACTPRED_HEADERS
	${PROJECT_ROOT}/include/pacman/Bham/ContactPred/ContactPred.h
)
SET(PACMAN_BHAM_CONTACTPRED_SOURCES
	${PROJECT_ROOT}/src/pacman/Bham/ContactPred/ContactPred.cpp
)
SET(PACMAN_BHAM_CONTACTPRED_FILES
)
if(BUILD_DYNAMIC_LIBS)
	ADD_LIBRARY(PaCManBhamContactPred SHARED ${PACMAN_BHAM_CONTACTPRED_HEADERS} ${PACMAN_BHAM_CONTACTPRED_SOURCES} ${PACMAN_BHAM_CONTACTPRED_FILES})
else(BUILD_DYNAMIC_LIBS)
	ADD_LIBRARY(PaCManBhamContactPred STATIC ${PACMAN_BHAM_CONTACTPRED_HEADERS} ${PACMAN_BHAM_CONTACTPRED_SOURCES} ${PACMAN_BHAM_CONTACTPRED_FILES})
endif(BUILD_DYNAMIC_LIBS)
if (WIN32)
	TARGET_LINK_LIBRARIES(
		PaCManBhamContactPred
		optimized GraspCore${CMAKE_RELEASE_POSTFIX} debug GraspCore${CMAKE_DEBUG_POSTFIX}
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
		optimized GolemDeviceSM${CMAKE_RELEASE_POSTFIX} debug GolemDeviceSM${CMAKE_DEBUG_POSTFIX}
		optimized CamcalbCalb${CMAKE_RELEASE_POSTFIX} debug CamcalbCalb${CMAKE_DEBUG_POSTFIX}
		optimized CamcalbMatas${CMAKE_RELEASE_POSTFIX} debug CamcalbMatas${CMAKE_DEBUG_POSTFIX}		 
		${PCL_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS}
	)
elseif (UNIX)
	TARGET_LINK_LIBRARIES(
		PaCManBhamContactPred
		${PCL_LIBRARIES}
		optimized CamcalbCalb${CMAKE_RELEASE_POSTFIX} debug CamcalbCalb${CMAKE_DEBUG_POSTFIX}
		optimized CamcalbMatas${CMAKE_RELEASE_POSTFIX} debug CamcalbMatas${CMAKE_DEBUG_POSTFIX}		 
		${OpenCV_LIBS}
		optimized GraspCore${CMAKE_RELEASE_POSTFIX} debug GraspCore${CMAKE_DEBUG_POSTFIX}
		optimized GraspGrasp${CMAKE_RELEASE_POSTFIX} debug GraspGrasp${CMAKE_DEBUG_POSTFIX}
		optimized GraspApp${CMAKE_RELEASE_POSTFIX} debug GraspApp${CMAKE_DEBUG_POSTFIX}		 
		${Boost_LIBRARIES}
		optimized GolemDefs${CMAKE_RELEASE_POSTFIX} debug GolemDefs${CMAKE_DEBUG_POSTFIX}
		optimized GolemSys${CMAKE_RELEASE_POSTFIX} debug GolemSys${CMAKE_DEBUG_POSTFIX}
		optimized GolemTools${CMAKE_RELEASE_POSTFIX} debug GolemTools${CMAKE_DEBUG_POSTFIX}
		optimized GolemCtrl${CMAKE_RELEASE_POSTFIX} debug GolemCtrl${CMAKE_DEBUG_POSTFIX}
		optimized GolemPlan${CMAKE_RELEASE_POSTFIX} debug GolemPlan${CMAKE_DEBUG_POSTFIX}
		optimized GolemPhysCtrl${CMAKE_RELEASE_POSTFIX} debug GolemPhysCtrl${CMAKE_DEBUG_POSTFIX}
		optimized GolemDeviceSM${CMAKE_RELEASE_POSTFIX} debug GolemDeviceSM${CMAKE_DEBUG_POSTFIX}
		${CMAKE_DL_LIBS}
		optimized GolemMath${CMAKE_RELEASE_POSTFIX} debug GolemMath${CMAKE_DEBUG_POSTFIX}
		optimized GolemPhys${CMAKE_RELEASE_POSTFIX} debug GolemPhys${CMAKE_DEBUG_POSTFIX}
		GL
		GLU
		glut
		NxCharacter NxCooking PhysXCore PhysXLoader
	)
endif()	
SET_PROPERTY(TARGET PaCManBhamContactPred PROPERTY RELEASE_POSTFIX ${CMAKE_RELEASE_POSTFIX})
SET_PROPERTY(TARGET PaCManBhamContactPred PROPERTY DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})
SET_PROPERTY(TARGET PaCManBhamContactPred PROPERTY PROJECT_LABEL "ContactPred")
SET_PROPERTY(TARGET PaCManBhamContactPred PROPERTY FOLDER "Bham")
SOURCE_GROUP("Resource Files" FILES ${PACMAN_BHAM_CONTACTPRED_FILES})
INSTALL(TARGETS PaCManBhamContactPred RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
INSTALL(FILES ${PACMAN_BHAM_CONTACTPRED_HEADERS} DESTINATION include/pacman/Bham/ContactPred/)
INSTALL(FILES ${PACMAN_BHAM_CONTACTPRED_FILES} DESTINATION bin)

###############################################################################
#
# Bham Tracker library
#
###############################################################################

SET(PACMAN_BHAM_TRACKER_HEADERS
	${PROJECT_ROOT}/include/pacman/Bham/Tracker/Tracker.h
)
SET(PACMAN_BHAM_TRACKER_SOURCES
	${PROJECT_ROOT}/src/pacman/Bham/Tracker/Tracker.cpp
)
SET(PACMAN_BHAM_TRACKER_FILES
)
if(BUILD_DYNAMIC_LIBS)
	ADD_LIBRARY(PaCManBhamTracker SHARED ${PACMAN_BHAM_TRACKER_HEADERS} ${PACMAN_BHAM_TRACKER_SOURCES} ${PACMAN_BHAM_TRACKER_FILES})
else(BUILD_DYNAMIC_LIBS)
	ADD_LIBRARY(PaCManBhamTracker STATIC ${PACMAN_BHAM_TRACKER_HEADERS} ${PACMAN_BHAM_TRACKER_SOURCES} ${PACMAN_BHAM_TRACKER_FILES})
endif(BUILD_DYNAMIC_LIBS)
if (WIN32)
	TARGET_LINK_LIBRARIES(
		PaCManBhamTracker
		optimized GraspCore${CMAKE_RELEASE_POSTFIX} debug GraspCore${CMAKE_DEBUG_POSTFIX}
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
		optimized GolemDeviceSM${CMAKE_RELEASE_POSTFIX} debug GolemDeviceSM${CMAKE_DEBUG_POSTFIX}
		optimized CamcalbCalb${CMAKE_RELEASE_POSTFIX} debug CamcalbCalb${CMAKE_DEBUG_POSTFIX}
		optimized CamcalbMatas${CMAKE_RELEASE_POSTFIX} debug CamcalbMatas${CMAKE_DEBUG_POSTFIX}		 
		${PCL_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS}
	)
elseif (UNIX)
	TARGET_LINK_LIBRARIES(
		PaCManBhamTracker
		${PCL_LIBRARIES}
		optimized CamcalbCalb${CMAKE_RELEASE_POSTFIX} debug CamcalbCalb${CMAKE_DEBUG_POSTFIX}
		optimized CamcalbMatas${CMAKE_RELEASE_POSTFIX} debug CamcalbMatas${CMAKE_DEBUG_POSTFIX}		 
		${OpenCV_LIBS}
		optimized GraspCore${CMAKE_RELEASE_POSTFIX} debug GraspCore${CMAKE_DEBUG_POSTFIX}
		optimized GraspGrasp${CMAKE_RELEASE_POSTFIX} debug GraspGrasp${CMAKE_DEBUG_POSTFIX}
		optimized GraspApp${CMAKE_RELEASE_POSTFIX} debug GraspApp${CMAKE_DEBUG_POSTFIX}		 
		${Boost_LIBRARIES}
		optimized GolemDefs${CMAKE_RELEASE_POSTFIX} debug GolemDefs${CMAKE_DEBUG_POSTFIX}
		optimized GolemSys${CMAKE_RELEASE_POSTFIX} debug GolemSys${CMAKE_DEBUG_POSTFIX}
		optimized GolemTools${CMAKE_RELEASE_POSTFIX} debug GolemTools${CMAKE_DEBUG_POSTFIX}
		optimized GolemCtrl${CMAKE_RELEASE_POSTFIX} debug GolemCtrl${CMAKE_DEBUG_POSTFIX}
		optimized GolemPlan${CMAKE_RELEASE_POSTFIX} debug GolemPlan${CMAKE_DEBUG_POSTFIX}
		optimized GolemPhysCtrl${CMAKE_RELEASE_POSTFIX} debug GolemPhysCtrl${CMAKE_DEBUG_POSTFIX}
		optimized GolemDeviceSM${CMAKE_RELEASE_POSTFIX} debug GolemDeviceSM${CMAKE_DEBUG_POSTFIX}
		${CMAKE_DL_LIBS}
		optimized GolemMath${CMAKE_RELEASE_POSTFIX} debug GolemMath${CMAKE_DEBUG_POSTFIX}
		optimized GolemPhys${CMAKE_RELEASE_POSTFIX} debug GolemPhys${CMAKE_DEBUG_POSTFIX}
		GL
		GLU
		glut
		NxCharacter NxCooking PhysXCore PhysXLoader
	)
endif()	
SET_PROPERTY(TARGET PaCManBhamTracker PROPERTY RELEASE_POSTFIX ${CMAKE_RELEASE_POSTFIX})
SET_PROPERTY(TARGET PaCManBhamTracker PROPERTY DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})
SET_PROPERTY(TARGET PaCManBhamTracker PROPERTY PROJECT_LABEL "Tracker")
SET_PROPERTY(TARGET PaCManBhamTracker PROPERTY FOLDER "Bham")
SOURCE_GROUP("Resource Files" FILES ${PACMAN_BHAM_TRACKER_FILES})
INSTALL(TARGETS PaCManBhamTracker RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib)
INSTALL(FILES ${PACMAN_BHAM_TRACKER_HEADERS} DESTINATION include/pacman/Bham/Tracker/)
INSTALL(FILES ${PACMAN_BHAM_TRACKER_FILES} DESTINATION bin)

###############################################################################
#
# Bham Demo application
#
###############################################################################

SET(PACMAN_BHAM_DEMO_SOURCES
	${PROJECT_ROOT}/src/pacman/Bham/Demo/Demo.cpp
)
SET(PACMAN_BHAM_DEMO_HEADERS
	${PROJECT_ROOT}/include/pacman/Bham/Demo/Demo.h
)
SET(PACMAN_BHAM_DEMO_FILES
	${PROJECT_ROOT}/resources/Bham/PaCManBhamDemo.xml
)

ADD_EXECUTABLE(PaCManBhamDemo ${PACMAN_BHAM_DEMO_SOURCES} ${PACMAN_BHAM_DEMO_HEADERS} ${PACMAN_BHAM_DEMO_FILES})
SET_TARGET_PROPERTIES(PaCManBhamDemo PROPERTIES COMPILE_FLAGS "${CMAKE_CXX_FLAGS}")
TARGET_LINK_LIBRARIES(PaCManBhamDemo PaCManBhamActiveSens PaCManBhamContactPred PaCManBhamTracker)
COPY_FILES(PaCManBhamDemo ${RUNTIME_OUTPUT_DIRECTORY} ${PACMAN_BHAM_DEMO_FILES})
SET_PROPERTY(TARGET PaCManBhamDemo PROPERTY RELEASE_POSTFIX ${CMAKE_RELEASE_POSTFIX})
SET_PROPERTY(TARGET PaCManBhamDemo PROPERTY DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})
INSTALL(TARGETS PaCManBhamDemo RUNTIME DESTINATION bin)
INSTALL(FILES ${PACMAN_BHAM_DEMO_HEADERS} DESTINATION include/pacman/Bham/Demo/)
INSTALL(FILES ${PACMAN_BHAM_DEMO_FILES} DESTINATION bin)

SET_PROPERTY(TARGET PaCManBhamDemo PROPERTY PROJECT_LABEL "Demo")
SET_PROPERTY(TARGET PaCManBhamDemo PROPERTY FOLDER "Bham")
SOURCE_GROUP("Include Files" FILES ${PACMAN_BHAM_DEMO_HEADERS})
SOURCE_GROUP("Resource Files" FILES ${PACMAN_BHAM_DEMO_FILES})

