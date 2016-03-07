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

option(BUILD_PACMAN_DEMO_DR55 "Build PaCMan DR55 demo application" YES)

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
	${GOLEM_INCLUDE}
	${GRASP_INCLUDE}
)

###############################################################################
#
# Bham Demo application
#
###############################################################################

IF (BUILD_PACMAN_DEMO_DR55)
	SET(DEMO_DR55_SOURCES
		${PROJECT_ROOT}/src/pacman/Bham/Demo/Demo.cpp
	)
	SET(DEMO_DR55_HEADERS
		${PROJECT_ROOT}/include/pacman/Bham/Demo/Demo.h
	)
	SET(DEMO_DR55_FILES
		${PROJECT_ROOT}/resources/Bham/GraspDemoDR55_RobotBoris.xml
		${PROJECT_ROOT}/resources/Bham/GraspCameraOpenNIDemoDR55.xml
		${PROJECT_ROOT}/resources/Bham/GraspDataImageDemoDR55.xml
		${PROJECT_ROOT}/resources/Bham/GraspDataImageObjectDemoDR55.xml
		${PROJECT_ROOT}/resources/Bham/GraspDataPointsCurvDemoDR55.xml
		${PROJECT_ROOT}/resources/Bham/GraspDataContactModelDemoDR55.xml
		${PROJECT_ROOT}/resources/Bham/GraspDataContactQueryDemoDR55.xml
		${PROJECT_ROOT}/resources/Bham/GraspDataTrajectoryDemoDR55.xml
		${PROJECT_ROOT}/resources/Bham/GraspCameraOpenNIDemoDR55.cal
	)
	SET(DEMO_DR55_MODEL_FILES
		#${PROJECT_ROOT}/resources/Bham/PaCManBhamDemoDrainer.obj
		${PROJECT_ROOT}/resources/Bham/PacmanDemoDR55Container.obj
	)
	
	ADD_EXECUTABLE(PacmanDemoDR55 ${DEMO_DR55_SOURCES} ${DEMO_DR55_HEADERS} ${DEMO_DR55_FILES})
	if (WIN32)
		SET_TARGET_PROPERTIES(PacmanDemoDR55 PROPERTIES COMPILE_FLAGS "${CMAKE_CXX_FLAGS}")
	elseif (UNIX)
		SET_TARGET_PROPERTIES(PacmanDemoDR55 PROPERTIES COMPILE_FLAGS "${CMAKE_CXX_FLAGS} -Wno-invalid-offsetof")
	endif()
#	TARGET_LINK_LIBRARIES(PacmanDemoDR55 GraspApp GraspContact GraspDataImage)
	if (WIN32)
		TARGET_LINK_LIBRARIES(
			PacmanDemoDR55
			optimized GraspCore${CMAKE_RELEASE_POSTFIX} debug GraspCore${CMAKE_DEBUG_POSTFIX}
			optimized GraspContact${CMAKE_RELEASE_POSTFIX} debug GraspContact${CMAKE_DEBUG_POSTFIX}
			optimized GraspApp${CMAKE_RELEASE_POSTFIX} debug GraspApp${CMAKE_DEBUG_POSTFIX}
			optimized GraspActiveCtrl${CMAKE_RELEASE_POSTFIX} debug GraspActiveCtrl${CMAKE_DEBUG_POSTFIX}
            optimized GraspDataPointsCurv${CMAKE_RELEASE_POSTFIX} debug GraspDataPointsCurv${CMAKE_DEBUG_POSTFIX}
			optimized GraspDataImage${CMAKE_RELEASE_POSTFIX} debug GraspDataImage${CMAKE_DEBUG_POSTFIX}
			optimized GraspArmHandForce${CMAKE_RELEASE_POSTFIX} debug GraspArmHandForce${CMAKE_DEBUG_POSTFIX}		 
			#NxCharacter64 PhysXCooking64 PhysXCore64 PhysXLoader64
			expat freeglut Gdiplus
			optimized GolemDefs${CMAKE_RELEASE_POSTFIX} debug GolemDefs${CMAKE_DEBUG_POSTFIX}
			optimized GolemMath${CMAKE_RELEASE_POSTFIX} debug GolemMath${CMAKE_DEBUG_POSTFIX}
			optimized GolemSys${CMAKE_RELEASE_POSTFIX} debug GolemSys${CMAKE_DEBUG_POSTFIX}
			optimized GolemTools${CMAKE_RELEASE_POSTFIX} debug GolemTools${CMAKE_DEBUG_POSTFIX}
			optimized GolemCtrl${CMAKE_RELEASE_POSTFIX} debug GolemCtrl${CMAKE_DEBUG_POSTFIX}
			optimized GolemPlan${CMAKE_RELEASE_POSTFIX} debug GolemPlan${CMAKE_DEBUG_POSTFIX}
			optimized GolemUI${CMAKE_RELEASE_POSTFIX} debug GolemUI${CMAKE_DEBUG_POSTFIX}
			optimized GolemUICtrl${CMAKE_RELEASE_POSTFIX} debug GolemUICtrl${CMAKE_DEBUG_POSTFIX}
			optimized GolemDeviceSM${CMAKE_RELEASE_POSTFIX} debug GolemDeviceSM${CMAKE_DEBUG_POSTFIX}
			optimized GolemDeviceSingleCtrl${CMAKE_RELEASE_POSTFIX} debug GolemDeviceSingleCtrl${CMAKE_DEBUG_POSTFIX}
            optimized GolemDeviceMultiCtrl${CMAKE_RELEASE_POSTFIX} debug GolemDeviceMultiCtrl${CMAKE_DEBUG_POSTFIX}
            ${CMAKE_DL_LIBS}
			optimized CamcalbCalb${CMAKE_RELEASE_POSTFIX} debug CamcalbCalb${CMAKE_DEBUG_POSTFIX}
			optimized CamcalbMatas${CMAKE_RELEASE_POSTFIX} debug CamcalbMatas${CMAKE_DEBUG_POSTFIX}		 
			${PCL_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS}
		)
	elseif (UNIX)
		TARGET_LINK_LIBRARIES(
			PacmanDemoDR55
			${PCL_LIBRARIES}
			optimized CamcalbCalb${CMAKE_RELEASE_POSTFIX} debug CamcalbCalb${CMAKE_DEBUG_POSTFIX}
			optimized CamcalbMatas${CMAKE_RELEASE_POSTFIX} debug CamcalbMatas${CMAKE_DEBUG_POSTFIX}		 
			${OpenCV_LIBS}
			optimized GraspCore${CMAKE_RELEASE_POSTFIX} debug GraspCore${CMAKE_DEBUG_POSTFIX}
			optimized GraspContact${CMAKE_RELEASE_POSTFIX} debug GraspContact${CMAKE_DEBUG_POSTFIX}
			optimized GraspApp${CMAKE_RELEASE_POSTFIX} debug GraspApp${CMAKE_DEBUG_POSTFIX}	
			optimized GraspActiveCtrl${CMAKE_RELEASE_POSTFIX} debug GraspActiveCtrl${CMAKE_DEBUG_POSTFIX}
            optimized GraspDataPointsCurv${CMAKE_RELEASE_POSTFIX} debug GraspDataPointsCurv${CMAKE_DEBUG_POSTFIX}
			optimized GraspDataImage${CMAKE_RELEASE_POSTFIX} debug GraspDataImage${CMAKE_DEBUG_POSTFIX}
			optimized GraspArmHandForce${CMAKE_RELEASE_POSTFIX} debug GraspArmHandForce${CMAKE_DEBUG_POSTFIX}	 
			${Boost_LIBRARIES}
			optimized GolemDefs${CMAKE_RELEASE_POSTFIX} debug GolemDefs${CMAKE_DEBUG_POSTFIX}
			optimized GolemSys${CMAKE_RELEASE_POSTFIX} debug GolemSys${CMAKE_DEBUG_POSTFIX}
			optimized GolemTools${CMAKE_RELEASE_POSTFIX} debug GolemTools${CMAKE_DEBUG_POSTFIX}
			optimized GolemCtrl${CMAKE_RELEASE_POSTFIX} debug GolemCtrl${CMAKE_DEBUG_POSTFIX}
			optimized GolemPlan${CMAKE_RELEASE_POSTFIX} debug GolemPlan${CMAKE_DEBUG_POSTFIX}
			optimized GolemUICtrl${CMAKE_RELEASE_POSTFIX} debug GolemUICtrl${CMAKE_DEBUG_POSTFIX}
			optimized GolemDeviceSM${CMAKE_RELEASE_POSTFIX} debug GolemDeviceSM${CMAKE_DEBUG_POSTFIX}
			optimized GolemDeviceSingleCtrl${CMAKE_RELEASE_POSTFIX} debug GolemDeviceSingleCtrl${CMAKE_DEBUG_POSTFIX}
			${CMAKE_DL_LIBS}
            optimized GolemDeviceMultiCtrl${CMAKE_RELEASE_POSTFIX} debug GolemDeviceMultiCtrl${CMAKE_DEBUG_POSTFIX}
			optimized GolemMath${CMAKE_RELEASE_POSTFIX} debug GolemMath${CMAKE_DEBUG_POSTFIX}
			optimized GolemUI${CMAKE_RELEASE_POSTFIX} debug GolemUI${CMAKE_DEBUG_POSTFIX}
			GL
			GLU
			glut
			#NxCharacter NxCooking PhysXCore PhysXLoader
		)
	endif(WIN32)	
	COPY_FILES(PacmanDemoDR55 ${RUNTIME_OUTPUT_DIRECTORY} ${DEMO_DR55_FILES})
	COPY_FILES(PacmanDemoDR55 ${RUNTIME_OUTPUT_DIRECTORY} ${DEMO_DR55_MODEL_FILES})
	SET_PROPERTY(TARGET PacmanDemoDR55 PROPERTY RELEASE_POSTFIX ${CMAKE_RELEASE_POSTFIX})
	SET_PROPERTY(TARGET PacmanDemoDR55 PROPERTY DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})
	
	INSTALL(TARGETS PacmanDemoDR55 RUNTIME DESTINATION bin COMPONENT demo_execs)
	INSTALL(FILES ${DEMO_DR55_HEADERS} DESTINATION include/pacman/Bham/Demo/ COMPONENT demo_headers)
	INSTALL(FILES ${DEMO_DR55_SOURCES} DESTINATION src/pacman/Bham/Demo/ COMPONENT demo_sources)
	INSTALL(FILES ${DEMO_DR55_FILES} DESTINATION bin COMPONENT demo_configs)
	INSTALL(FILES ${DEMO_DR55_MODEL_FILES} DESTINATION bin COMPONENT demo_configs)

	SET_PROPERTY(TARGET PacmanDemoDR55 PROPERTY PROJECT_LABEL "DR55")
	SET_PROPERTY(TARGET PacmanDemoDR55 PROPERTY FOLDER "Pacman/Demo")
	SOURCE_GROUP("Include Files" FILES ${DEMO_DR55_HEADERS})
	SOURCE_GROUP("Resource Files" FILES ${DEMO_DR55_FILES})
ENDIF (BUILD_PACMAN_DEMO_DR55)


