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

# Adding additional cmake module path
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake_modules/)

###############################################################################
#
#	Bham Common Setup
#
###############################################################################

include(CommonBhamSetup)

###############################################################################
#
#	Bham software
#
###############################################################################

include(CommonBhamDependencies)


###############################################################################
#
# Base Demo DR55
#
###############################################################################

include(BaseDemo)

#################################################################################
# 
#   ActiveSense standalone library
#
#################################################################################

find_package(ActiveSense REQUIRED)

IF(${ACTIVE_SENSE_FOUND})
        message("ActiveSense was found!")
ELSE()
         message("ActiveSense NOT found!")
ENDIF(${ACTIVE_SENSE_FOUND})


INCLUDE_DIRECTORIES(
    ${ACTIVE_SENSE_INCLUDE_DIRS} 
    )

LINK_DIRECTORIES(${ACTIVE_SENSE_LIBRARY_DIRS})
SET(ACTIVE_SENSE_LIBRARIES 
  optimized ${ACTIVE_AO_SENSE_LIBRARY}
  debug ${ACTIVE_AO_SENSE_LIBRARY_DEBUG}
  optimized ${ACTIVE_AS_SENSE_LIBRARY}
  debug ${ACTIVE_AS_SENSE_LIBRARY_DEBUG}
)

	
#################################################################################
# 
#   Octomap library
#
#################################################################################


find_package(octomap REQUIRED)
add_definitions(-DOCTOMAP_NODEBUGOUT)


INCLUDE_DIRECTORIES(
    ${OCTOMAP_INCLUDE_DIRS}
    )
	
LINK_DIRECTORIES(${OCTOMAP_LIBRARY_DIRS})
SET(OCTOMAP_LIBRARIES 
  optimized octomap${CMAKE_RELEASE_POSTFIX}
  debug octomap${CMAKE_DEBUG_POSTFIX} 
  optimized octomath${CMAKE_RELEASE_POSTFIX}
  debug octomath${CMAKE_DEBUG_POSTFIX} 
)

#################################################################################
# ActiveSenseGraspCore
# Usually good to wrapp functionalities in libraries: 
#	visualisation demo doesn't need to link with all (Golem+Grasp) framework
#
#################################################################################	

include(ActiveSenseGraspCore)


###############################################################################
#
# Active Sense Grasp Demo Library
#
###############################################################################

SET(DEMO_NAME ActiveSenseGrasp)


SET(PACMAN_ACTIVE_SENSE_GRASP_DEMO_SOURCES
        ${PROJECT_ROOT}/src/pacman/Bham/${DEMO_NAME}/Demo/${DEMO_NAME}Demo.cpp
        #${PROJECT_ROOT}/src/pacman/Bham/${DEMO_NAME}/Demo/BaseDemo.cpp
		#${PROJECT_ROOT}/src/pacman/Bham/Demo/Demo.cpp
)
SET(PACMAN_ACTIVE_SENSE_GRASP_DEMO_HEADERS
        ${PROJECT_ROOT}/include/pacman/Bham/${DEMO_NAME}/Demo/${DEMO_NAME}Demo.h
        #${PROJECT_ROOT}/include/pacman/Bham/${DEMO_NAME}/Demo/BaseDemo.h
		#${PROJECT_ROOT}/include/pacman/Bham/Demo/Demo.h
)
SET(PACMAN_ACTIVE_SENSE_GRASP_DEMO_FILES
        ${PROJECT_ROOT}/resources/pacman/${DEMO_NAME}/ActiveSenseGraspDataPointsCurv.xml
        # ${PROJECT_ROOT}/resources/pacman/${DEMO_NAME}/ActiveSenseGraspDataContactQuery.xml
        # ${PROJECT_ROOT}/resources/pacman/${DEMO_NAME}/ActiveSenseGraspDataContactModel.xml
        # ${PROJECT_ROOT}/resources/pacman/${DEMO_NAME}/ActiveSenseGraspDataTrajectory.xml
        # ${PROJECT_ROOT}/resources/pacman/${DEMO_NAME}/ActiveSenseGraspDemoGraspData.xml
        # ${PROJECT_ROOT}/resources/pacman/${DEMO_NAME}/ActiveSenseGraspDemoGraspData-item-model-1.model
        # ${PROJECT_ROOT}/resources/pacman/${DEMO_NAME}/ActiveSenseGraspDemo_RobotBoris.xml
        # ${PROJECT_ROOT}/resources/pacman/${DEMO_NAME}/ActiveSenseGraspDemo_RobotBorisSim.xml
        # ${PROJECT_ROOT}/resources/pacman/${DEMO_NAME}/ActiveSenseGraspDemo_RobotBorisRealRobot.xml
        # ${PROJECT_ROOT}/resources/pacman/${DEMO_NAME}/ActiveSenseGraspCameraOpenNI.xml
        # ${PROJECT_ROOT}/resources/pacman/${DEMO_NAME}/ActiveSenseGraspCameraOpenNI.cal
        # ${PROJECT_ROOT}/resources/pacman/${DEMO_NAME}/ActiveSenseGraspCameraDepthSim.xml
        # ${PROJECT_ROOT}/resources/pacman/${DEMO_NAME}/ActiveSenseGraspCameraDepthSim.cal
        # ${PROJECT_ROOT}/resources/pacman/${DEMO_NAME}/ActiveSenseGraspCameraPointGrey.xml
        # ${PROJECT_ROOT}/resources/pacman/${DEMO_NAME}/ActiveSenseGraspCameraPointGrey.cal
        ${PROJECT_ROOT}/resources/pacman/${DEMO_NAME}/ActiveSenseGraspDataImage.xml
        ${PROJECT_ROOT}/resources/pacman/${DEMO_NAME}/ActiveSenseGraspDataImageNoCrop.xml
        # ${PROJECT_ROOT}/resources/pacman/${DEMO_NAME}/GolemDeviceRobotBorisTactileSim.xml
		# ${PROJECT_ROOT}/resources/pacman/${DEMO_NAME}/ActiveSenseGraspDemo_RobotBorisDepthSim.xml
)

ADD_LIBRARY(${DEMO_NAME} ${PACMAN_ACTIVE_SENSE_GRASP_DEMO_SOURCES} ${PACMAN_ACTIVE_SENSE_GRASP_DEMO_HEADERS} ${PACMAN_ACTIVE_SENSE_GRASP_DEMO_FILES})
IF (WIN32)
                SET_TARGET_PROPERTIES(${DEMO_NAME} PROPERTIES COMPILE_FLAGS "${CMAKE_CXX_FLAGS}")
ELSEIF (UNIX)
                SET_TARGET_PROPERTIES(${DEMO_NAME} PROPERTIES COMPILE_FLAGS "${CMAKE_CXX_FLAGS} -Wno-invalid-offsetof  -Wno-old-style-cast")
ENDIF()

IF (WIN32)
                TARGET_LINK_LIBRARIES(${DEMO_NAME}
                        ${LIB_NAME}Core
						${BASE_DEMO_DR55_NAME}
		)
ELSEIF (UNIX)
                TARGET_LINK_LIBRARIES(${DEMO_NAME}
                        ${LIB_NAME}Core
						${BASE_DEMO_DR55_NAME}
		)
ENDIF()



COPY_FILES(${DEMO_NAME} ${RUNTIME_OUTPUT_DIRECTORY} ${PACMAN_ACTIVE_SENSE_GRASP_DEMO_FILES})
SET_PROPERTY(TARGET ${DEMO_NAME} PROPERTY RELEASE_POSTFIX ${CMAKE_RELEASE_POSTFIX})
SET_PROPERTY(TARGET ${DEMO_NAME} PROPERTY DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})
INSTALL(TARGETS ${DEMO_NAME} RUNTIME DESTINATION bin  LIBRARY DESTINATION bin ARCHIVE DESTINATION lib  COMPONENT activesense_demo_execs)
INSTALL(FILES ${PACMAN_ACTIVE_SENSE_GRASP_DEMO_HEADERS} DESTINATION include/pacman/Bham/Demo/)
INSTALL(FILES ${PACMAN_ACTIVE_SENSE_GRASP_DEMO_FILES} DESTINATION bin)

SET_PROPERTY(TARGET ${DEMO_NAME} PROPERTY PROJECT_LABEL "${DEMO_NAME}Demo")
SET_PROPERTY(TARGET ${DEMO_NAME} PROPERTY FOLDER "Pacman/${DEMO_NAME}")
SOURCE_GROUP("Include Files" FILES ${PACMAN_ACTIVE_SENSE_GRASP_DEMO_HEADERS})
SOURCE_GROUP("Resource Files" FILES ${PACMAN_ACTIVE_SENSE_GRASP_DEMO_FILES})


###############################################################################
#
# Bham Demo application
#
###############################################################################

option(BUILD_PACMAN_DEMO_DR55 "Build PaCMan DR55 demo application" YES)

IF (BUILD_PACMAN_DEMO_DR55)
	SET(DEMO_DR55_SOURCES
		${PROJECT_ROOT}/src/pacman/Bham/Demo/Demo.cpp
	)
	SET(DEMO_DR55_HEADERS
		${PROJECT_ROOT}/include/pacman/Bham/Demo/Demo.h
	)
	SET(DEMO_DR55_FILES
		${PROJECT_ROOT}/resources/GraspDemoDR55_RobotBoris.xml
		${PROJECT_ROOT}/resources/GraspDemoDR55_RobotBorisSim.xml
		${PROJECT_ROOT}/resources/GraspDemoDR55_RobotBorisDepthSim.xml
		${PROJECT_ROOT}/resources/GraspCameraOpenNIDemoDR55Chest.xml
		${PROJECT_ROOT}/resources/GraspCameraOpenNIDemoDR55RightArm.xml
		${PROJECT_ROOT}/resources/GraspCameraOpenNIDemoDR55LeftArm.xml
		${PROJECT_ROOT}/resources/GraspCameraDepthSimDemoDR55Chest.xml
		${PROJECT_ROOT}/resources/GraspCameraDepthSimDemoDR55RightArm.xml
		${PROJECT_ROOT}/resources/GraspCameraDepthSimDemoDR55LeftArm.xml
		${PROJECT_ROOT}/resources/GraspDataImageDemoDR55.xml
		${PROJECT_ROOT}/resources/GraspDataImageDemoDR55RightArm.xml
		${PROJECT_ROOT}/resources/GraspDataImageDemoDR55LeftArm.xml
		${PROJECT_ROOT}/resources/GraspDataImageDemoDR55Object.xml
		${PROJECT_ROOT}/resources/GraspDataPointsCurvDemoDR55.xml
		${PROJECT_ROOT}/resources/GraspDataContactModelDemoDR55.xml
		${PROJECT_ROOT}/resources/GraspDataContactQueryDemoDR55.xml
		${PROJECT_ROOT}/resources/GraspDataTrajectoryDemoDR55.xml
		${PROJECT_ROOT}/resources/GraspFTClientDemoDR55.xml
		${PROJECT_ROOT}/resources/GolemCtrlRobotBorisDemoDR55Sim.xml
		${PROJECT_ROOT}/resources/GolemCtrlRobotBorisDemoDR55.xml
	)
	SET(DEMO_DR55_CAL_FILES
		${PROJECT_ROOT}/resources/cal/GraspCameraOpenNIDemoDR55Chest.cal
		${PROJECT_ROOT}/resources/cal/GraspCameraOpenNIDemoDR55RightArm.cal
		${PROJECT_ROOT}/resources/cal/GraspCameraOpenNIDemoDR55LeftArm.cal
		${PROJECT_ROOT}/resources/cal/GraspCameraDepthSimDemoDR55Chest.cal
		${PROJECT_ROOT}/resources/cal/GraspCameraDepthSimDemoDR55RightArm.cal
		${PROJECT_ROOT}/resources/cal/GraspCameraDepthSimDemoDR55LeftArm.cal
		${PROJECT_ROOT}/resources/cal/GraspFTClientDemoDR55.cal
	)
	SET(DEMO_DR55_MODEL_FILES
		#${PROJECT_ROOT}/resources/PaCManBhamDemoDrainer.obj
		${PROJECT_ROOT}/resources/PacmanDemoDR55Drainer.obj
	)

    ADD_EXECUTABLE(PacmanDemoDR55 ${DEMO_DR55_SOURCES} ${DEMO_DR55_HEADERS} ${DEMO_DR55_FILES})

	if (WIN32)
		SET_TARGET_PROPERTIES(PacmanDemoDR55 PROPERTIES COMPILE_FLAGS "${CMAKE_CXX_FLAGS}")
	elseif (UNIX)
		SET_TARGET_PROPERTIES(PacmanDemoDR55 PROPERTIES COMPILE_FLAGS "${CMAKE_CXX_FLAGS} -Wno-invalid-offsetof")
	endif()

	IF (WIN32)
        TARGET_LINK_LIBRARIES(
                PacmanDemoDR55
                ${GRASP_LIBRARIES}
                ${GOLEM_LIBRARIES}
                ${GOLEM_GRASP_3RD_PARTY_LIBRARIES}
				${DEMO_NAME}
#                ${ACTIVE_SENSE_LIBRARIES}
#                ${OCTOMAP_LIBRARIES}


        )
	ELSEIF (UNIX)
        TARGET_LINK_LIBRARIES(
                PacmanDemoDR55
                ${GRASP_LIBRARIES}
                ${GOLEM_LIBRARIES}
                ${GOLEM_GRASP_3RD_PARTY_LIBRARIES}
				${DEMO_NAME}
#                ${ACTIVE_SENSE_LIBRARIES}
#                ${OCTOMAP_LIBRARIES}
        )
	endif()
	
	COPY_FILES(PacmanDemoDR55 ${RUNTIME_OUTPUT_DIRECTORY} ${DEMO_DR55_FILES})
	COPY_FILES(PacmanDemoDR55 ${RUNTIME_OUTPUT_DIRECTORY} ${DEMO_DR55_MODEL_FILES})
	COPY_FILES(PacmanDemoDR55 ${RUNTIME_OUTPUT_DIRECTORY} ${DEMO_DR55_CAL_FILES})
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


