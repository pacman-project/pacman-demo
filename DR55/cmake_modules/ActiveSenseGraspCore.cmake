#################################################################################
# Some Library 
# Usually good to wrapp functionalities in libraries: 
#	visualisation demo doesn't need to link with all (Golem+Grasp) framework
#
#################################################################################

SET(LIB_NAME ActiveSenseGrasp)

SET(PACMAN_ACTIVE_SENSE_GRASP_CORE_HEADERS
        #${PROJECT_ROOT}/include/pacman/${LIB_NAME}/${LIB_NAME}.h
        ${PROJECT_ROOT}/include/pacman/Bham/${LIB_NAME}/Core/ActiveSense.h
        ${PROJECT_ROOT}/include/pacman/Bham/${LIB_NAME}/Core/ActiveSenseOM.h
        ${PROJECT_ROOT}/include/pacman/Bham/${LIB_NAME}/Core/ActiveSenseOM2.h
        ${PROJECT_ROOT}/include/pacman/Bham/${LIB_NAME}/Core/HypothesisSensor.h
        ${PROJECT_ROOT}/include/pacman/Bham/${LIB_NAME}/Core/Collision.h
        ${PROJECT_ROOT}/include/pacman/Bham/${LIB_NAME}/Utils/PCLConversions.h
        ${PROJECT_ROOT}/include/pacman/Bham/${LIB_NAME}/UI/UI.h
        ${PROJECT_ROOT}/include/pacman/Bham/${LIB_NAME}/IO/IO_Adhoc.h
)
SET(PACMAN_ACTIVE_SENSE_GRASP_CORE_SOURCES
        #${PROJECT_ROOT}/src/pacman/${LIB_NAME}/${LIB_NAME}.cpp
        ${PROJECT_ROOT}/src/pacman/Bham/${LIB_NAME}/Core/ActiveSense.cpp
        ${PROJECT_ROOT}/src/pacman/Bham/${LIB_NAME}/Core/ActiveSenseOM.cpp
        ${PROJECT_ROOT}/src/pacman/Bham/${LIB_NAME}/Core/ActiveSenseOM2.cpp
        ${PROJECT_ROOT}/src/pacman/Bham/${LIB_NAME}/Core/HypothesisSensor.cpp
        ${PROJECT_ROOT}/src/pacman/Bham/${LIB_NAME}/Core/Collision.cpp
        ${PROJECT_ROOT}/src/pacman/Bham/${LIB_NAME}/IO/IO_Adhoc.cpp
)
SET(PACMAN_ACTIVE_SENSE_GRASP_CORE_FILES
)

IF(BUILD_DYNAMIC_LIBS)
        ADD_LIBRARY(${LIB_NAME}Core SHARED ${PACMAN_ACTIVE_SENSE_GRASP_CORE_HEADERS} ${PACMAN_ACTIVE_SENSE_GRASP_CORE_SOURCES} ${PACMAN_ACTIVE_SENSE_GRASP_CORE_FILES})
ELSE(BUILD_DYNAMIC_LIBS)
        ADD_LIBRARY(${LIB_NAME}Core STATIC ${PACMAN_ACTIVE_SENSE_GRASP_CORE_HEADERS} ${PACMAN_ACTIVE_SENSE_GRASP_CORE_SOURCES} ${PACMAN_ACTIVE_SENSE_GRASP_CORE_FILES})
ENDIF(BUILD_DYNAMIC_LIBS)


IF (WIN32)
                SET_TARGET_PROPERTIES(${LIB_NAME}Core PROPERTIES COMPILE_FLAGS "${CMAKE_CXX_FLAGS}")
ELSEIF (UNIX)
                SET_TARGET_PROPERTIES(${LIB_NAME}Core PROPERTIES COMPILE_FLAGS "${CMAKE_CXX_FLAGS} -Wno-invalid-offsetof -Wno-old-style-cast")
ENDIF()


IF (WIN32)
        TARGET_LINK_LIBRARIES(
                ${LIB_NAME}Core
                ${GRASP_LIBRARIES}
                ${GOLEM_LIBRARIES}
                ${GOLEM_GRASP_3RD_PARTY_LIBRARIES}
                ${ACTIVE_SENSE_LIBRARIES}
                ${OCTOMAP_LIBRARIES}


        )
ELSEIF (UNIX)
        TARGET_LINK_LIBRARIES(
                ${LIB_NAME}Core
                ${GRASP_LIBRARIES}
                ${GOLEM_LIBRARIES}
                ${GOLEM_GRASP_3RD_PARTY_LIBRARIES}
                ${ACTIVE_SENSE_LIBRARIES}
                ${OCTOMAP_LIBRARIES}
        )
endif()


SET_PROPERTY(TARGET ${LIB_NAME}Core PROPERTY RELEASE_POSTFIX ${CMAKE_RELEASE_POSTFIX})
SET_PROPERTY(TARGET ${LIB_NAME}Core PROPERTY DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})
SET_PROPERTY(TARGET ${LIB_NAME}Core PROPERTY PROJECT_LABEL "${LIB_NAME}")
SET_PROPERTY(TARGET ${LIB_NAME}Core PROPERTY FOLDER "Pacman/${LIB_NAME}")
SOURCE_GROUP("Resource Files" FILES ${PACMAN_ACTIVE_SENSE_GRASP_CORE_FILES})
SOURCE_GROUP("Include Files" FILES ${PACMAN_ACTIVE_SENSE_GRASP_CORE_HEADERS})
INSTALL(TARGETS ${LIB_NAME}Core RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib  COMPONENT activesense_core_execs)
INSTALL(FILES ${PACMAN_ACTIVE_SENSE_GRASP_CORE_HEADERS} DESTINATION include/pacman/Bham/${LIB_NAME}/ COMPONENT activesense_core_headers)
INSTALL(FILES ${DEMO_DR55_SOURCES} DESTINATION src/pacman/Bham/${LIB_NAME}/ COMPONENT demo_sources)
INSTALL(FILES ${PACMAN_ACTIVE_SENSE_GRASP_CORE_FILES} DESTINATION bin COMPONENT activesense_core_configs)

COPY_FILES( ${LIB_NAME}Core ${RUNTIME_OUTPUT_DIRECTORY} ${PACMAN_ACTIVE_SENSE_GRASP_CORE_FILES})
	



