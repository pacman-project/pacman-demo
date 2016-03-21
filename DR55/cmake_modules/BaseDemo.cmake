#################################################################################
# Some Library 
# Usually good to wrapp functionalities in libraries: 
#	visualisation demo doesn't need to link with all (Golem+Grasp) framework
#
#################################################################################

SET(BASE_DEMO_DR55_NAME BaseDemoDR55)

	SET(BASE_DEMO_DR55_SOURCES
		${PROJECT_ROOT}/src/pacman/Bham/Demo/BaseDemo.cpp
	)
	SET(BASE_DEMO_DR55_HEADERS
		${PROJECT_ROOT}/include/pacman/Bham/Demo/BaseDemo.h
	)
	
	
SET(BASE_DEMO_DR55_FILES
)

IF(BUILD_DYNAMIC_LIBS)
        ADD_LIBRARY(${BASE_DEMO_DR55_NAME} SHARED ${BASE_DEMO_DR55_HEADERS} ${BASE_DEMO_DR55_SOURCES} ${BASE_DEMO_DR55_FILES})
ELSE(BUILD_DYNAMIC_LIBS)
        ADD_LIBRARY(${BASE_DEMO_DR55_NAME} STATIC ${BASE_DEMO_DR55_HEADERS} ${BASE_DEMO_DR55_SOURCES} ${BASE_DEMO_DR55_FILES})
ENDIF(BUILD_DYNAMIC_LIBS)


IF (WIN32)
                SET_TARGET_PROPERTIES(${BASE_DEMO_DR55_NAME} PROPERTIES COMPILE_FLAGS "${CMAKE_CXX_FLAGS}")
ELSEIF (UNIX)
                SET_TARGET_PROPERTIES(${BASE_DEMO_DR55_NAME} PROPERTIES COMPILE_FLAGS "${CMAKE_CXX_FLAGS} -Wno-invalid-offsetof -Wno-old-style-cast")
ENDIF()


IF (WIN32)
        TARGET_LINK_LIBRARIES(
                ${BASE_DEMO_DR55_NAME}
                ${GRASP_LIBRARIES}
                ${GOLEM_LIBRARIES}
                ${GOLEM_GRASP_3RD_PARTY_LIBRARIES}

        )
ELSEIF (UNIX)
        TARGET_LINK_LIBRARIES(
                ${BASE_DEMO_DR55_NAME}
                ${GRASP_LIBRARIES}
                ${GOLEM_LIBRARIES}
                ${GOLEM_GRASP_3RD_PARTY_LIBRARIES}
        )
endif()


SET_PROPERTY(TARGET ${BASE_DEMO_DR55_NAME} PROPERTY RELEASE_POSTFIX ${CMAKE_RELEASE_POSTFIX})
SET_PROPERTY(TARGET ${BASE_DEMO_DR55_NAME} PROPERTY DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})
SET_PROPERTY(TARGET ${BASE_DEMO_DR55_NAME} PROPERTY PROJECT_LABEL "${BASE_DEMO_DR55_NAME}")
SET_PROPERTY(TARGET ${BASE_DEMO_DR55_NAME} PROPERTY FOLDER "Pacman/Demo")
SOURCE_GROUP("Resource Files" FILES ${BASE_DEMO_DR55_FILES})
SOURCE_GROUP("Include Files" FILES ${BASE_DEMO_DR55_HEADERS})
INSTALL(TARGETS ${BASE_DEMO_DR55_NAME} RUNTIME DESTINATION bin LIBRARY DESTINATION bin ARCHIVE DESTINATION lib  COMPONENT  base_demo_dr55_execs)
INSTALL(FILES ${BASE_DEMO_DR55_HEADERS} DESTINATION include/pacman/Bham/${LIB_NAME}/ COMPONENT base_demo_dr55_headers)
INSTALL(FILES ${DEMO_DR55_SOURCES} DESTINATION src/pacman/Bham/${LIB_NAME}/ COMPONENT  base_demo_dr55_sources)
INSTALL(FILES ${BASE_DEMO_DR55_FILES} DESTINATION bin COMPONENT  base_demo_dr55_configs)

COPY_FILES( ${BASE_DEMO_DR55_NAME} ${RUNTIME_OUTPUT_DIRECTORY} ${BASE_DEMO_DR55_FILES})
	



