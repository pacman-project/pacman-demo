
###############################################################################
#
#	Bham Common Setup
#
###############################################################################


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