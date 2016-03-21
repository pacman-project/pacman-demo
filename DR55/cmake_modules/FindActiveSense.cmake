# - Try to find ActiveSense
# Once done this will define
#  ACTIVE_SENSE_FOUND - System has ActiveSense
#  ACTIVE_SENSE_INCLUDE_DIRS - The ActiveSense include directories
#  ACTIVE_SENSE_LIBRARIES - The libraries needed to use ActiveSense
#  ACTIVE_SENSE_DEFINITIONS - Compiler switches required for using ActiveSense

find_package(PkgConfig)
pkg_check_modules(PC_ACTIVE_SENSE QUIET active-sense-1.0)
set(ACTIVE_SENSE_DEFINITIONS ${PC_ACTIVE_SENSE_CFLAGS_OTHER})

option(ACTIVE_SENSE_USE_SHARED "Use libraries shared." ON)

if(WIN32)
	set(LIB_SUFFIX ".lib")
elseif(UNIX)
	set(LIB_SUFFIX ".a")
endif()

if(ACTIVE_SENSE_USE_SHARED)
	if(WIN32)
		set(LIB_SUFFIX ".lib")
	elseif(UNIX)
		set(LIB_SUFFIX ".so")
	endif()
endif()

if(NOT DEFINED ENV{ACTIVE_SENSE})
	message("Error: Environment variable ACTIVE_SENSE not found! \nPlease set ACTIVE_SENSE environment variable to the path where ActiveSense project is located.")
	return()
else()
	message("ACTIVE_SENSE_ENV_VAR: " $ENV{ACTIVE_SENSE})
endif()

set(ACTIVE_SENSE_INCLUDE_DIR "NOTFOUND")
find_path(ACTIVE_SENSE_INCLUDE_DIR ActiveSense/active_sense_config.h
          HINTS $ENV{ACTIVE_SENSE}/include ${PC_ACTIVE_SENSE_INCLUDEDIR} ${PC_ACTIVE_SENSE_INCLUDE_DIRS})

set(ACTIVE_AO_SENSE_LIBRARY "NOTFOUND")
# AO = Angle OcTree
find_library(ACTIVE_AO_SENSE_LIBRARY NAMES libangle_octree${LIB_SUFFIX} angle_octree${LIB_SUFFIX}
             HINTS $ENV{ACTIVE_SENSE}/build/ $ENV{ACTIVE_SENSE}/build/Release $ENV{ACTIVE_SENSE}/bin $ENV{ACTIVE_SENSE}/lib ${PC_ACTIVE_SENSE_LIBDIR} ${PC_ACTIVE_SENSE_LIBRARY_DIRS} )

set(ACTIVE_AS_SENSE_LIBRARY "NOTFOUND")
# AS = ActiveSense
find_library(ACTIVE_AS_SENSE_LIBRARY NAMES libactive_sense${LIB_SUFFIX} active_sense${LIB_SUFFIX}
             HINTS  $ENV{ACTIVE_SENSE} $ENV{ACTIVE_SENSE}/build $ENV{ACTIVE_SENSE}/build/ $ENV{ACTIVE_SENSE}/build/Release $ENV{ACTIVE_SENSE}/bin $ENV{ACTIVE_SENSE}/lib ${PC_ACTIVE_SENSE_LIBDIR} ${PC_ACTIVE_SENSE_LIBRARY_DIRS})

message("Found Libraries: ${ACTIVE_AO_SENSE_LIBRARY} ${ACTIVE_AS_SENSE_LIBRARY}")


set(ACTIVE_SENSE_LIBRARIES "${ACTIVE_AO_SENSE_LIBRARY};${ACTIVE_AS_SENSE_LIBRARY}" )

set(ACTIVE_SENSE_INCLUDE_DIR ${ACTIVE_SENSE_INCLUDE_DIR})
set(ACTIVE_SENSE_INCLUDE_DIRS ${ACTIVE_SENSE_INCLUDE_DIR})
message("ACTIVE_SENSE_INCLUDE_DIRS:" ${ACTIVE_SENSE_INCLUDE_DIRS})

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set ACTIVE_SENSE_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(Active_Sense  DEFAULT_MSG
                                  ACTIVE_AO_SENSE_LIBRARY ACTIVE_AS_SENSE_LIBRARY ACTIVE_SENSE_INCLUDE_DIR)

mark_as_advanced(ACTIVE_SENSE_INCLUDE_DIR ACTIVE_SENSE_LIBRARY)

#-fPIC -std=c++11	
#set(ACTIVE_SENSE_CXX_FLAGS "-std=c++0x -pthread")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${ACTIVE_SENSE_CXX_FLAGS}")
