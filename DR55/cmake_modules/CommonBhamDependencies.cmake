
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


####### List of Golem Components ==> Lib Names ######
#-Ctrl 		============> GolemCtrl
#-Defs  	============> GolemDefs
#-Demo 		============> ____
#-Planner   ============> ____
#-Math		============> GolemMath
#-Phys 		============> GolemPhys
#-Plan 		============> GolemPlan
#-Sys 		============> GolemSys
#-Tiny 		============> ____
#-TinyIce 	============> ____
#-Tools		============> GolemTools
#-UI 		============> GolemUI
#-UICtrl	============> GolemUICtrl


# TODO: add remaining libraries to GOLEM_LIBRARIES
SET(GOLEM_LIBRARIES    optimized GolemDefs${CMAKE_RELEASE_POSTFIX} debug GolemDefs${CMAKE_DEBUG_POSTFIX}
					   optimized GolemMath${CMAKE_RELEASE_POSTFIX} debug GolemMath${CMAKE_DEBUG_POSTFIX}
					   optimized GolemSys${CMAKE_RELEASE_POSTFIX} debug GolemSys${CMAKE_DEBUG_POSTFIX}
					   optimized GolemTools${CMAKE_RELEASE_POSTFIX} debug GolemTools${CMAKE_DEBUG_POSTFIX}
					   optimized GolemCtrl${CMAKE_RELEASE_POSTFIX} debug GolemCtrl${CMAKE_DEBUG_POSTFIX}
					   optimized GolemPlanner${CMAKE_RELEASE_POSTFIX} debug GolemPlanner${CMAKE_DEBUG_POSTFIX}
					   optimized GolemUI${CMAKE_RELEASE_POSTFIX} debug GolemUI${CMAKE_DEBUG_POSTFIX}
					   optimized GolemUICtrl${CMAKE_RELEASE_POSTFIX} debug GolemUICtrl${CMAKE_DEBUG_POSTFIX}
					   optimized GolemCtrlSM${CMAKE_RELEASE_POSTFIX} debug GolemCtrlSM${CMAKE_DEBUG_POSTFIX}
					   optimized GolemCtrlSingleCtrl${CMAKE_RELEASE_POSTFIX} debug GolemCtrlSingleCtrl${CMAKE_DEBUG_POSTFIX}
			           optimized GolemCtrlMultiCtrl${CMAKE_RELEASE_POSTFIX} debug GolemCtrlMultiCtrl${CMAKE_DEBUG_POSTFIX}
    )

####### List of Grasp Components ==> Lib Names ######

#-DAQFT/daqft 	
#-Grasp
#-----ActiveCtrl ======> GraspActiveCtrl	 
#--------ActiveCtrl 	============> ____
#--------ArmHandForce 	============> GraspArmHandForce
#--------JointCtrl 		============> ____
#--------OspaceCtrl 	============> ____
#--------WorkspaceCtrl	============> ____
#-----App 		 ======> GraspApp
#-----Contact 	 ======>
#-----Core ============> GraspCore
#-----Data ============>
#--------ContactModel 	============> GraspContact
#--------ContactModelUA ============> ____	
#--------ContactQuery 	============> ____
#--------ContactQueryUA ============> ____
#--------Image          ============> GraspDataImage
#--------Part3DHoPCloud ============> ____
#--------PointsCurv 	============> GraspDataPointsCurv
#--------Trajectory 	============> ____
#--------TrajectoryUA	============> ____
#--------Video
#-----Demo ============>
#-----Sensor ==========>
#-----Tiny ============>
#-TactileToolbox 
#-WTS/wts-ft 
#-camcalb
#-----calb ============> CamcalbCalb
#-----matas ===========> CamcalbMatas

# TODO: i) Add remaining libraries to GRASP_LIBRARIES
#      ii) Organise both golem and grasp into components like the PCL does







			#NxCharacter64 PhysXCooking64 PhysXCore64 PhysXLoader64
#			expat freeglut Gdiplus

 
#			${PCL_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS}

SET(GRASP_LIBRARIES 
			optimized GraspCore${CMAKE_RELEASE_POSTFIX} debug GraspCore${CMAKE_DEBUG_POSTFIX}
			optimized GraspContact${CMAKE_RELEASE_POSTFIX} debug GraspContact${CMAKE_DEBUG_POSTFIX}
			optimized GraspApp${CMAKE_RELEASE_POSTFIX} debug GraspApp${CMAKE_DEBUG_POSTFIX}
			optimized GraspActiveCtrl${CMAKE_RELEASE_POSTFIX} debug GraspActiveCtrl${CMAKE_DEBUG_POSTFIX}
            optimized GraspDataPointsCurv${CMAKE_RELEASE_POSTFIX} debug GraspDataPointsCurv${CMAKE_DEBUG_POSTFIX}
			optimized GraspDataImage${CMAKE_RELEASE_POSTFIX} debug GraspDataImage${CMAKE_DEBUG_POSTFIX}
			optimized GraspArmHandForce${CMAKE_RELEASE_POSTFIX} debug GraspArmHandForce${CMAKE_DEBUG_POSTFIX}
			optimized GraspJointCtrl${CMAKE_RELEASE_POSTFIX} debug GraspJointCtrl${CMAKE_DEBUG_POSTFIX}
            ${CMAKE_DL_LIBS}
			optimized CamcalbCalb${CMAKE_RELEASE_POSTFIX} debug CamcalbCalb${CMAKE_DEBUG_POSTFIX}
			optimized CamcalbMatas${CMAKE_RELEASE_POSTFIX} debug CamcalbMatas${CMAKE_DEBUG_POSTFIX}		 
#                                         debug GraspWorkspaceCtrl${CMAKE_DEBUG_POSTFIX}
#                                         optimized GraspJointCtrl${CMAKE_RELEASE_POSTFIX}
#                                         debug GraspJointCtrl${CMAKE_DEBUG_POSTFIX}
 )


# TODO: Complete GOLEM_GRASP_3RD_PARTY_LIBRARIES
SET(GOLEM_GRASP_3RD_PARTY_LIBRARIES ${Boost_LIBRARIES} 
									${PCL_LIBRARIES} 
									${OpenCV_LIBS} 										 
									expat freeglut Gdiplus)

### Extra dependencies (if needed) ###
#  ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS} (WIN32/UNIX)
#  ${CMAKE_DL_LIBS} (UNIX)
#  GL GLU glut (UNIX)
#  expat freeglut Gdiplus (WIN32)
#  NxCharacter NxCooking PhysXCore PhysXLoader (UNIX)
#  NxCharacter64 PhysXCooking64 PhysXCore64 PhysXLoader64 (WIN32)

## Linking e.g. ###

#target_link_libraries(TARGET optimized CamcalbCalb${CMAKE_RELEASE_POSTFIX} 
#							  debug CamcalbCalb${CMAKE_DEBUG_POSTFIX})

### Extra info about linking ###

# TARGET_LINK_LIBRARIES()'s debug/optimized/general keywords apply to
# their following argument only: 'A "debug", "optimized", or "general"
# keyword indicates that the library immediately following it is to be
# used only for the corresponding build configuration.' [Documentation
# of TARGET_LINK_LIBRARIES()].


