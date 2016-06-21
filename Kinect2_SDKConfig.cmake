#  =============================================================================
#  Kinect2 SDK configuration (for Windows)
#
#  This package will search for the Kinect2 SDK from Microsoft installed on
#  your Windows. It will copy necessary files (dll and faces models) to the
#  project folder (for face detection and easy debugging).
#
#  Usage from an external project:
#    In your CMakeLists.txt, add these lines:
#
#    set(Kinect2_SDK_DIR PATH_TO_THIS_FILE)
#	 find_package( Kinect2_SDK REQUIRED )
#
#    PATH_TO_THIS_FILE represents the path to reach this configuration file
#
#    If the module is found then Kinect2_SDK_FOUND is set to TRUE.
#
#    This file will define the following variables:
#      - Kinect2_SDK_ROOT_PATH    : The Kinect2 SDK root folder.
#      - Kinect2_SDK_INCLUDE_DIRS : The Kinect2 SDK include directories.
#      - Kinect2_SDK_LIBRARY_DIRS : The Kinect2 SDK library directories.
#      - Kinect2_SDK_LIBS         : The Kinect2 SDK librairies to link with.
#
#  =============================================================================

set(Kinect2_SDK_FOUND FALSE CACHE BOOL "Kinect2_SDK found")
set(Kinect2_SDK_ROOT_PATH "UNK" CACHE PATH "Kinect2_SDK root PATH")

if ( WIN32 OR WIN64 )
	set(Kinect2_SDK_ConfigNotDone TRUE)
	if ( EXISTS "$ENV{KinectSDK20_DIR}" )
		set(Kinect2_SDK_ConfigNotDone FALSE)
		set(Kinect2_SDK_FOUND TRUE CACHE BOOL "Kinect2_SDK found" FORCE)
		set(Kinect2_SDK_ROOT_PATH "$ENV{KinectSDK20_DIR}" CACHE PATH "Kinect2_SDK root PATH" FORCE)
		
		MESSAGE( STATUS "Found Kinect2 SDK in " ${Kinect2_SDK_ROOT_PATH})

		if ( EXISTS "${Kinect2_SDK_ROOT_PATH}/inc" )
			set(Kinect2_SDK_INCLUDE_DIRS "${Kinect2_SDK_ROOT_PATH}/inc")
		else()
			set(Kinect2_SDK_ConfigNotDone TRUE)
			MESSAGE( WARNING "Missing Kinect2 SDK include folder" )
		endif()

		if( EXISTS "${Kinect2_SDK_ROOT_PATH}/lib/x64/" )
			set(Kinect2_SDK_LIBRARY_DIRS "${Kinect2_SDK_ROOT_PATH}/lib/x64/")
		else()
			set(Kinect2_SDK_ConfigNotDone TRUE)
			MESSAGE( WARNING "Missing Kinect2 SDK library folder" )
		endif()

		set(Kinect2_SDK_LIBS "")
		if ( EXISTS "${Kinect2_SDK_LIBRARY_DIRS}Kinect20.lib" )
			list(APPEND Kinect2_SDK_LIBS "Kinect20.lib")
		else()
			set(Kinect2_SDK_ConfigNotDone TRUE)
			MESSAGE( WARNING "Missing Kinect20.lib in Kinect2 SDK library folder" )
		endif()

		if ( EXISTS "${Kinect2_SDK_LIBRARY_DIRS}Kinect20.Face.lib" )
			list(APPEND Kinect2_SDK_LIBS "Kinect20.Face.lib")
		else()
			set(Kinect2_SDK_ConfigNotDone TRUE)
			MESSAGE( WARNING "Missing Kinect20.Face.lib in Kinect2 SDK library folder" )
		endif()
		
		if ( NOT ${Kinect2_SDK_ConfigNotDone} )
			MESSAGE( STATUS "Copy ${Kinect2_SDK_ROOT_PATH}/Redist/Face/x64/ to project folder" )
			FILE(GLOB Kinect2_SDK_FilesToCopy "${Kinect2_SDK_ROOT_PATH}/Redist/Face/x64/*")
			# Copy file to destination
			FILE(COPY ${Kinect2_SDK_FilesToCopy} DESTINATION ".")
		endif()

	endif()

	if ( ${Kinect2_SDK_ConfigNotDone} )
		MESSAGE( WARNING "Fall back to manual configuration. THIS MAY NOT WORK" )
		set(Kinect2_SDK_FOUND TRUE CACHE BOOL "Kinect2_SDK found" FORCE)
		set(Kinect2_SDK_ROOT_PATH "C:/Program Files/Microsoft SDKs/Kinect/v2.0_1409/Lib/x64/" CACHE PATH "Kinect2_SDK root PATH" FORCE)
		set(Kinect2_SDK_INCLUDE_DIRS "${Kinect2_SDK_ROOT_PATH}/inc")
		set(Kinect2_SDK_LIBRARY_DIRS "${Kinect2_SDK_ROOT_PATH}/lib/x64/")
		set(Kinect2_SDK_LIBS "")
		list(APPEND Kinect2_SDK_LIBS "Kinect20.lib" "Kinect20.Face.lib")
	endif()
else()
	MESSAGE(FATAL_ERROR This Kinect2_SDK search is design for Windows Kinect2 SDK)
endif()

