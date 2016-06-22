cmake_minimum_required (VERSION 2.8)

project (RecordKinect)

include(../CMakeModule/DomsModule.cmake)

set(Omiscid_MODULES "Messaging")
set(Omiscid_DIR "../Omiscid/")
find_package( Omiscid REQUIRED COMPONENTS Messaging)

if ( MSVC )
	foreach(flag_var
	        CMAKE_CXX_FLAGS CMAKE_CXX_FLAGS_DEBUG CMAKE_CXX_FLAGS_RELEASE
	        CMAKE_CXX_FLAGS_MINSIZEREL CMAKE_CXX_FLAGS_RELWITHDEBINFO)
	   if (${flag_var} MATCHES "/MD")
	      string(REGEX REPLACE "/MD" "/MT" ${flag_var} "${${flag_var}}")
	   endif()
	endforeach(flag_var)
	
	# Add performance profil to Release compilation
	AddFlag( CMAKE_CXX_FLAGS_RELEASE "/O2" )
	AddFlag( CMAKE_CXX_FLAGS_RELEASE "/Ob2" )
	AddFlag( CMAKE_CXX_FLAGS_RELEASE "/Oi" )
	AddFlag( CMAKE_CXX_FLAGS_RELEASE "/Ot" )
	AddFlag( CMAKE_CXX_FLAGS_RELEASE "/Oy" )
	AddFlag( CMAKE_CXX_FLAGS_RELEASE "/GL" )
	# MESSAGE ( ${CMAKE_CXX_FLAGS_RELEASE} )
	
	# add flag to remove warning about usage of non secure versions of fopen, fprintf...
	add_definitions(-D_CRT_SECURE_NO_WARNINGS)

	# Multi processor flag
	AddFlag( CMAKE_CXX_FLAGS_DEBUG "/MP" )
	AddFlag( CMAKE_CXX_FLAGS_RELEASE "/MP" )
endif()


find_package( OpenCV REQUIRED )

set(Kinect2_SDK_DIR "../")
find_package( Kinect2_SDK REQUIRED )

set(CWD ${CMAKE_CURRENT_SOURCE_DIR})
set(PARENTWD ${CMAKE_CURRENT_SOURCE_DIR}/..)

add_definitions(-DKINECT_2 -DKINECT_LIVE -DACTIVATE_KINECT_DRAWING)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${Kinect2_SDK_INCLUDE_DIRS})
include_directories("${PARENTWD}/")
include_directories("${PARENTWD}/Kinect/")
include_directories("${PARENTWD}/Drawing/")

# Add main files from the project
file(GLOB SRCS RELATIVE ${CWD}
	"${CWD}/*.cpp"
)

# Add drawing files
file(GLOB DRAWING RELATIVE ${CWD}
	"${PARENTWD}/Drawing/Drawable.cpp"
	"${PARENTWD}/Drawing/DrawRawData.cpp"
	"${PARENTWD}/Drawing/DrawCameraView.cpp"
	"${PARENTWD}/Drawing/DrawDepthView.cpp"
)

# Add mandatory files for the Kinect folder
file(GLOB KINECT RELATIVE ${CWD} 
	"${PARENTWD}/Kinect/KinectBasics.cpp"
	"${PARENTWD}/Kinect/KinectBody.cpp"
	"${PARENTWD}/Kinect/KinectDataAsMemoryBuffer.cpp"
	"${PARENTWD}/Kinect/KinectFaceStream.cpp"
	"${PARENTWD}/Kinect/KinectRGBStream.cpp"
	"${PARENTWD}/Kinect/KinectDraw.cpp"
	"${PARENTWD}/Kinect/KinectFace.cpp"
	"${PARENTWD}/Kinect/KinectImageConverter.cpp"
	"${PARENTWD}/Kinect/KinectSensor.cpp"
	"${PARENTWD}/Kinect/KinectSensor.cpp"
	"${PARENTWD}/Kinect/KinectSensor-v2.cpp"
	"${PARENTWD}/Kinect/RecordingContextFactory.cpp"
	"${PARENTWD}/Kinect/RecordingManagement.cpp"
	"${PARENTWD}/Kinect/KinectFacestream.cpp"
	"${PARENTWD}/Kinect/KinectAudioStream.cpp"
	"${PARENTWD}/Kinect/KinectRGBStream.cpp"
)

# And for Geometric map
AddSourceInGroup(${CWD} "${PARENTWD}/Map" "MAP")

file(GLOB HDRS RELATIVE ${CWD}
	"${PARENTWD}/Drawing/*.h"
	"${PARENTWD}/Kinect/*.h"
)

link_directories(${Kinect2_SDK_LIBRARY_DIRS})
add_executable(RecordKinect Main.cpp ${SRCS} ${HDRS} ${Omiscid_SRCS} ${Omiscid_HDRS} ${KINECT})
target_link_libraries(RecordKinect ${Kinect2_SDK_LIBS})
target_link_libraries(RecordKinect ${OpenCV_LIBS})
target_link_libraries(RecordKinect ws2_32.lib)

# Microsoft specific case, no effect on other systems
source_group("Omiscid\\System" FILES ${OmiscidSystem_SRCS} ${OmiscidSystem_HDRS})
source_group("Omiscid\\Messaging" FILES ${OmiscidMessaging_SRCS} ${OmiscidMessaging_HDRS})
source_group("Kinect" FILES ${KINECT})
source_group("Drawing" FILES ${DRAWING})
