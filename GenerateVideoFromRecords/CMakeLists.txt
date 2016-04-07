cmake_minimum_required (VERSION 2.8)

project (GenerateVideoFromRecords)

if ( MSVC )
	foreach(flag_var
	        CMAKE_CXX_FLAGS CMAKE_CXX_FLAGS_DEBUG CMAKE_CXX_FLAGS_RELEASE
	        CMAKE_CXX_FLAGS_MINSIZEREL CMAKE_CXX_FLAGS_RELWITHDEBINFO)
	   if (${flag_var} MATCHES "/MD")
	      string(REGEX REPLACE "/MD" "/MT" ${flag_var} "${${flag_var}}")
	   endif(${flag_var} MATCHES "/MD")
	endforeach(flag_var)
	
	# add flag to remove warning about usage of non secure versions of fopen, fprintf...
	add_definitions(-D_CRT_SECURE_NO_WARNINGS)
endif()

find_package( OpenCV REQUIRED )

if(CMAKE_COMPILER_IS_GNUCXX)
    add_definitions(-std=c++11)
endif()

set(CWD ${CMAKE_CURRENT_SOURCE_DIR})
set(PARENTWD ${CMAKE_CURRENT_SOURCE_DIR}/..)

add_definitions(-DKINECT_2 -DACTIVATE_KINECT_DRAWING -D_FILE_OFFSET_BITS=64)

function(AddSourceInGroup MainFolder cpp_folder GroupName)
	# Add cpp data
	# Message( STATUS ${cpp_folder} )
	file(GLOB_RECURSE CPP_SRCS RELATIVE ${MainFolder} "${cpp_folder}/*.cpp")
	# source_group(${GroupName} FILES ${CPP_SRCS})
	set(${GroupName} ${${GroupName}} ${CPP_SRCS} PARENT_SCOPE)
endfunction(AddSourceInGroup)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories("${PARENTWD}/Omiscid/Json/")
include_directories("${PARENTWD}/Omiscid/System/")
include_directories("${PARENTWD}/Omiscid/Messaging/")
include_directories("${PARENTWD}/Map/")
include_directories(${PARENTWD})

# Add main files from the project
file(GLOB SRCS RELATIVE ${CWD} ${CWD})

# Add Omiscid files
AddSourceInGroup(${CWD} "${PARENTWD}/Omiscid/System" "OMISCID_SYSTEM")
AddSourceInGroup(${CWD} "${PARENTWD}/Omiscid/Json" "OMISCID_JSON")
AddSourceInGroup(${CWD} "${PARENTWD}/Omiscid/Messaging" "OMISCID_MESSAGING")


# Add Drawing and DataManagement files
AddSourceInGroup(${CWD} "${PARENTWD}/Drawing" "DRAWING")
AddSourceInGroup(${CWD} "${PARENTWD}/DataManagement" "DATAMANAGEMENT")

# Add mandatory files for the Kinect folder
file(GLOB_RECURSE KINECT RELATIVE ${CWD} 
	"${PARENTWD}/Kinect/KinectBasics.cpp"
	"${PARENTWD}/Kinect/KinectBody.cpp"
	"${PARENTWD}/Kinect/KinectDataAsMemoryBuffer.cpp"
	"${PARENTWD}/Kinect/KinectFace.cpp"
	"${PARENTWD}/Kinect/KinectImageConverter.cpp"
)


# And for Geometric map
AddSourceInGroup(${CWD} "${PARENTWD}/Map" "MAP")

file(GLOB_RECURSE HDRS RELATIVE ${CWD}
	"${PARENTWD}/Omiscid/System/System/*.h"
	"${PARENTWD}/Omiscid/Messaging/Messaging/*.h"
	"${PARENTWD}/Omiscid/Json/Json/*.h"
	"${PARENTWD}/Drawing/*.h"
	"${PARENTWD}/Kinect/*.h"
	"${PARENTWD}/DataManagement/*.h"
 	"${PARENTWD}/Map/*.hpp"
)

add_executable(GenerateVideoFromRecords Main.cpp ${SRCS} ${HDRS} ${OMISCID_SYSTEM} ${OMISCID_JSON} ${OMISCID_MESSAGING} ${DRAWING}  ${DATAMANAGEMENT} ${KINECT} ${MAP})

# Microsoft specific case, no effect on other systems
source_group("Omiscid\\System" FILES ${OMISCID_SYSTEM})
source_group("Omiscid\\Json" FILES ${OMISCID_JSON})
source_group("Omiscid\\Messaging" FILES ${OMISCID_MESSAGING})
source_group("Drawing" FILES ${DRAWING})
source_group("Datamanagement" FILES ${DATAMANAGEMENT})
source_group("Kinect" FILES ${KINECT})
source_group("Map" FILES ${MAP})

target_link_libraries(GenerateVideoFromRecords ${OpenCV_LIBS})
target_link_libraries(GenerateVideoFromRecords ${CMAKE_THREAD_LIBS_INIT})
