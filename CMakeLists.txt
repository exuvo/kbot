cmake_minimum_required (VERSION 2.6)
project (kbotpi)
set (kbotpi_VERSION_MAJOR 0)
set (kbotpi_VERSION_MINOR 0)

#Options
option (USE_OPENCV "Use opencv for vision things" OFF)

#Boost
SET(Boost_USE_MULTITHREAD ON) #set a flag
FIND_PACKAGE(Boost)
INCLUDE_DIRECTORIES(${INCLUDE_DIRECTORIES} ${Boost_INCLUDE_DIRS})
LINK_DIRECTORIES(${LINK_DIRECTORIES} ${Boost_LIBRARY_DIRS})

if (USE_OPENCV)
	#OpenCV
	FIND_PACKAGE(OpenCV REQUIRED )
	INCLUDE_DIRECTORIES( ${OPENCV_INCLUDE_DIR} )
	 
	ADD_EXECUTABLE(Scalar Scalar.cxx)
	TARGET_LINK_LIBRARIES(Scalar ${OpenCV_LIBS})
endif (USE_OPENCV)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -g -Wall")

#configure a header file to pass some of the CMake settings to the source code
configure_file (
	"${PROJECT_SOURCE_DIR}/config.h.in"
	"${PROJECT_BINARY_DIR}/config.h"
	)

#add the binary tree to the search path for include files so that we will find
# generated headers
include_directories("${PROJECT_BINARY_DIR}")
include_directories("${PROJECT_SOURCE_DIR}/src/lib/bstr")
add_subdirectory(src/lib/bstr)

add_executable(kbotpi src/main)
target_link_libraries(kbotpi bstr)
