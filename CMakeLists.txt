cmake_minimum_required (VERSION 2.8)
project (kbotpi)
set (kbotpi_VERSION_MAJOR 0)
set (kbotpi_VERSION_MINOR 0)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

#Options
option (USE_OPENCV "Use opencv for vision things" OFF)
option (DEBUG "Produce extra debug info" ON)

#Boost
SET(Boost_USE_MULTITHREAD ON) #set a flag
FIND_PACKAGE(Boost REQUIRED)
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})
LINK_DIRECTORIES(${Boost_LIBRARY_DIRS})

#Gflags
FIND_PACKAGE(Gflags REQUIRED)
INCLUDE_DIRECTORIES(${Gflags_INCLUDE_DIRS})
LINK_DIRECTORIES(${Gflags_LIBRARY_DIRS})

if (USE_OPENCV)
	#OpenCV
	FIND_PACKAGE(OpenCV REQUIRED)
	INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIR})
	LINK_DIRECTORIES(${OpenCV_LIBRARY_DIRS})
endif (USE_OPENCV)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -g -W -Wall -Wextra -pedantic -std=c++11")

if(DEBUG)
	#set(CMAKE_VERBOSE_MAKEFILE ON)
else(DEBUG)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DNDEBUG")
endif(DEBUG)

#configure a header file to pass some of the CMake settings to the source code
configure_file (
	"${PROJECT_SOURCE_DIR}/config.h.in"
	"${PROJECT_BINARY_DIR}/config.h"
	)

MACRO(HEADER_DIRECTORIES return_list)
    FILE(GLOB_RECURSE new_list *.h)
    SET(dir_list "")
    FOREACH(file_path ${new_list})
        GET_FILENAME_COMPONENT(dir_path ${file_path} PATH)
        SET(dir_list ${dir_list} ${dir_path})
    ENDFOREACH()
    LIST(REMOVE_DUPLICATES dir_list)
    SET(${return_list} ${dir_list})
ENDMACRO()

#add the binary tree to the search path for include files so that we will find generated headers
include_directories("${PROJECT_BINARY_DIR}")
HEADER_DIRECTORIES(includeList)
include_directories(${includeList})
#include_directories("${PROJECT_SOURCE_DIR}/src")
#include_directories("${PROJECT_SOURCE_DIR}/src/lib/bstr")
add_subdirectory(src/lib/bstr)

add_executable(kbotpi src/main)
target_link_libraries(kbotpi bstr gflags)
install(TARGETS kbotpi RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX})
