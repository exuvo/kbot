cmake_minimum_required (VERSION 2.8)
project (kbotpi)
set (kbotpi_VERSION_MAJOR 0)
set (kbotpi_VERSION_MINOR 0)
set (kbotpi_VERSION_PATCH 4)
set (kbotpi_VERSION_TWEAK 2)

#Options
option (USE_OPENCV "Use opencv for vision things" OFF)
option (DEBUG "Produce extra debug info" ON)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

#Boost
SET(Boost_USE_MULTITHREAD ON) #set a flag
FIND_PACKAGE(Boost REQUIRED)
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})

#Gflags
FIND_PACKAGE(Gflags REQUIRED)
INCLUDE_DIRECTORIES(${Gflags_INCLUDE_DIRS})

if (USE_OPENCV)
	#OpenCV
	FIND_PACKAGE(OpenCV REQUIRED)
	INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIR})
endif (USE_OPENCV)

SET(CURSES_USE_NCURSES TRUE)
FIND_PACKAGE(Curses REQUIRED)
#FIND_PACKAGE(Panel REQUIRED)
#FIND_PACKAGE(Menu REQUIRED)

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
#include_directories("${PROJECT_SOURCE_DIR}/src/lib/bstr")
#add_subdirectory(src/lib/bstr)

aux_source_directory(src srcRoot)
aux_source_directory(src/network srcNet)
aux_source_directory(src/ui srcUI)
aux_source_directory(src/map srcMap)
add_executable(kbotpi ${srcRoot} ${srcNet} ${srcUI} ${srcMap})
target_link_libraries(kbotpi gflags ncurses menu panel)
install(TARGETS kbotpi RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX})
