cmake_minimum_required(VERSION 2.8.3)
project(kbot_tui)

#set(CMAKE_VERBOSE_MAKEFILE ON)

# disable debug output
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DNDEBUG")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation geometry_msgs sensor_msgs kbot_bridge)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Gflags REQUIRED)

set(CURSES_USE_NCURSES true)
find_package(Curses REQUIRED)
#FIND_PACKAGE(Panel REQUIRED)
#FIND_PACKAGE(Menu REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/groovy/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
#add_message_files(
#  FILES
#)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
	geometry_msgs
	sensor_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES kbot_bridge
  CATKIN_DEPENDS message_runtime kbot_bridge
#  DEPENDS system_lib
)

###########
## Build ##
###########

set (kbotpi_VERSION_MAJOR 0)
set (kbotpi_VERSION_MINOR 0)
set (kbotpi_VERSION_PATCH 4)
set (kbotpi_VERSION_TWEAK 2)

#configure a header file to pass some of the CMake settings to the source code
configure_file (
	"${PROJECT_SOURCE_DIR}/config.h.in"
	"${PROJECT_BINARY_DIR}/config.h"
)

MACRO(HEADER_DIRECTORIES return_list)
    FILE(GLOB_RECURSE new_list src/*.h src/*.hpp)
    SET(dir_list "")
    FOREACH(file_path ${new_list})
        GET_FILENAME_COMPONENT(dir_path ${file_path} PATH)
        SET(dir_list ${dir_list} ${dir_path})
    ENDFOREACH()
    LIST(REMOVE_DUPLICATES dir_list)
    SET(${return_list} ${dir_list})
ENDMACRO()

# all our files.
FILE (GLOB_RECURSE project_SRCS src/*.cpp src/*.cxx src/*.cc src/*.C src/*.c)
FILE (GLOB_RECURSE project_HEADERS src/*.h src/*.hpp)
HEADER_DIRECTORIES(project_HEADERS_DIRS)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${Gflags_INCLUDE_DIRS}
	${PROJECT_BINARY_DIR}
	${project_HEADERS_DIRS}
)

## Declare a cpp library
# add_library(kbot_bridge
#   src/${PROJECT_NAME}/kbot_bridge.cpp
# )

## Declare a cpp executable
add_executable(kbot_tui ${source} ${project_SRCS})

set_target_properties(kbot_tui PROPERTIES COMPILE_FLAGS "-std=c++11 -O2 -g -Wall -Wextra")

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(
  kbot_tui
  kbot_tui_generate_messages_cpp
  kbot_tui_gencpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(kbot_tui
  ${catkin_LIBRARIES}
  gflags
	ncurses
	menu
	panel
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/groovy/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS kbot_bridge kbot_bridge_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_kbot_bridge.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
