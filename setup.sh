#!/bin/sh
source /opt/ros/groovy/setup.bash
source ../../devel/setup.bash
kbotDir=`pwd`

alias catkin_make='catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python2 -DPYTHON_INCLUDE_DIR=/usr/include/python2.7 -DPYTHON_LIBRARY=/usr/lib/libpython2.7.so'
alias catkin_make_isolated='catkin_make_isolated -DPYTHON_EXECUTABLE=/usr/bin/python2 -DPYTHON_INCLUDE_DIR=/usr/include/python2.7 -DPYTHON_LIBRARY=/usr/lib/libpython2.7.so'

function build {
	pushd .
	cd ${kbotDir}/../../
	catkin_make
	popd
}
