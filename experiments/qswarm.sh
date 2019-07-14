#!/bin/bash
# ================================================================================
# Created by: Patka Zsolt-András
# On: 13.07.2019
# This script makes it easier to clean, build, debug and run the ARGoS experiment
# 
# Arguments:$1	The job to execute, can be:
#					build 		builds the project
#					run			runs the experiment
#					build-run   builds the project and runs the experiment	
#					build-debug builds the project and stars the experiment in
#								debug mode
#			$2	The name of the scene
#
# build - requires no $2
# ================================================================================

# Subtasks implemented as functions


# Clean subtask
function f_clean {
	rm -r build
	mkdir build
}

# Build subtask
function f_build {
	f_clean
	cd build
	cmake -DCMAKE_BUILD_TYPE=Release ..
	make
}

# Run subtask
function f_run {
	isBuild=$(pwd | grep build)
	if ! [ -z $isBuild ]; then
		cd ..
	fi
	
	argos3 -c $scene_name
}

# Debug subtask
function f_build_debug {
	f_clean
	cd build
	cmake -DCMAKE_BUILD_TYPE=Debug ..
	make
	f_run
}

# ================================================================================
if (( $# < 1 || $# > 2 )); then # Invalid number of arguments
	echo "Invalid usage. Correct usage: qswarm <job> [<scene_name>]"
	exit 1;
fi

job=$1
scene_name="scenes/$2.argos"

if [ "$1" != "build" ]; then
		if ! [ -a $scene_name ]; then
			echo "Could not find the scene: $scene_name"
			exit 1;
		fi
	elif ! [ -z $2 ]; then
		echo "Invalid usage. Correct usage: qswarm build"
		exit 1;
fi

case $job in
	build) 
		f_build
		;;
	run)
		f_run
		;;
	build-run)
		f_build
		f_run
		;;
	build-debug)
		f_build_debug
		;;
	*) echo "Incorrect job, available jobs: build, run, build-run and build-debug"
esac

exit 0;