#!/bin/sh

cdbin(){
		if [ ! -e bin ]; then
			mkdir bin
		fi

		cd bin
}

A=$1
shift 1

case "$A" in
	clean)
		echo "Cleaning.."
		rm -rf bin
		;;
	build)
		cdbin
		cmake .. $*	&& make
		exit $?
		;;
	run)
		./bin/kbotpi $*
		exit $?
		;;
	all)
		$0 clean &&	$0 build $* && $0 run
		exit $?
		;;
	-h|--help)
		echo "TODO: print help here"
		exit 1
		;;
	cmake)
		cdbin
		cmake .. $*
		exit $?
		;;
	make)
		cdbin
		make $*
		exit $?
		;;
	*)
		$0 build $*
		exit $?
		;;
esac

exit 0
