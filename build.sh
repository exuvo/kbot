#!/bin/sh

case "$1" in
	clean)
		echo "Cleaning.."
		rm -rf bin
		;;
	run)
		shift 1
		./bin/kbotpi $*
		exit $?
		;;
	all)
		$0 clean
		$0 
		$0 run
		;;
	-h|--help)
		echo "TODO: print help here"
		exit 1
		;;
	*)
		if [ ! -e bin ]; then
			mkdir bin
		fi

		cd bin
		cmake .. $*
		make
		;;
esac

exit 0
