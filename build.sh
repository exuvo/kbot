#!/bin/sh

case "$1" in
	clean)
		echo "Cleaning.."
		rm -rf bin
    exit 0
		;;
	run)
		shift 1
		./bin/kbotpi $*
		exit $?
		;;
	-h|--help)
		echo "TODO: print help here"
		exit 1
		;;
esac

if [ ! -e bin ]; then
	mkdir bin
fi

cd bin
cmake .. $*
make
