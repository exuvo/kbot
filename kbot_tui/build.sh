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
	clean|c)
		echo "Cleaning.."
		rm -rf bin
		;;
	build|b)
		cdbin
		cmake .. $*	&& make
		exit $?
		;;
	run|r)
		./bin/kbotpi $* 2> ./bin/error
		ret=$?
		cat ./bin/error
		exit $ret
		;;
	valgrind|v)
		valgrind --leak-check=full ./bin/kbotpi $* 2> ./bin/error
		ret=$?
		cat ./bin/error
		exit $ret
		;;
	all|a)
		$0 clean &&	$0 build $* && $0 run
		exit $?
		;;
	-h|--help|help)
		echo "$0 [option] [...]
    b[uild] c[lean] r[un] a[ll] cm[ake] m[ake] v[algrind]
    Normal usage: $0 c |$0 br | $0 bv"
		exit 1
		;;
	cmake|cm)
		cdbin
		cmake .. $*
		exit $?
		;;
	make|m)
		cdbin
		make $*
		exit $?
		;;
	buildrun|build-run|br)
		$0 build && $0 run $*
		exit $?
		;;
		bv)
		$0 build && $0 valgrind $*
		exit $?
		;;
	*)
		$0 build $*
		exit $?
		;;
esac

exit 0
