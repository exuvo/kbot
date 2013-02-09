#include <stdio.h>
#include "config.h"
#include <string.h>
#include "dbg.h"
#include <gflags/gflags.h>

DEFINE_bool(crash, false, "Crash quickly."); 

int main( int argc, char* argv[] ){
	printf("kbotpi version %u.%u\n", VERSION_MAJOR, VERSION_MINOR);
 	debug("argc=%d", argc);

	google::ParseCommandLineFlags(&argc, &argv, true);
	
	if(FLAGS_crash){
		int *p = 0;
		*p = 42;
	}
}
