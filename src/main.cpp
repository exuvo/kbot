#include <stdio.h>
#include "config.h"
#include <string.h>
#include "dbg.h"
#include <gflags/gflags.h>
#include "ui/ui.h"

DEFINE_bool(crash, false, "Crash faster than usual."); 

int main( int argc, char* argv[] ){
	printf("kbotpi version %u.%u\n", VERSION_MAJOR, VERSION_MINOR);
 	debug("argc=%d", argc);

	google::ParseCommandLineFlags(&argc, &argv, true);
	
	if(FLAGS_crash){
		int *p = 0;
		*p = 42;
	}

	new UI();
	std::cout << "Main exit" << endl;
}
