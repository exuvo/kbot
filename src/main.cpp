#include <stdio.h>
#include "config.h"
#include <string.h>
#include "dbg.h"
#include <gflags/gflags.h>

int main( int argc, const char* argv[] ){
	printf("kbotpi version %u.%u\n", VERSION_MAJOR, VERSION_MINOR);
 	debug("argc=%d", argc);
 
  for(int i=0; i < argc; i++){
		if(strcmp(argv[i], "--help")){

		}

	
	}
}
