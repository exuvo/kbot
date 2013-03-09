#include <iostream>
#include "config.h"
#include <string>
#include "dbg.h"
#include <gflags/gflags.h>
#include "ui/ui.h"
#include "utils.h"

DEFINE_bool(crash, false, "Crash faster than usual."); 

UI ui;

int main( int argc, char* argv[] ){
	std::string version = util::format("%u.%u.%u.%u\n", VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH, VERSION_TWEAK);

	google::SetVersionString(version);
  google::SetUsageMessage("Write something clever here.");
	google::ParseCommandLineFlags(&argc, &argv, true);
	
	if(FLAGS_crash){
		int *p = 0;
		*p = 42;
	}

	ui.start();



	ui.join();
	std::cout << "Main exit\n";
}


