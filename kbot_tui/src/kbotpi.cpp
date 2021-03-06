#include <iostream>
#include "config.h"
#include <string>
#include "dbg.h"
#include <gflags/gflags.h>
#include "kbotpi.h"
#include "utils.h"

using namespace std;

DEFINE_bool(crash, false, "Crash faster than usual."); 

UI ui;
Network network;

int main( int argc, char* argv[] ){
	string version = util::format("%u.%u.%u.%u\n", VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH, VERSION_TWEAK);

	gflags::SetVersionString(version);
  gflags::SetUsageMessage("Write something clever here.");
	gflags::ParseCommandLineFlags(&argc, &argv, true);
	
	if(FLAGS_crash){
		int *p = 0;
		*p = 42;
	}

	ui.start();
	network.start(argc, argv);

	ui.join();
  network.stop();
	network.join();
	cout << "Main exit\n";
}


