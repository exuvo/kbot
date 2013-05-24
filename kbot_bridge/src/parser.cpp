
#include <algorithm>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "serial.h"
//#include "dbg.h"
#include "parser.h"

using namespace std;

void parseMessage(Message m){



}

M_Type toMType(uint8_t id){
	switch(id) {
		case 0: return M_Type::Ping;
		case 1: return M_Type::Power;
		case 2: return M_Type::Sonar;
		case 3: return M_Type::Tracks;
		case 4: return M_Type::Dome;
		default: throw logic_error(__FILE__ ": enum M_Type out of range");
	}
}

uint8_t fromMType(M_Type type){
	switch(type) {
		case M_Type::Ping: 		return 0;
		case M_Type::Power:	 	return 1;
		case M_Type::Sonar: 	return 2;
		case M_Type::Tracks: 	return 3;
		case M_Type::Dome: 		return 4;
	}
}
