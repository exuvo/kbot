
#include <algorithm>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "serial.h"
//#include "dbg.h"
#include "parser.h"

using namespace std;

void parse(Message* m){



}

M_Type toMType(uint8_t id){
	switch(id) {
		case 0: return M_Type::Ping;
		case 1: return M_Type::Power;
		case 2: return M_Type::Sonar;
		case 3: return M_Type::Tracks;
		case 4: return M_Type::Dome;
		case 5: return M_Type::Console;
		case 6: return M_Type::Text;
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
		case M_Type::Console: return 5;
		case M_Type::Text: 		return 6;
	}
}

void Message::calcChecksum(){
	for(int i=0; i< length; i++){
		checksum += data[i];
	}
}
