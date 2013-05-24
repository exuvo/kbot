#include "serial.h"
#include "ros/ros.h"
#include "parser.h"

using namespace std;

serial::Serial port;
unsigned int _pos;
Message* _msg;

bool open(string portname, int baudrate){
	port.setPort(portname);
	port.setBaudrate(baudrate);
	serial::Timeout t = serial::Timeout::simpleTimeout(1000);
	port.setTimeout(t);
	port.open();
	return port.isOpen();
}

serial::Serial& getSerial(){
	return port;
}

void checkConnection(){
	if(!port.isOpen()){
		ROS_WARN_THROTTLE(10, "Serial port closed, reopening");
		port.open();
	}
}

void receive(){
	checkConnection();
	
	size_t amount;
	while(amount=port.available() > 0){
		if(_pos > 0){
			amount = min((unsigned int)amount, _msg->length - _pos + 4);
			port.read(&(_msg->data[_pos]), amount);
		
		} else if(amount >= 4){
			uint8_t b;
			port.read(&b, 1);

			if(b != START_BYTE){
				ROS_WARN_THROTTLE(1, "Parser: Expected start byte, got this instead: %u", b);
				return;
			}
			
			uint8_t d[3];
			port.read(d, 3);
			uint16_t len = d[0] << 8 | d[1];
			_msg = new Message(len, toMType(d[2]));
			_pos = 3;
		}
		
	}
}

void transmit(){
	checkConnection();

}
