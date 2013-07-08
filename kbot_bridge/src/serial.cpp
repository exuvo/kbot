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
		try{
			port.open();
		} catch (const serial::IOException& ex){
			ROS_ERROR_THROTTLE(10, "IOException while attempting to reopen serial port: %s", ex.what());
		}
	}
}

void resetMessage(){
  delete _msg;
  _pos = 0;
}

void receive(){
  checkConnection();
  
  // reads:
  //  4 bytes:
  //     1 byte  to compare with START_BYTE
  //     2 bytes into _msg->len
  //     1 byte  into _msg->type
  //  len+1 bytes:
  //   len bytes into _msg->data
  //     1 byte  to compare with _msg->checksum

  
  size_t amount;
  while((amount=port.available()) > 0){
    if(_pos > 0){
      amount = min((unsigned int)amount, _msg->length + 4 - _pos);
      port.read(&(_msg->data[_pos]), amount);
      _pos += amount;

      if(_pos == _msg->length + 4 && port.available()){
        uint8_t checksum;
        port.read(&checksum, 1);
        _msg->calcChecksum();

        if(checksum == _msg->checksum){
          parse(_msg);
        }else{
          ROS_WARN_THROTTLE(1, "Serial: Checksum mismatch: %u != %u", checksum, _msg->checksum);
        }
        resetMessage();
      }
  
    }else if(amount >= 4){
      uint8_t b;
      port.read(&b, 1);

      if(b != START_BYTE){
        ROS_WARN_THROTTLE(1, "Serial: Expected start byte, got this instead: %u", b);
        return;
      }
      uint8_t d[3];
      port.read(d, 3);
      uint16_t len = d[0] << 8 | d[1];
      _msg = new Message(len, toMType(d[2])); // TODO catch exception?
      _pos = 4;
    }
  }
}

void transmit(){
  checkConnection();
  // TODO
}
