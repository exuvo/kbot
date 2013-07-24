#include "serial.h"
#include "ros/ros.h"
#include "parser.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

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
  //  3 bytes:
  //     1 byte  to compare with START_BYTE
  //     1 bytes into _msg->len
  //     1 byte  into _msg->type
  //  len+1 bytes:
  //   len bytes into _msg->data
  //     1 byte  to compare with _msg->checksum

  
  size_t amount;
  while((amount=port.available()) > 0){
    if(_pos > 0){
      amount = min((unsigned int)amount, _msg->length + 3 - _pos);
      port.read(&(_msg->data[_pos]), amount);
      _pos += amount;

      if(_pos == _msg->length + 3 && port.available()){
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
  
    }else if(amount >= 3){
      uint8_t b;
      port.read(&b, 1);

      if(b != START_BYTE){
        ROS_WARN_THROTTLE(1, "Serial: Expected start byte, got this instead: %u", b);
        return;
      }
      uint8_t d[2];
      port.read(d, 2);
      uint16_t len = d[0];
			if(len > 0){
	      _msg = new Message(len - 1, d[1]); // TODO catch exception?
      	_pos = 3;
			}
    }
  }
}

void transmit(){
  checkConnection();
  // TODO
}

constexpr uint8_t fromMType(M_Type type){
  return (uint8_t) type;
}

Message::Message(uint8_t length_, uint8_t id){
	M_Type mtype;
  switch(id) {
    case fromMType(M_Type::Ping): 
			mtype = M_Type::Ping;
			break;
    case fromMType(M_Type::Power):
			mtype = M_Type::Power;
			break;
    case fromMType(M_Type::Sonar):
			mtype = M_Type::Sonar;
			break;
    case fromMType(M_Type::Odometry):
			mtype = M_Type::Odometry;
			break;
    case fromMType(M_Type::Dome):
	 		mtype = M_Type::Dome;
			break;
    case fromMType(M_Type::Console):
			mtype = M_Type::Console;
			break;
    case fromMType(M_Type::Text):
 			mtype = M_Type::Text;
			break;
    case fromMType(M_Type::IMU):
 			mtype = M_Type::IMU;
			break;
    case fromMType(M_Type::Time):
 			mtype = M_Type::Time;
			break;
    default: throw out_of_range(__FILE__ ": enum M_Type");
  }

 Message(length_, mtype);
}

uint8_t Message::typeToInt(){
  return fromMType(type);
}

void Message::calcChecksum(){
	checksum = typeToInt();
  for(int i=0; i< length; i++){
    checksum += data[i];
  }
}

uint8_t Message::expectedLength(){
  switch(type) {
    case M_Type::Ping: 
			return 1;
    case M_Type::Power:
			return 13;
    case M_Type::Sonar:
			return 58;
    case M_Type::Odometry:
			return 0;
    case M_Type::Dome:
			return 0;
    case M_Type::Console:
			return -1;
    case M_Type::Text:
			return -1;
    case M_Type::IMU:
			return 0;
    case M_Type::Time:
			return 0;
    default: throw out_of_range(__FILE__ ": enum M_Type");
  }
}


uint8_t Message::readUInt8(){
	return n();
}

int8_t Message::readInt8(){
	return (int8_t) n();
}

uint16_t Message::readUInt16(){
	return (uint16_t)n() << 8 | n();
}

int16_t Message::readInt16(){
	return (int16_t)n() << 8 | n();
}

uint32_t Message::readUInt32(){
	return (uint32_t)n() << 24 | (uint32_t)n() << 16 | (uint32_t)n() << 8 | n();
}

int32_t Message::readInt32(){
	return (int32_t)n() << 24 | (int32_t)n() << 16 | (int32_t)n() << 8 | n();
}

uint8_t Message::n(){
	if(_next < length){
		return data[_next++];
	}
	throw out_of_range(__FILE__ "trying to read beyond end of Message");
}

void Message::n(uint8_t* p, uint8_t len){
	while(len-- >= 0){
		*p++ = n();
	}
}

#define FRAC_MAX 9223372036854775807LL /* 2**63 - 1 */

struct dbl_packed{
    int32_t exp;
    int64_t frac;
};

union dbl_packed_bytes{
	struct dbl_packed pack;
	uint8_t bytes[12];
};

void pack(double x, struct dbl_packed *r){
    double xf = fabs(frexp(x, (int*)&r->exp)) - 0.5;

    if (xf < 0.0){
        r->frac = 0;
        return;
    }

    r->frac = 1 + (long long)(xf * 2.0 * (FRAC_MAX - 1));

    if (x < 0.0)
        r->frac = -r->frac;
}

double unpack(const struct dbl_packed *p){
    double xf, x;

    if (p->frac == 0)
        return 0.0;

    xf = ((double)(llabs(p->frac) - 1) / (FRAC_MAX - 1)) / 2.0;

    x = ldexp(xf + 0.5, p->exp);

    if (p->frac < 0)
        x = -x;

    return x;
}

double Message::readDouble(){
	union dbl_packed_bytes bytes;
	n(bytes.bytes, 12);
	return unpack(&bytes.pack);
}
