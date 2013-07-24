#ifndef KBOT_BRIDGE_SERIAL_H
#define KBOT_BRIDGE_SERIAL_H

#include <string>
#include "serial/serial.h"

#define START_BYTE 0x7e

using namespace std;

enum class M_Type: uint8_t {
// When changing remember to update to/from methods. 
  Ping='p', Power='P', Sonar='s', Odometry='o', Dome='d', Console='c', Text='t', IMU='i', Time='T' 
};

class Message{
private:
	uint8_t _next;
	uint8_t n();
	void n(uint8_t*, uint8_t len);
public:
  uint8_t length; // excluding M_Type byte
  M_Type type;
  uint8_t* data;
  uint8_t checksum; // type + data
  Message(uint8_t length_, M_Type type_): length(length_), type(type_), data(new uint8_t[length]), _next(0), checksum(0) {}
  Message(uint8_t length_, uint8_t id);
 ~Message(){delete[] data;}
	uint8_t typeToInt();
  void calcChecksum();
	uint8_t expectedLength();
	uint8_t readUInt8();
	int8_t readInt8();
	uint16_t readUInt16();
	int16_t readInt16();
	uint32_t readUInt32();
	int32_t readInt32();
	double readDouble();
};

bool open(string portname, int baudrate);
serial::Serial& getSerial();
void checkSerial();
void receive();
void transmit();

#endif /* KBOT_BRIDGE_SERIAL_H*/
