#ifndef SERIAL2_H
#define SERIAL2_H

#include <string>
#include "serial/serial.h"

using namespace std;

#define START_BYTE 0x7e
#define ESCAPE 0x7d

bool open(string portname, int baudrate);
serial::Serial& getSerial();
void checkSerial();
void receive();
void transmit();

#endif /* SERIAL2_H */
