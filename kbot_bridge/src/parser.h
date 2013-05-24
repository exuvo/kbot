#ifndef PARSER_H
#define PARSER_H

enum class M_Type: uint8_t {
// When changing remember to update to/from methods.
	Ping, Power, Sonar, Tracks, Dome
};

M_Type toMType(uint8_t id);
uint8_t fromMType(M_Type type);

class Message {
private:
public:
	uint16_t length;
	M_Type type;
	uint8_t* data;
	uint8_t checksum; // type + data
	Message(uint16_t length_, M_Type type_): length(length_), type(type_), data(new uint8_t[length]){}
 ~Message(){delete[] data;}
};

void parse(Message m);

#endif /* PARSER_H */
