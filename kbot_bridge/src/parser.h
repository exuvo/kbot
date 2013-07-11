#ifndef KBOT_BRIDGE_PARSER_H
#define KBOT_BRIDGE_PARSER_H

enum class M_Type: uint8_t {
// When changing remember to update to/from methods. // TODO update M_TYPE_COUNT instead.
  Ping='p', Power='P', Sonar='s', Odometry='o', Dome='d', Console='c', Text='t', IMU='i', Time='T' // explicit just to be sure
};
#define M_TYPE_COUNT 6

class Message{
private:
public:
  uint16_t length;
  M_Type type;
  uint8_t* data;
  uint8_t checksum; // type + data
  Message(uint16_t length_, M_Type type_): length(length_), type(type_), data(new uint8_t[length]) {}
  Message(uint16_t length_, uint8_t id);
 ~Message(){delete[] data;}
	uint8_t typeToInt();
  void calcChecksum();
	uint16_t expectedLength();
};

void parse(Message* m);

#endif /* KBOT_BRIDGE_PARSER_H */
