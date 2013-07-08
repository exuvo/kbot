#ifndef KBOT_BRIDGE_PARSER_H
#define KBOT_BRIDGE_PARSER_H

enum class M_Type: uint8_t {
// When changing remember to update to/from methods. // TODO update M_TYPE_COUNT instead.
  //Ping, Power, Sonar, Tracks, Dome, Console, Text
  Ping=0, Power=1, Sonar=2, Tracks=3, Dome=4, Console=5, Text=6 // explicit just to be sure
};
#define M_TYPE_COUNT 6

M_Type toMType(uint8_t id);
uint8_t fromMType(M_Type type);

class Message{
private:
public:
  uint16_t length;
  M_Type type;
  uint8_t* data;
  uint8_t checksum; // type + data
  Message(uint16_t length_, M_Type type_): length(length_), type(type_), data(new uint8_t[length]) {}
 ~Message(){delete[] data;}
  void calcChecksum();
};

void parse(Message* m);

#endif /* KBOT_BRIDGE_PARSER_H */
