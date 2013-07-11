
#include <algorithm>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "serial.h"
//#include "dbg.h"
#include "parser.h"
#include "kbot_bridge/SonarPing.h"
#include "node.h"
#include <chrono>

using namespace std;

chrono::time_point<chrono::steady_clock> lastReceive, pingSent;
chrono::milliseconds pingRoundTrip;

void parseSonar(Message* m) {
  kbot_bridge::SonarPing msg;
  // TODO parse
  
  // TODO manually generate msg->header?
  
  msg.pose.position.x = 0.0; 
  msg.pose.position.y =  0.0; 
  msg.pose.position.z =  0.0; 

  msg.pose.orientation.x =  0.0; 
  msg.pose.orientation.y =  0.0; 
  msg.pose.orientation.z =  0.0; 
  msg.pose.orientation.w =  0.0; 
  
  // TODO manually generate range->header?
  msg.range.radiation_type = 0; // 0 = ULTRASOUND
  msg.range.field_of_view = 0.0;
  msg.range.min_range = 0.0;
  msg.range.max_range = 0.0;
  msg.range.range = 0.0;
  
  sonar_pub.publish(msg);
}

void parse(Message* m){
	lastReceive = chrono::steady_clock::now();

	if(m->length != m->expectedLength()){
		ROS_WARN_THROTTLE(10, "Message length does not match expected length for type:");
		return;
	}

  switch(m->type) {
    case M_Type::Ping:
      pingRoundTrip = chrono::duration_cast<chrono::milliseconds>(lastReceive - pingSent);
      return;
    case M_Type::Power:
      // TODO kbot_bridge::Power
      return; 
    case M_Type::Sonar:
	    parseSonar(m);
      return;
    case M_Type::Odometry:
      // TODO nav_msgs::Odometry
      return;
    case M_Type::Dome:
      // TODO geometry_msgs::Pose
      return;
    case M_Type::Console:
      // TODO
      break;
    case M_Type::Text:
      // TODO
    case M_Type::IMU:
      // TODO sensor_msgs::Imu
      return;
    case M_Type::Time:
      // TODO sensor_msgs::TimeReference
      return;
    default:
      return; // TODO err
  }
}

uint8_t fromMType(M_Type type){
  return (uint8_t) type;
}

Message::Message(uint16_t length_, uint8_t id){
  switch(id) {
    case fromMType(M_Type::Ping): return M_Type::Ping;
    case fromMType(M_Type::Power): return M_Type::Power;
    case fromMType(M_Type::Sonar): return M_Type::Sonar;
    case fromMType(M_Type::Tracks): return M_Type::Tracks;
    case fromMType(M_Type::Dome): return M_Type::Dome;
    case fromMType(M_Type::Console): return M_Type::Console;
    case fromMType(M_Type::Text): return M_Type::Text;
    case fromMType(M_Type::): return M_Type::;
    case fromMType(M_Type::): return M_Type::;
    default: throw logic_error(__FILE__ ": enum M_Type out of range");
  }
}

uint8_t Message::typeToInt(){
  return fromMType(type);
}

void Message::calcChecksum(){
  for(int i=0; i< length; i++){
    checksum += data[i];
  }
}
