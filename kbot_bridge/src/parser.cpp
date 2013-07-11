
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
		ROS_WARN_THROTTLE(10, "Message length does not match expected length for type: %c", (char)m->type);
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

constexpr uint8_t fromMType(M_Type type){
  return (uint8_t) type;
}

Message::Message(uint16_t length_, uint8_t id){
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
  for(int i=0; i< length; i++){
    checksum += data[i];
  }
}

uint16_t Message::expectedLength(){
  switch(type) {
    case M_Type::Ping: 
			return 0;
    case M_Type::Power:
			return 0;
    case M_Type::Sonar:
			return 0;
    case M_Type::Odometry:
			return 0;
    case M_Type::Dome:
			return 0;
    case M_Type::Console:
			return 0;
    case M_Type::Text:
			return 0;
    case M_Type::IMU:
			return 0;
    case M_Type::Time:
			return 0;
    default: throw out_of_range(__FILE__ ": enum M_Type");
  }
}
