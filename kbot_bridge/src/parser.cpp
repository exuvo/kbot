
#include <algorithm>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "serial.h"
#include "parser.h"
#include "kbot_bridge/SonarPing.h"
#include "kbot_bridge/Power.h"
#include "node.h"
#include <chrono>
#include <sensor_msgs/Range.h>

using namespace std;

chrono::time_point<chrono::steady_clock> lastReceive, pingSent;
chrono::milliseconds pingRoundTrip;

void parseSonar(Message* m) {
  kbot_bridge::SonarPing msg;
  msg.header.stamp = ros::Time::now();
  
  msg.pose.orientation.x = m->readDouble(); 
  msg.pose.orientation.y = m->readDouble(); 
  msg.pose.orientation.z = m->readDouble(); 
  msg.pose.orientation.w = m->readDouble(); 

	uint16_t distance = m->readUInt16();
  msg.pose.position.x = m->readInt32(); 
  msg.pose.position.y = m->readInt32(); 
  msg.pose.position.z = 0.2; //TODO measure sensor height

  msg.range.radiation_type = sensor_msgs::Range::ULTRASOUND;
  msg.range.field_of_view = 0.0; //TODO measure field of view
  msg.range.min_range = 0.1;
  msg.range.max_range = 7.7;
  msg.range.range = distance / 100.0f;

// TODO fill in range->header?
// range header time = measurement time
//	msg.range.header.stamp = ros::Time(?)
  
  sonar_pub.publish(msg);
}

void parsePower(Message* m){
	kbot_bridge::Power msg;
	
	msg.mainVoltage = m->readUInt16() / 100.0f;
	msg.mainVoltageDiff = m->readInt16() / 100.0f;
	msg.mainCurrent = m->readInt16() / 10.0f;
	msg.mainCurrentDiff = m->readInt16() / 10.0f;
	msg.secondaryVoltage = m->readInt16() / 1000.0f;
	msg.mainStatus = m->readUInt8();
	msg.status = m->readUInt8();
	msg.charging = m->readUInt8();

	power_pub.publish(msg);
}

void parse(Message* m){
	lastReceive = chrono::steady_clock::now();

	if(m->expectedLength() != -1 && m->length != m->expectedLength()){
		ROS_WARN_THROTTLE(10, "Message length does not match expected length for type: %c", (char)m->type);
		return;
	}

  std::string s{(const char*)m->data, (size_t)m->length};
  ROS_DEBUG("Received '%s'", s.c_str());

  switch(m->type) {
    case M_Type::Ping:
      pingRoundTrip = chrono::duration_cast<chrono::milliseconds>(lastReceive - pingSent);
      return;
    case M_Type::Power:
      parsePower(m);
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

