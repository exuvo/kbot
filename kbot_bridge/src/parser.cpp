
#include <algorithm>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "serial.h"
//#include "dbg.h"
#include "parser.h"
#include "kbot_bridge/SonarPing.h"
#include "node.h"

using namespace std;

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
  switch(m->type) {
    case M_Type::Ping:
      // TODO
      break;
    case M_Type::Power:
      // TODO kbot_bridge::Power
      return; 
    case M_Type::Sonar:
      // TODO
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
    default: // TODO err
  }
}

M_Type toMType(uint8_t id){
  if (id >= 0 && id <= M_TYPE_COUNT) {
    return (M_Type)id;
  } else {
    throw logic_error(__FILE__ ": enum M_Type out of range");
  }
//  switch(id) {
//    case 0: return M_Type::Ping;
//    case 1: return M_Type::Power;
//    case 2: return M_Type::Sonar;
//    case 3: return M_Type::Tracks;
//    case 4: return M_Type::Dome;
//    case 5: return M_Type::Console;
//    case 6: return M_Type::Text;
//    default: throw logic_error(__FILE__ ": enum M_Type out of range");
//  }
}

uint8_t fromMType(M_Type type){
  return (uint8_t) type;
//  switch(type) {
//    case M_Type::Ping:    return 0;
//    case M_Type::Power:   return 1;
//    case M_Type::Sonar:   return 2;
//    case M_Type::Tracks:  return 3;
//    case M_Type::Dome:    return 4;
//    case M_Type::Console: return 5;
//    case M_Type::Text:    return 6;
//  }
}

void Message::calcChecksum(){
  for(int i=0; i< length; i++){
    checksum += data[i];
  }
}
