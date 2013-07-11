#include "ros/ros.h"
#include "std_msgs/String.h"
#include "serial.h"
#include "parser.h"
#include <gflags/gflags.h>
#include "kbot_bridge/SonarPing.h"
#include "kbot_bridge/Power.h"

using namespace std;

DEFINE_int32(baud, 9600, "Baudrate.");
DEFINE_string(port, "", "Serial port.");

ros::NodeHandle node;
ros::Publisher sonar_pub, power_pub;

int main(int argc, char **argv){
  ros::init(argc, argv, "kbot_bridge");

  google::SetVersionString("Best version.");
  google::SetUsageMessage("Write something clever here.");
  google::ParseCommandLineFlags(&argc, &argv, true);

  if(!open(FLAGS_port, FLAGS_baud)){
    ROS_FATAL("Failed to open serial port");
    return(1);
  }
  ROS_INFO("Serial port open");
  
  //TODO add output messages: sonar(Range), Odometry, Imu, TimeReference, 
  //TODO add input messages; motorControl(forward, left)

  sonar_pub = node.advertise<kbot_bridge::SonarPing>("sonar_pings", 100);
  power_pub = node.advertise<kbot_bridge::Power>("power", 10);
  
  ros::Rate loop_rate(10);

  while(ros::ok()){
    receive();
    transmit();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
