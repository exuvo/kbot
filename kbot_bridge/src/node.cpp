#include "ros/ros.h"
#include "std_msgs/String.h"
#include "serial.h"
#include "parser.h"
#include <gflags/gflags.h>
#include "kbot_bridge/SonarPing.h"
#include "kbot_bridge/Power.h"
#include "subscriptions.h"

DEFINE_int32(baud, 9600, "Baudrate");
DEFINE_string(port, "", "Serial port");

ros::NodeHandle* node;
ros::Publisher sonar_pub, power_pub;

int main(int argc, char **argv){
  ros::init(argc, argv, "kbot_bridge");
  node = new ros::NodeHandle;

  gflags::SetVersionString("Best version.");
  gflags::SetUsageMessage("Write something clever here.");
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  if(!FLAGS_port.empty()){
 		if(open(FLAGS_port, FLAGS_baud)){
		  ROS_INFO("Serial port open.");
		}
  } else {
		ROS_INFO("No serial port given, dry run started");
  }
  
  //TODO add output messages: sonar(Range), Odometry, Imu, TimeReference, 
  //TODO add input messages; motorControl(forward, left)

  sonar_pub = node->advertise<kbot_bridge::SonarPing>("sonar_pings", 100);
  power_pub = node->advertise<kbot_bridge::Power>("power", 10);
  initSubscriptions();
  
  ros::Rate loop_rate(10);

  while(ros::ok()){
   	receive();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
