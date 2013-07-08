#include "ros/ros.h"
#include "sensor_msgs/Range.h"
#include <gflags/gflags.h>


void subscriptionCallback(const sensor_msgs::Range::ConstPtr& msg){
  // TODO do something with Range msg
}

int main(int argc, char **argv){
  ros::init(argc, argv, "kbot_mapper");
  ros::NodeHandle n;

  google::SetVersionString("TODO version"); // TODO version
  google::SetUsageMessage("TODO usage"); // TODO usage msg
  google::ParseCommandLineFlags(&argc, &argv, true);


  ros::Subscriber sub = n.subscribe("TODO", 1000, subscriptionCallback);

  // TODO octomap
  // TODO moveIt

  ros::spin();

  return 0;
}
