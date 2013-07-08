#include "ros/ros.h"
#include "sensor_msgs/Range.h"
#include <gflags/gflags.h>


void subscriptionCallback(const sensor_msgs::Range::ConstPtr& msg){
  //TODO
}

int main(int argc char **argv){
  ros::init(argc, argv, "kbot_mapper");
  ros::Nodehandle n;

  google::SetVersionString("TODO version"); // TODO version
  google::SetusageMessage("TODO usage"); // TODO usage msg
  google::ParseCommandLineFlags(&argc, &argv, true);


  ros::Subscriber sub m.subscribe("TODO", 1000, callback);

  // TODO octomap
  // TODO moveIt

  ros::spin();

  return 0;
}
