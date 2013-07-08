#include "ros/ros.h"
#include <gflags/gflags.h>
#include "sensor_msgs/Range.h"
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include "math.h"

octomap::OcTree *tree;

octomap::pose6d robot_pose;

/*
 * The distance between ray end-points on the arc.
 */
double ray_diff = 5.0 * 3.14159/180.0; // TODO


void handleSonarMsg(const sensor_msgs::Range::ConstPtr& msg){
  
  if (msg->min_range > msg->max_range) {
    // TODO err
    return;
  }

  
  // TODO use msg->header timestamp for something?
  // TODO use msg->radiation_type for something? for spam?


  octomap::Pointcloud *scan = new octomap::Pointcloud();
  addArc(scan, robot_pose, msg->range, msg->field_of_view, ray_diff);

  tree->insertPointCloud(scan, robot_pose.trans());
}


int main(int argc, char **argv){
  ros::init(argc, argv, "kbot_mapper");
  ros::NodeHandle n;

  google::SetVersionString("TODO version"); // TODO version
  google::SetUsageMessage("TODO usage"); // TODO usage msg
  google::ParseCommandLineFlags(&argc, &argv, true);


  ros::Subscriber sub = n.subscribe("TODO", 1000, handleSonarMsg);


  double resolution = 0.01; // meters // TODO from flag?

  tree = new octomap::OcTree(resolution);

  // TODO pretty print? i.e. convert to dm/cm/mm, etc.
  ROS_INFO("Created octree with resolution: %fm", resolution);

  // TODO flag to read from file.
  //tree = new octomap::OcTree(filename);


  // TODO octomap
  // TODO moveIt


  ros::spin();


  // TODO save tree to file?
  // TODO get filename from flag?
  // TODO allow overwriting files?


  return 0;
}
