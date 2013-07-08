#include "ros/ros.h"
#include <gflags/gflags.h>
#include "sensor_msgs/Range.h"
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include "math.h"

octomap::OcTree *tree;

/* The current position of the kbot. */
octomap::point3d sensor_origin;

/* The direction the vector is pointing in. Allways normalized. */
octomap::point3d sensor_direction;

/*
 * The radian distance between rays
 * when converting an sonar arc to rays.
 */
double angle_diff = 5.0 * 3.14159/180.0;


void handleSonarMsg(const sensor_msgs::Range::ConstPtr& msg){
  
  if (msg->min_range > msg->max_range) {
    // TODO err
    return;
  }

  
  // TODO use msg->header timestamp for something?
  // TODO use msg->radiation_type for something? for spam?

  double len = msg->range;
  // TODO ignore out-of-range messages instead?
  if (len < msg->min_range) {
    len = msg->min_range;
  } else if (len > msg->max_range){
    len = msg->max_range;
  }


  octomap::Pointcloud *scan = new octomap::Pointcloud();

  arc3d arc;
  arc.vec = sensor_direction;
  arc.angle = msg->field_of_view;
  addArc(scan, arc, angle_diff);

  tree->insertPointCloud(scan, sensor_origin);
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
