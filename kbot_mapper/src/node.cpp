#include "ros/ros.h"
#include <gflags/gflags.h>
#include "sensor_msgs/Range.h"
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include "math.h"
#include <kbot_bridge/SonarPing.h>

octomap::OcTree *tree;

double resolution = 0.01; // meters

void addArc(octomap::Pointcloud *cloud, octomap::pose6d robot_pose, double angle, double range, double field_of_view, double ray_diff) {
  
  // radian diff between each ray
  double diff_angle = ray_diff / range; // arc: theta = L/r

  robot_pose.rot().inv_IP(); // needed to convert FROM robot-based coords

  octomap::point3d ray(1,0,0); // in robot-base

  // rotate (around z) to left-most ray
  ray.rotate_IP(0, 0, angle-field_of_view/2 + fmod(field_of_view, diff_angle));

  double cos_diff_angle = cos(diff_angle), // 2D rotation matrix:
         sin_diff_angle = sin(diff_angle); //  (c -s)
                                           //  (s  c)

  for (int a = -field_of_view/2; a < field_of_view/2; a += diff_angle) {
    // convert to world-oriented coords before add
    cloud->push_back(robot_pose.rot().rotate(ray));

    // rotate counter-clockwise, around z (i.e. 2D).
    double x = ray.x(), y = ray.y(); 
    ray.x() = x * cos_diff_angle + y * -sin_diff_angle; // (c -s)   (x)
    ray.y() = x * sin_diff_angle + y * cos_diff_angle;  // (s  c) * (y)
  }

}

void handleSonarMsg(const kbot_bridge::SonarPing::ConstPtr& msg){
  
  octomap::pose6d robot_pose; // TODO new msg type
  double angle; // TODO new msg type
  
  // TODO use msg->header timestamp for something?
  // TODO use msg->radiation_type for something? for spam?

  octomap::Pointcloud *scan = new octomap::Pointcloud();
  addArc(scan, robot_pose, angle, msg->range, msg->field_of_view, resolution);

  tree->insertPointCloud(scan, robot_pose.trans());
}


int main(int argc, char **argv){
  ros::init(argc, argv, "kbot_mapper");
  ros::NodeHandle n;

  google::SetVersionString("TODO version"); // TODO version
  google::SetUsageMessage("TODO usage"); // TODO usage msg
  google::ParseCommandLineFlags(&argc, &argv, true); // TODO conflict with ros flags?


  ros::Subscriber sub = n.subscribe("TODO", 1000, handleSonarMsg);

  // TODO change resolution from flag?

  tree = new octomap::OcTree(resolution);

  // TODO pretty print? i.e. convert to dm/cm/mm, etc.
  ROS_INFO("Created octree with resolution: %fm", resolution);

  // TODO flag to read from file?
  //tree = new octomap::OcTree(filename);


  // TODO moveIt


  ros::spin();


  // TODO save tree to file?
  // TODO get filename from flag?
  // TODO allow overwriting files?


  return 0;
}
