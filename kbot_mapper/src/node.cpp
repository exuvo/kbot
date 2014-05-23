#include "ros/ros.h"
#include <gflags/gflags.h>
#include "sensor_msgs/Range.h"
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <math.h>
#include "kbot_bridge/SonarPing.h"

octomap::OcTree *tree;

double resolution = 0.01; // meters


/// Simple.
void addRay(octomap::Pointcloud *cloud, octomap::pose6d orientation, double range, double field_of_view, double ray_diff) {
  octomap::point3d ray(range,0,0); // in sensor-base
  octomap::point3d v = orientation.rot().rotate(ray); // in world-base
  cloud->push_back(v);
}

/// Bad. 2d arc.
void addArc(octomap::Pointcloud *cloud, octomap::pose6d orientation, double range, double field_of_view, double ray_diff) {
  
  // radian diff between each ray
  double diff_angle = ray_diff / range; // arc: theta = L/r

  orientation.rot().inv_IP(); // needed to convert FROM sensor-based coords

  octomap::point3d ray(range,0,0); // in sensor-base
  
  // rotate (around y) to left-most ray
  ray.rotate_IP(0, 0, -field_of_view/2.0 + fmod(field_of_view, diff_angle)/2.0);

  double cos_diff_angle = cos(diff_angle), // 2D rot-matrix:
         sin_diff_angle = sin(diff_angle); //  (c -s)
                                           //  (s  c)
  for (int a = -field_of_view/2; a <= field_of_view/2; a += diff_angle) {
    
    octomap::point3d v = orientation.rot().rotate(ray); // to world-coords
    cloud->push_back(v);

    ROS_DEBUG("Added ray. sensor-based coords: (%f,%f,%f), world-based coords: (%f,%f,%f).", ray.x(), ray.y(), ray.z(), v.x(), v.y(), v.z());

    // rotate clockwise, around z (i.e. in 2D-plane).
    double x = ray.x(), y = ray.y(); 
    ray.x() = x * cos_diff_angle + y * -sin_diff_angle; // (c -s)   (x)
    ray.y() = x * sin_diff_angle + y * cos_diff_angle;  // (s  c) * (y)
  }

}

/// Best way of doing it probably. Gives a cone with a round bottom.
// Like a cone on top of a hemisphere.
void addBurst(octomap::Pointcloud *cloud, octomap::pose6d orientation, double range, double field_of_view, double ray_diff) {
  // TODO efficiency?
  double diff_angle = ray_diff / range; // arc: theta = L/r

  orientation.rot().inv_IP(); // needed to convert FROM sensor-based coords
  for (double a = 0.0; a <= 2.0*M_PI; a += diff_angle) {
    double sa = sin(a), ca = cos(a);
    for (double b = 0.0; b <= field_of_view/2.0; b += diff_angle) {
      double c = M_PI/2.0 - b;
      octomap::point3d ray(sin(c), sa*cos(c), ca*cos(c)); // magic
      ray += range;
      octomap::point3d v = orientation.rot().rotate(ray); // to world-coords
      cloud->push_back(v);
    }
  }
}


// Bad
void addCone(octomap::Pointcloud *cloud, octomap::pose6d orientation, double range, double field_of_view, double ray_diff) {
  // TODO how to evenly distribute rays? randomly? circularly? effectiveness?
  orientation.rot().inv_IP(); // needed to convert FROM sensor-based coords

  // draws circles. one radius at a time.
  double base_r = range*tan(field_of_view/2.0);
  for (double r = ray_diff/2.0; r <= base_r; r += ray_diff) {
    double rot_angle = ray_diff/r,
           circ = 2.0*M_PI*r,
           cr = cos(rot_angle), // for rot-matrix.
           sr = sin(rot_angle); //

    // x:forward, y:up, z:right
    octomap::point3d ray(range,0,r); // note: length may be >range (cone).

    int no = circ/ray_diff + 1; // rounding up
    while (--no >= 0) {
      octomap::point3d v = orientation.rot().rotate(ray); // to world-coords.
      cloud->push_back(v);
      // rotate 2d. (ignore x)
      double y = ray.y(), z = ray.z(); 
      ray.y() = y * cr + z * -sr; // (c -s)   (y)
      ray.z() = y * sr + z * cr;  // (s  c) * (z)
      // precision declines.. have to ensure length... TODO better way
      float len = sqrt(ray.y()*ray.y() + ray.z()*ray.z());
      ray.y() = ray.y()/len * r;
      ray.z() = ray.z()/len * r;
    }
  }

}


void handleSonarMsg(const kbot_bridge::SonarPing::ConstPtr& msg){
  
  // TODO use msg->header timestamp for something?
  // TODO use msg->radiation_type for something? for spam?
  

  octomap::point3d sensor_position;
  sensor_position.x() = msg->pose.position.x;
  sensor_position.y() = msg->pose.position.y;
  sensor_position.z() = msg->pose.position.z;

  // note: can't reach type octomath:Quarternion directly
  // so have to go through pose6d.
  octomap::pose6d sensor_orientation;
  sensor_orientation.rot().x() = msg->pose.orientation.x;
  sensor_orientation.rot().y() = msg->pose.orientation.y;
  sensor_orientation.rot().z() = msg->pose.orientation.z;
  sensor_orientation.rot().u() = msg->pose.orientation.w;


  octomap::Pointcloud *scan = new octomap::Pointcloud();

  //addRay(scan, sensor_orientation, msg->range.range, msg->range.field_of_view, resolution);
  addArc(scan, sensor_orientation, msg->range.range, msg->range.field_of_view, resolution);
  //addCone(scan, sensor_orientation, msg->range.range, msg->range.field_of_view, resolution);

  tree->insertPointCloud(scan, sensor_position);
}


int main(int argc, char **argv){
  ros::init(argc, argv, "kbot_mapper");
  ros::NodeHandle n;

  google::SetVersionString("TODO version"); // TODO version
  google::SetUsageMessage("TODO usage"); // TODO usage-msg
  google::ParseCommandLineFlags(&argc, &argv, true); // TODO conflicts with ros flags?


  ros::Subscriber sub = n.subscribe("TODO", 1000, handleSonarMsg);

  // TODO change resolution from flag?

  tree = new octomap::OcTree(resolution);

  // TODO pretty print? i.e. convert to dm/cm/mm, etc.
  ROS_INFO("Created octree with resolution: %fm", resolution);

  // TODO flag to read from file?
  //tree = new octomap::OcTree(filename);


  // TODO config srvs?
  // TODO add srvs for adding ray/arc/cone.

  // TODO moveIt

  ros::spin(); // TODO build pointcloud from multiple msg and insert it at end of each spin (instead of at end of each msg)?


  // TODO save tree to file?
  // TODO get filename from flag?
  // TODO allow overwriting files?


  return 0;
}
