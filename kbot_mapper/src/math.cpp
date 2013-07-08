#include "math.h"
#include <octomap/OcTree.h>
#include <octomap/octomap.h>
#include <vector>
#include <cmath>


void addArc(octomap::Pointcloud *cloud, octomap::pose6d robot_pose, double range, double field_of_view, double ray_diff) {
  // TODO test (don't trust my math)
  // TODO optimize?
  

  double diff_angle = ray_diff / range; // theta = L/r

  octomap::point3d dir = robot_pose.rot().toEuler();
  octomap::pose6d rotpose(robot_pose.x(), robot_pose.y(), robot_pose.z(), (double)dir.roll(), (double)dir.pitch(), diff_angle);

  std::vector<double> matrix; // 00,01,02,10,11,12,20,21,22 (rc)
  rotpose.rot().toRotMatrix(matrix);

  // add left-side rays
  octomap::point3d v = dir;
  for (double angle = 0; angle < dir.yaw()/2; angle += diff_angle) {
    v = mul(matrix, v);
    cloud->push_back(v);
  }

  // TODO possible to reuse old matrix?
  // add right-side rays
  v = dir;
  rotpose = octomap::pose6d(robot_pose.x(), robot_pose.y(), robot_pose.z(), (double)dir.roll(), (double)dir.pitch(), -diff_angle);
  rotpose.rot().toRotMatrix(matrix);
  for (double angle = 0; angle < dir.yaw()/2; angle += diff_angle) {
    cloud->push_back(v);
    v = mul(matrix, v);
  }

}


octomap::point3d mul(std::vector<double> m, octomap::point3d v) {
  octomap::point3d u;
  u.x() = m[0+0]*v.x() + m[0+1]*v.y() + m[0+2]*v.z(),
  u.y() = m[3+0]*v.x() + m[3+1]*v.y() + m[3+2]*v.z(),
  u.y() = m[6+0]*v.x() + m[6+1]*v.y() + m[6+2]*v.z();
  return u;
}
