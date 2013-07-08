#include "math.h"
#include <octomap/OcTree.h>
#include <octomap/octomap.h>
#include <vector>


void addArc(octomap::Pointcloud *cloud, arc3d arc, double angle_diff) {
  
  // rotate arg.vec to get rays.
  // TODO:
  // rot_normal = new octomap::point3d(-arc.vec.x(), -arc.vec.y(), 1/arc.vec.z()); // rotation-normal
  // Quaternion q = new Quaternion(rot_normal, radSkip); // TODO Quaternion from where?
  // std::vector<double> matrix;
  // q.toRotMatrix(matrix);
  // for (...) {cloud->push_back(rot(matrix,v))}


  // TODO can octomap::pose6d be used for rotation instead?
  std::vector<double> matrix;
  octomap::pose6d(0,0,0,0,0,angle_diff).rot().toRotMatrix(matrix);

  octomap::point3d v = arc.vec;
  // TODO rotate v to startposition
  // TODO loop: push v; rot v;
}

octomap::point3d rot(std::vector<double> matrix, octomap::point3d v) {

}
