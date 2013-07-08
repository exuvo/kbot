#ifndef KBOT_MAPPER_MATH_H
#define MATH_H

#include <octomap/OcTree.h>
#include <octomap/octomap.h>

struct arc3d {
  octomap::point3d vec; // points to center of arc.
  double angle; // radians
};

/*
 * Divides arc into rays and adds those to cloud.
 * The rays will have angle_diff radians between them.
 */
void addArc(octomap::Pointcloud *cloud, arc3d arc, double angle_diff);

octomap::point3d rot(std::vector<double> matrix, octomap::point3d v);

#endif

