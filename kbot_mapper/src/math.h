#ifndef KBOT_MAPPER_MATH_H
#define MATH_H

#include <octomap/OcTree.h>
#include <octomap/octomap.h>

/*
 * Divides arc into rays and adds those to cloud.
 * The rays will have angle_diff radians between them.
 */
void addArc(octomap::Pointcloud *cloud, octomap::pose6d robot_pose, double range, double field_of_view, double ray_diff);

/*
 * Multiplies the matrix with the vector, like: v' = Mv.
 */
octomap::point3d mul(std::vector<double> matrix, octomap::point3d v);

#endif

