// Copyright (c) 2020 by Pensa Systems, Inc. -- All rights reserved
// Confidential and Proprietary

#pragma once

#include "mapper/msg_conversions.h"

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <octomap/octomap.h>
#include <pcl/point_cloud.h>

#include <vector>

namespace helper {

// p2 - p1
void SubtractRosPoints(const geometry_msgs::Point& p1,
                       const geometry_msgs::Point& p2,
                       geometry_msgs::Vector3* v);

// p2 - p1
void SubtractRosPoints(const geometry_msgs::Point& p1,
                       const geometry_msgs::Point& p2,
                       Eigen::Vector3d* v);

double NormDistanceRosPoints(const geometry_msgs::Point& p1,
                             const geometry_msgs::Point& p2);

double VectorNormSquared(const double &x,
                         const double &y,
                         const double &z);

// Shift a point in pcl
void ShiftPclPoint(const pcl::PointXYZ& pt, const pcl::PointXYZ& shift,
                   pcl::PointXYZ* pt_out);

// Shift a whole pcl in a direction given by shift_vec
void ShiftPcl(const pcl::PointCloud< pcl::PointXYZ >& pcl_in,
              const Eigen::Vector3d& shift_vec,
              pcl::PointCloud< pcl::PointXYZ >* pcl_out);

void FindNearestCollision(const std::vector<octomap::point3d> &colliding_nodes,
                          const geometry_msgs::Point &origin,
                          geometry_msgs::Point *nearest_node,
                          double *min_dist);

struct timespec TimeFromNow(const uint& increment_sec);

// Function that returns whether value1 == value2 (for double inputs)
bool AreDoubleSame(const double &value1, const double &value2);

}  // namespace helper
