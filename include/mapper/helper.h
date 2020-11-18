// Copyright (c) 2020 by Pensa Systems, Inc. -- All rights reserved
// Confidential and Proprietary

#pragma once

#include "mapper/msg_conversions.h"

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <octomap/octomap.h>
#include <pcl/point_cloud.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Cpp includes
#include <string>
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

bool IsTreeRootOnly(const octomap::OcTree &tree);

struct timespec TimeFromNow(const uint& increment_sec);

// Function that returns whether value1 and value2 are close enough (for double inputs)
bool AreDoubleApproxEqual(const double &value1, const double &value2, const double &epsilon);

// Function to lookup an arbitrary transform in the tf tree
bool LookupTransform(const std::string &from_frame,
                     const std::string &to_frame,
                     tf2::Transform *tf_output);

}  // namespace helper
