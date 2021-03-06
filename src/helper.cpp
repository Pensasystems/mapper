// Copyright (c) 2020 by Pensa Systems, Inc. -- All rights reserved
// Confidential and Proprietary

#include "mapper/helper.h"

// ROS includes
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

// C++ libraries
#include <limits>
#include <string>
#include <vector>

namespace helper {

// p2 - p1
void SubtractRosPoints(const geometry_msgs::Point& p1,
                       const geometry_msgs::Point& p2,
                       geometry_msgs::Vector3* v) {
    v->x = p2.x - p1.x;
    v->y = p2.y - p1.y;
    v->z = p2.z - p1.z;
}

// p2 - p1
void SubtractRosPoints(const geometry_msgs::Point& p1,
                       const geometry_msgs::Point& p2,
                       Eigen::Vector3d* v) {
    *v = Eigen::Vector3d(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
}

double NormDistanceRosPoints(const geometry_msgs::Point& p1,
                             const geometry_msgs::Point& p2) {
    Eigen::Vector3d dist;
    SubtractRosPoints(p1, p2, &dist);
    return dist.norm();
}

double VectorNormSquared(const double &x,
                         const double &y,
                         const double &z) {
    return x*x + y*y + z*z;
}

// Shift a point in pcl
void ShiftPclPoint(const pcl::PointXYZ& pt, const pcl::PointXYZ& shift,
                   pcl::PointXYZ* pt_out) {
    pt_out->x = pt.x + shift.x;
    pt_out->y = pt.y + shift.y;
    pt_out->z = pt.z + shift.z;
}

// Shift a whole pcl in a direction given by shift_vec
void ShiftPcl(const pcl::PointCloud< pcl::PointXYZ >& pcl_in,
              const Eigen::Vector3d& shift_vec,
              pcl::PointCloud< pcl::PointXYZ >* pcl_out) {
    pcl_out->resize(pcl_in.size());
    pcl::PointXYZ shift = msg_conversions::eigen_to_pcl_point(shift_vec);
    for (uint i = 0; i < pcl_in.size(); i++) {
        ShiftPclPoint(pcl_in.points[i], shift, &pcl_out->points[i]);
    }
}

void FindNearestCollision(const std::vector<octomap::point3d> &colliding_nodes,
                          const geometry_msgs::Point &origin,
                          geometry_msgs::Point *nearest_node,
                          double *min_dist) {
    *min_dist = std::numeric_limits<double>::infinity();
    for (uint i = 0; i < colliding_nodes.size(); i++) {
        const geometry_msgs::Point current_node =
            msg_conversions::set_ros_point(colliding_nodes[i].x(),
                                           colliding_nodes[i].y(),
                                           colliding_nodes[i].z());
        const double cur_distance = NormDistanceRosPoints(origin, current_node);
        if (cur_distance < *min_dist) {
            *nearest_node = current_node;
            *min_dist = cur_distance;
        }
    }
}

bool IsTreeRootOnly(const octomap::OcTree &tree) {
  octomap::OcTree::leaf_iterator it = tree.begin_leafs();
  if (it == tree.end_leafs()) {
    // Empty tree, return false
    return false;
  } else if (it.getDepth() == 0) {
    // Yeah, first node is the root, return true
    return true;
  }
  return false;
}

struct timespec TimeFromNow(const uint& increment_sec) {
  struct timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);
  ts.tv_sec += increment_sec;
  return ts;
}

bool AreDoubleApproxEqual(const double &value1, const double &value2, const double &epsilon) {
  return std::fabs(value1 - value2) < epsilon;
}

bool LookupTransform(const std::string &from_frame,
                     const std::string &to_frame,
                     tf2::Transform *tf_output) {
    // Create tf listener
    static tf2_ros::Buffer tf_buffer;
    static tf2_ros::TransformListener tf_listener(tf_buffer);
    try {
        const geometry_msgs::TransformStamped transform =
            tf_buffer.lookupTransform(from_frame, to_frame, ros::Time(0), ros::Duration(10.0));
        tf2::fromMsg(transform.transform, *tf_output);
    } catch (tf2::TransformException &ex) {
        ROS_ERROR("[mapper]: Transform between frame \"%s\" and frame \"%s\" was not found: %s",
                 from_frame.c_str(), to_frame.c_str(), ex.what());
        return false;
    }
    return true;
}

}  // namespace helper
