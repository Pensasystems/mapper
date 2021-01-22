/* Copyright (c) 2017, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 * 
 * All rights reserved.
 * 
 * The Astrobee platform is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Vector3.h>
#include <pcl/point_types.h>
#include <tf2/LinearMath/Transform.h>

namespace msg_conversions {

  Eigen::Vector3d        ros_point_to_eigen_vector(const geometry_msgs::Point & p);
  Eigen::Vector3d        ros_to_eigen_vector(const geometry_msgs::Vector3 & v);
  geometry_msgs::Vector3 eigen_to_ros_vector(const Eigen::Vector3d & v);
  void                   eigen_to_array_vector(const Eigen::Vector3d & v, float* array);
  void                   ros_to_array_vector(const geometry_msgs::Vector3 & v, float* array);
  geometry_msgs::Vector3 array_to_ros_vector(float* array);
  geometry_msgs::Vector3 set_ros_vector(const double & x, const double & y, const double & z);

  geometry_msgs::Point   eigen_to_ros_point(const Eigen::Vector3d & v);
  void                   eigen_to_ros_point(const Eigen::Vector3d &eigen_pt, geometry_msgs::Point *ros_pt);
  pcl::PointXYZ          eigen_to_pcl_point(const Eigen::Vector3d &pt);
  void                   ros_to_array_point(const geometry_msgs::Point & p, float* array);
  geometry_msgs::Point   array_to_ros_point(float* array);
  geometry_msgs::Point   set_ros_point(const double & x, const double & y, const double & z);
  geometry_msgs::Point   pcl_to_ros_point(const pcl::PointXYZ& pt);
  geometry_msgs::Point   tf_vector3_to_ros_point(const tf2::Vector3 & v);
  tf2::Transform         ros_pose_to_tf2_transform(const geometry_msgs::Pose &pose);

  Eigen::Quaterniond        ros_to_eigen_quat(const geometry_msgs::Quaternion & q);
  geometry_msgs::Quaternion eigen_to_ros_quat(const Eigen::Quaterniond & q);
  geometry_msgs::Quaternion eigen_to_ros_quat(const Eigen::Vector4d & v);
  void                      eigen_to_array_quat(const Eigen::Quaterniond & q, float* array);
  void                      ros_to_array_quat(const geometry_msgs::Quaternion & q, float* array);
  geometry_msgs::Quaternion array_to_ros_quat(float* array);
  geometry_msgs::Quaternion identity_quaternion();
  Eigen::Affine3d           ros_pose_to_eigen_transform(const geometry_msgs::Pose & p);
  Eigen::Affine3d           ros_to_eigen_transform(const geometry_msgs::Transform & p);
  Eigen::Affine3d           tf_transform_to_eigen_transform(const tf2::Transform & transform);

}  // namespace msg_conversions
