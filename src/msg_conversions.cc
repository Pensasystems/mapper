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

#include <mapper/msg_conversions.h>

namespace msg_conversions {

Eigen::Vector3d ros_point_to_eigen_vector(const geometry_msgs::Point & p) {
  return Eigen::Vector3d(p.x, p.y, p.z);
}


Eigen::Vector3d ros_to_eigen_vector(const geometry_msgs::Vector3 & v) {
  return Eigen::Vector3d(v.x, v.y, v.z);
}

geometry_msgs::Vector3 eigen_to_ros_vector(const Eigen::Vector3d & v) {
  geometry_msgs::Vector3 n;
  n.x = v[0];
  n.y = v[1];
  n.z = v[2];
  return n;
}

geometry_msgs::Point eigen_to_ros_point(const Eigen::Vector3d & v) {
  geometry_msgs::Point n;
  n.x = v[0];
  n.y = v[1];
  n.z = v[2];
  return n;
}

void eigen_to_ros_point(const Eigen::Vector3d &eigen_pt,
                        geometry_msgs::Point *ros_pt) {
  ros_pt->x = eigen_pt[0];
  ros_pt->y = eigen_pt[1];
  ros_pt->z = eigen_pt[2];
}

pcl::PointXYZ eigen_to_pcl_point(const Eigen::Vector3d &pt) {
  return pcl::PointXYZ(pt[0], pt[1], pt[2]);
}

Eigen::Quaterniond ros_to_eigen_quat(const geometry_msgs::Quaternion & q) {
  return Eigen::Quaterniond(q.w, q.x, q.y, q.z);
}

geometry_msgs::Quaternion eigen_to_ros_quat(const Eigen::Quaterniond & q) {
  geometry_msgs::Quaternion out;
  out.x = q.x();
  out.y = q.y();
  out.z = q.z();
  out.w = q.w();
  return out;
}

geometry_msgs::Quaternion eigen_to_ros_quat(const Eigen::Vector4d & v) {
  geometry_msgs::Quaternion out;
  out.x = v.x();
  out.y = v.y();
  out.z = v.z();
  out.w = v.w();
  return out;
}

geometry_msgs::Vector3 array_to_ros_vector(float* array) {
  geometry_msgs::Vector3 v;
  v.x = array[0];
  v.y = array[1];
  v.z = array[2];
  return v;
}

geometry_msgs::Point array_to_ros_point(float* array) {
  geometry_msgs::Point v;
  v.x = array[0];
  v.y = array[1];
  v.z = array[2];
  return v;
}

geometry_msgs::Point set_ros_point(const double & x, const double & y, const double & z) {
  geometry_msgs::Point v;
  v.x = x; v.y = y; v.z = z;
  return v;
}

Eigen::Vector3d tf_vector3_to_eigen_vector(const tf::Vector3 & v) {
  return Eigen::Vector3d(v.getX(), v.getY(), v.getZ());
}

geometry_msgs::Point tf_vector3_to_ros_point(const tf::Vector3 & v) {
  return set_ros_point(v.getX(), v.getY(), v.getZ());
}

geometry_msgs::Quaternion array_to_ros_quat(float* array) {
  geometry_msgs::Quaternion q;
  q.x = array[0];
  q.y = array[1];
  q.z = array[2];
  q.w = array[3];
  return q;
}

geometry_msgs::Quaternion identity_quaternion() {
  geometry_msgs::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = 0.0;
  q.w = 1.0;
}

void ros_to_array_vector(const geometry_msgs::Vector3 & v, float* array) {
  array[0] = v.x;
  array[1] = v.y;
  array[2] = v.z;
}

void ros_to_array_point(const geometry_msgs::Point & p, float* array) {
  array[0] = p.x;
  array[1] = p.y;
  array[2] = p.z;
}

void ros_to_array_quat(const geometry_msgs::Quaternion & q, float* array) {
  array[0] = q.x;
  array[1] = q.y;
  array[2] = q.z;
  array[3] = q.w;
}

void eigen_to_array_vector(const Eigen::Vector3d & v, float* array) {
  array[0] = v.x();
  array[1] = v.y();
  array[2] = v.z();
}

void eigen_to_array_quat(const Eigen::Quaterniond & q, float* array) {
  array[0] = q.x();
  array[1] = q.y();
  array[2] = q.z();
  array[3] = q.w();
}

Eigen::Affine3d ros_pose_to_eigen_transform(const geometry_msgs::Pose & p) {
  Eigen::Affine3d transform;
  transform.translation() = Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
  transform.linear() = Eigen::Quaterniond(
    p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z).toRotationMatrix();
  return transform;
}

Eigen::Affine3d ros_to_eigen_transform(const geometry_msgs::Transform & p) {
  Eigen::Affine3d transform;
  transform.translation() = Eigen::Vector3d(p.translation.x, p.translation.y, p.translation.z);
  transform.linear() = Eigen::Quaterniond(
    p.rotation.w, p.rotation.x, p.rotation.y, p.rotation.z).toRotationMatrix();
  return transform;
}

}  // end namespace msg_conversions
