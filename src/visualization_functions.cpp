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

#include "mapper/msg_conversions.h"
#include "mapper/visualization_functions.h"

#include <set>
#include <string>
#include <vector>

namespace visualization_functions {

// Overwrites the given properties of the marker array.
void SetMarkerProperties(const std_msgs::Header &header,
                         const double &life_time,
                         visualization_msgs::MarkerArray* markers) {
  int count = 0;
  for (visualization_msgs::Marker& marker : markers->markers) {
    marker.header = header;
    marker.id = count;
    marker.lifetime = ros::Duration(life_time);
    ++count;
  }
}


void DeleteMarkersTemplate(const std::string &frame_id,
                           visualization_msgs::MarkerArray *marker_array) {
  visualization_msgs::Marker delete_marker;
  delete_marker.action = visualization_msgs::Marker::DELETEALL;
  delete_marker.scale.x = 0.1;
  delete_marker.scale.y = 0.1;
  delete_marker.scale.z = 0.1;
  delete_marker.header.frame_id = frame_id;
  delete_marker.ns = "";
  marker_array->markers.push_back(delete_marker);
}

void SetMarkersForDeletion(visualization_msgs::MarkerArray* marker_array) {
  for (uint i = 0; i < marker_array->markers.size(); i++) {
    marker_array->markers[i].action = visualization_msgs::Marker::DELETE;
  }
}

void DrawObstacleNodes(const std::vector<Eigen::Vector3d> &points,
                       const std::string &frame_id,
                       const double &resolution,
                       visualization_msgs::MarkerArray* marker_array) {
  DrawNodes(points, frame_id, "obstacle", resolution, Color::Blue(), 0.1, marker_array);
}

void DrawTreeNodes(const std::vector<Eigen::Vector3d> &points,
                   const std::string &frame_id,
                   const double &resolution,
                   visualization_msgs::MarkerArray* marker_array) {
  DrawNodes(points, frame_id, "path", resolution, Color::Orange(), 0.1, marker_array);
}

void DrawCollidingNodes(const std::vector<Eigen::Vector3d> &points,
                        const std::string &frame_id,
                        const double &resolution,
                        visualization_msgs::MarkerArray* marker_array) {
  marker_array->markers.clear();
  DrawNodes(points, frame_id, "collision", resolution, Color::Red(), 0.9, marker_array);
}

void DrawCollidingNodes(const std::vector<octomap::point3d> &points,
                        const std::string &frame_id,
                        const double &resolution,
                        visualization_msgs::MarkerArray *marker_array) {
  marker_array->markers.clear();
  DrawNodes(points, frame_id, "collision", resolution, Color::Red(), 0.9, marker_array);
}

void DrawNodes(const std::vector<Eigen::Vector3d> &points,
               const std::string &frame_id,
               const std::string &ns,  // namespace
               const double &resolution,
               const std_msgs::ColorRGBA &color,
               const double &transparency,  // 0 -> transparent, 1 -> opaque
               visualization_msgs::MarkerArray *marker_array) {
  // marker_array->markers.clear();

  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::CUBE_LIST;
  marker.color = color;
  marker.color.a = transparency;
  marker.scale.x = resolution;
  marker.scale.y = resolution;
  marker.scale.z = resolution;
  marker.ns = ns;
  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time::now();
  marker.pose.orientation.w = 1.0;
  marker.header.seq = 0;
  marker.id = 0;

  // Get the number of requested waypoints
  uint n_w = points.size();

  if (n_w == 0) {
    marker.action = visualization_msgs::Marker::DELETE;
  } else {
    marker.action = visualization_msgs::Marker::ADD;
  }

  for (size_t i = 0; i < n_w; ++i) {
    geometry_msgs::Point new_point;
    new_point.x = points[i](0);
    new_point.y = points[i](1);
    new_point.z = points[i](2);
    marker.points.push_back(new_point);
  }
  marker_array->markers.push_back(marker);

  std_msgs::Header header;
  header.frame_id = frame_id;
  header.stamp = ros::Time::now();
  SetMarkerProperties(header, 1.0, marker_array);
}

void DrawNodes(const std::vector<octomap::point3d> &points,
               const std::string &frame_id,
               const std::string &ns,  // namespace
               const double &resolution,
               const std_msgs::ColorRGBA &color,
               const double &transparency,  // 0 -> transparent, 1 -> opaque
               visualization_msgs::MarkerArray *marker_array) {
  // marker_array->markers.clear();

  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::CUBE_LIST;
  marker.color = color;
  marker.color.a = transparency;
  marker.scale.x = resolution;
  marker.scale.y = resolution;
  marker.scale.z = resolution;
  marker.ns = ns;
  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time::now();
  marker.pose.orientation.w = 1.0;
  marker.header.seq = 0;
  marker.id = 0;

  // Get the number of requested waypoints
  uint n_w = points.size();

  if (n_w == 0) {
    marker.action = visualization_msgs::Marker::DELETE;
  } else {
    marker.action = visualization_msgs::Marker::ADD;
  }

  for (size_t i = 0; i < n_w; ++i) {
    geometry_msgs::Point new_point;
    new_point.x = points[i].x();
    new_point.y = points[i].y();
    new_point.z = points[i].z();
    marker.points.push_back(new_point);
  }
  marker_array->markers.push_back(marker);

  std_msgs::Header header;
  header.frame_id = frame_id;
  header.stamp = ros::Time::now();
  SetMarkerProperties(header, 1.0, marker_array);
}

void MarkerNode(const Eigen::Vector3d &point,
                const std::string &frame_id,
                const std::string &ns,  // namespace
                const double &resolution,
                const std_msgs::ColorRGBA &color,
                const double &transparency,  // 0 -> transparent, 1 -> opaque
                const int &seqNumber,
                visualization_msgs::Marker* marker) {
  marker->type = visualization_msgs::Marker::CUBE;
  marker->action = visualization_msgs::Marker::ADD;
  marker->color = color;
  marker->color.a = transparency;
  marker->scale.x = resolution;
  marker->scale.y = resolution;
  marker->scale.z = resolution;
  marker->ns = ns;
  marker->header.frame_id = frame_id;
  marker->header.stamp = ros::Time::now();
  marker->pose.orientation.w = 1.0;

  geometry_msgs::Point position_msg;
  position_msg.x = point(0);
  position_msg.y = point(1);
  position_msg.z = point(2);
  marker->pose.position = position_msg;
  marker->header.seq = seqNumber;
}

void PathVisualization(const std::vector<Eigen::Vector3d> &total_path,
                       const std::vector<Eigen::Vector3d> &waypoints,
                       visualization_msgs::MarkerArray *markers) {
    PathVisualization(total_path, waypoints, Color::Green(), "/Path", markers);
}

void PathVisualization(const std::vector<Eigen::Vector3d> &total_path,
                       const std::vector<Eigen::Vector3d> &waypoints,
                       const std_msgs::ColorRGBA &color,
                       const std::string &ns,  // namespace
                       visualization_msgs::MarkerArray *markers) {
    // Initialize edges marker
    visualization_msgs::Marker line_list;
    line_list.header.frame_id = "/world";
    line_list.header.stamp = ros::Time::now();
    line_list.ns = ns;
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.id = 0;
    line_list.scale.x = 0.02;  // Line width
    line_list.pose.orientation.w = 1.0;
    line_list.color = color;

    // Populate edges
    uint path_n_points = total_path.size();
    geometry_msgs::Point node;
    if (path_n_points >= 2) {
        node = msg_conversions::eigen_to_ros_point(total_path[0]);
        line_list.points.push_back(node);
        for (uint i = 1; i < total_path.size()-1; i++) {
            node = msg_conversions::eigen_to_ros_point(total_path[i]);
            line_list.points.push_back(node);
            line_list.points.push_back(node);
        }
        node = msg_conversions::eigen_to_ros_point(total_path[total_path.size()-1]);
        line_list.points.push_back(node);
    }

    // Populate the waypoints
    visualization_msgs::Marker endpoints;
    endpoints.header.frame_id = "/world";
    endpoints.header.stamp = ros::Time::now();
    endpoints.ns = ns;
    endpoints.action = visualization_msgs::Marker::ADD;
    endpoints.type = visualization_msgs::Marker::SPHERE_LIST;
    endpoints.id = 1;
    endpoints.scale.x = 0.1;
    endpoints.scale.y = 0.1;
    endpoints.scale.z = 0.1;
    endpoints.pose.orientation.w = 1.0;
    endpoints.color = Color::Blue();
    for (uint i = 0; i < waypoints.size(); i++) {
        node = msg_conversions::eigen_to_ros_point(waypoints[i]);
        endpoints.points.push_back(node);
    }

    markers->markers.push_back(endpoints);
    markers->markers.push_back(line_list);
}

void DrawArrowPoints(const Eigen::Vector3d& p1,
                     const Eigen::Vector3d& p2,
                     const std_msgs::ColorRGBA &color,
                     const double &diameter,
                     visualization_msgs::Marker* marker) {
  marker->type = visualization_msgs::Marker::ARROW;
  marker->action = visualization_msgs::Marker::ADD;
  marker->color = color;

  marker->points.resize(2);
  marker->points[0] = msg_conversions::eigen_to_ros_point(p1);
  marker->points[1] = msg_conversions::eigen_to_ros_point(p2);
  // EigenPoint2RosPoint(p1, &marker->points[0]);
  // EigenPoint2RosPoint(p2, &marker->points[1]);

  marker->scale.x = diameter * 0.1;
  marker->scale.y = diameter * 2 * 0.1;
  marker->scale.z = 0.1;
  marker->pose.orientation.w = 1.0;
}

void SphereVisMarker(const geometry_msgs::Point &pos,
                     const std::string &frame_id,
                     const std::string &name_space,
                     const double &diameter,
                     const std_msgs::ColorRGBA &color,
                     visualization_msgs::Marker *marker) {
    // Set position
    marker->points.push_back(pos);
    marker->colors.push_back(color);

    // Set marker properties
    marker->header.frame_id = frame_id;
    marker->header.stamp = ros::Time::now();
    marker->ns = name_space;
    marker->id = 0;
    marker->type = visualization_msgs::Marker::SPHERE_LIST;
    marker->scale = msg_conversions::set_ros_vector(diameter, diameter, diameter);
    marker->pose.orientation.w = 1.0;
    marker->lifetime = ros::Duration(1.0);
    marker->action = visualization_msgs::Marker::ADD;
}

void ReferenceVisMarker(const geometry_msgs::Point &pos,
                        const std::string& frame_id,
                        visualization_msgs::MarkerArray* marker_array) {
    visualization_msgs::Marker marker;

    // Set color parameters
    std_msgs::ColorRGBA color = Color::Red();
    color.a = 0.9;
    const std::string ns = "ReferencePos";
    const double diameter = 0.15;

    SphereVisMarker(pos, frame_id, ns, diameter, color, &marker);

    marker_array->markers.push_back(marker);
}

void RobotPosVisMarker(const geometry_msgs::Point &pos,
                       const std::string &frame_id,
                       visualization_msgs::MarkerArray* marker_array) {
    visualization_msgs::Marker marker;

    // Set color parameters
    std_msgs::ColorRGBA color = Color::Green();
    color.a = 0.9;
    const std::string ns = "RobotPos";
    const double diameter = 0.15;

    SphereVisMarker(pos, frame_id, ns, diameter, color, &marker);

    marker_array->markers.push_back(marker);
}

void ProjectedPosVisMarker(const geometry_msgs::Point &pos,
                           const std::string &frame_id,
                           visualization_msgs::MarkerArray* marker_array) {
    visualization_msgs::Marker marker;

    // Set color parameters
    std_msgs::ColorRGBA color = Color::Blue();
    color.a = 0.9;
    const std::string ns = "ProjectedRobotPos";
    const double diameter = 0.15;

    SphereVisMarker(pos, frame_id, ns, diameter, color, &marker);

    marker_array->markers.push_back(marker);
}

// Markers for trajectory
void TrajVisMarkers(const pcl::PointCloud<pcl::PointXYZ> &pcl_traj,
                    const std::string &frame_id,
                    const double &size,
                    visualization_msgs::MarkerArray* marker_array) {
    // Markers: each marker array stores a set of nodes with similar size
    // visualization_msgs::MarkerArray occupiedNodesVis;
    const ros::Time rostime = ros::Time::now();

    // Set color parameters
    std_msgs::ColorRGBA color;
    color = Color::Green();
    color.a = 0.3;

    // Publish all leafs from the tree
    visualization_msgs::Marker markers;
    markers.header.frame_id = frame_id;
    markers.header.stamp = rostime;
    markers.ns = "thick_traj_shifted";
    markers.id = 0;
    markers.type = visualization_msgs::Marker::CUBE_LIST;
    markers.scale.x = size;
    markers.scale.y = size;
    markers.scale.z = size;
    markers.color = color;
    markers.pose.orientation.w = 1.0;

    if (pcl_traj.size() > 0) {
      markers.action = visualization_msgs::Marker::ADD;
    } else {
      markers.action = visualization_msgs::Marker::DELETE;
    }

    for (uint i = 0; i < pcl_traj.size(); i++) {
      geometry_msgs::Point point_center =
          msg_conversions::pcl_to_ros_vector(pcl_traj.points[i]);
      markers.points.push_back(point_center);
      markers.colors.push_back(color);
    }

    marker_array->markers.push_back(markers);
}

void VisualizeRange(const Eigen::Vector3d &pos,
                    const double &radius,
                    const std::string &frame_id,
                    const std::string &ns,
                    const std_msgs::ColorRGBA &color,
                    visualization_msgs::Marker *range_marker) {
    // Initialize array
    range_marker->header.frame_id = frame_id;
    range_marker->header.stamp = ros::Time::now();
    range_marker->ns = ns;
    range_marker->action = visualization_msgs::Marker::ADD;
    range_marker->pose.position = msg_conversions::eigen_to_ros_point(pos);
    range_marker->pose.orientation = msg_conversions::identity_quaternion();
    range_marker->type = visualization_msgs::Marker::CYLINDER;
    range_marker->id = 0;
    range_marker->scale.x = 2*radius;
    range_marker->scale.y = 2*radius;
    range_marker->scale.z = 0.01;
    range_marker->color = color;
    range_marker->lifetime = ros::Duration(1);  // Disappears in 1 second
}

void SetCuboidMarker(const Eigen::Vector3d &center,
                     const double &x_dim,
                     const double &y_dim,
                     const double &z_dim,
                     const std::string &frame_id,
                     const std::string &ns,  // namespace
                     const uint &id,
                     const std_msgs::ColorRGBA &color,
                     const double &lifetime,
                     visualization_msgs::Marker *cuboid_marker) {
    // Initialize array
    cuboid_marker->header.frame_id = frame_id;
    cuboid_marker->header.stamp = ros::Time::now();
    cuboid_marker->ns = ns;
    cuboid_marker->action = visualization_msgs::Marker::ADD;
    cuboid_marker->pose.position = msg_conversions::eigen_to_ros_point(center);
    cuboid_marker->pose.orientation = msg_conversions::identity_quaternion();
    cuboid_marker->type = visualization_msgs::Marker::CUBE;
    cuboid_marker->id = id;
    cuboid_marker->scale.x = x_dim;
    cuboid_marker->scale.y = y_dim;
    cuboid_marker->scale.z = z_dim;
    cuboid_marker->color = color;
    cuboid_marker->lifetime = ros::Duration(lifetime);
}

void DrawPathPlanningConfig(const pensa_msgs::PathPlanningConfig &path_planning_config,
                            const std::string &ns,  // namespace
                            const std::string &inertial_frame_id,
                            const std_msgs::ColorRGBA &no_fly_zone_color,
                            const std_msgs::ColorRGBA &fly_zone_color,
                            const double &thickness,  // in meters
                            visualization_msgs::MarkerArray *marker_array) {
  const double lifetime = 0.0;  // Should not disappear over time
  uint id = 0;

  // Markers for no-fly-zones
  for (const auto& no_fly_zone : path_planning_config.no_fly_zones) {
    // Compute marker dimensions
    const double x_center = 0.5*(no_fly_zone.x_corner1 + no_fly_zone.x_corner2);
    const double y_center = 0.5*(no_fly_zone.y_corner1 + no_fly_zone.y_corner2);
    const double x_width = std::fabs(no_fly_zone.x_corner1 - no_fly_zone.x_corner2);
    const double y_width = std::fabs(no_fly_zone.y_corner1 - no_fly_zone.y_corner2);
    const double z_width = thickness;
    Eigen::Vector3d center(x_center, y_center, 0.0);

    // Get marker
    visualization_msgs::Marker no_fly_zone_marker;
    SetCuboidMarker(center, x_width, y_width, z_width, no_fly_zone.frame_id,
                    ns, id++, no_fly_zone_color, lifetime, &no_fly_zone_marker);
    marker_array->markers.push_back(no_fly_zone_marker);
  }

  // Markers for allowed area
  const double x_center = 0.5*(path_planning_config.map_x_corner1 + path_planning_config.map_x_corner2);
  const double y_center = 0.5*(path_planning_config.map_y_corner1 + path_planning_config.map_y_corner2);
  const double x_width = std::fabs(path_planning_config.map_x_corner1 - path_planning_config.map_x_corner2);
  const double y_width = std::fabs(path_planning_config.map_y_corner1 - path_planning_config.map_y_corner2);
  const double z_width = thickness;
  Eigen::Vector3d center(x_center, y_center, 0.0);

  // Get marker
  visualization_msgs::Marker fly_zone_marker;
  SetCuboidMarker(center, x_width, y_width, z_width, inertial_frame_id,
                  ns, id++, fly_zone_color, lifetime, &fly_zone_marker);
  marker_array->markers.push_back(fly_zone_marker);
}

}  // namespace visualization_functions
