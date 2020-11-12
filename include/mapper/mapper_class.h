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

// Octomap libraries
#include <octomap/octomap.h>

// PCL specific includes
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>

// ROS libraries
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <std_msgs/Float32.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>
#include <visualization_msgs/MarkerArray.h>

// Pensa messages/services
#include <pensa_msgs/Astar.h>
#include <pensa_msgs/ObstacleInPath.h>
#include <pensa_msgs/PathPlanningConfig.h>
#include <pensa_msgs/RRT_RRG_PRM.h>
#include <pensa_msgs/SetFloat.h>

// C++ libraries
#include <atomic>
#include <exception>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

// Pensa-ros msg types
#include "pensa_msgs/trapezoidal_p2pAction.h"
#include "pensa_msgs/VecPVA_4d.h"
#include "pensa_msgs/WaypointSet.h"

// Classes
#include "mapper/octoclass.h"
#include "mapper/polynomials.h"
#include "mapper/sampled_trajectory.h"
#include "mapper/tf_class.h"

// Data structures
#include "mapper/structs.h"

// My defined libraries
#include "mapper/visualization_functions.h"

namespace mapper {

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2,
                                                        geometry_msgs::PoseStamped> SyncPolicyApprox;
typedef message_filters::Synchronizer<SyncPolicyApprox> Sync;

class MapperClass {
 public:
  MapperClass();
  ~MapperClass();

  virtual void Initialize(ros::NodeHandle *nh);
  void TerminateNode();  // Signals for thread termination

 protected:
  // Functions within mapper_class.cc -------------------------------
  geometry_msgs::Point GetTfBodyToWorld();

  geometry_msgs::Point GetCurrentSetPoint();

  bool RobotPosProjectedOnTrajectory(const geometry_msgs::Point& robot_position,
                                     geometry_msgs::Point *robot_projected_on_traj);

  void GetCollidingNodesPcl(const pcl::PointCloud<pcl::PointXYZ>& pcl,
                            std::vector<octomap::point3d> *colliding_nodes);

  void GetOctomapResolution(double *octomap_resolution);

  void PublishNearestCollision(const geometry_msgs::Point &nearest_collision,
                               const double &collision_distance);

  void PublishPathMarkers(const visualization_msgs::MarkerArray &collision_markers,
                          const visualization_msgs::MarkerArray &traj_markers,
                          const visualization_msgs::MarkerArray &samples_markers,
                          const visualization_msgs::MarkerArray &compressed_samples_markers);

  void PublishRadiusMarkers(const Eigen::Vector3d &center,
                            const double &radius);

  // Publish no-fly-zones and planning zone for Rviz visualization
  void PublishPathPlanningConfigMarkers();

  // Publish markers of the planned trajectory (Rviz visualization)
  void PublishPathPlanningPathMarkers(const std::vector<Eigen::Vector3d> &path,
                                      const std::vector<Eigen::Vector3d> &pruned_path,
                                      const std::string &inertial_frame_id);

  // Calls store_arbiter's service to retrieve path planning config
  void LoadPathPlanningConfig(const std::string &inertial_frame_id,
                              pensa_msgs::PathPlanningConfig *path_planning_config,
                              ros::NodeHandle *nh);

  // Callbacks (see callbacks.cc for implementation) ----------------
  // Callback that subscribes to both lidar and base link localization measurements
  void LidarSyncCallback(const sensor_msgs::PointCloud2::ConstPtr &lidar_msg,
                         const geometry_msgs::PoseStamped::ConstPtr &base_link_pose_msg);

  // Callback for handling incoming sampled trajectory
  void SampledTrajectoryCallback(const pensa_msgs::VecPVA_4d::ConstPtr &msg);

  // Callback for handling incoming waypoints
  void WaypointsCallback(const pensa_msgs::WaypointSetConstPtr &msg);

  // Callback to know the current status of trajectory tracking
  void TrajectoryStatusCallback(const pensa_msgs::trapezoidal_p2pActionFeedbackConstPtr &msg);

  // Destroy all callbacks (used when killing the node)
  void DestroyAllCallbacks();

  // Services (see services.cc for implementation) -----------------
  // Update resolution of the map
  bool UpdateResolution(pensa_msgs::SetFloat::Request &req,
                        pensa_msgs::SetFloat::Response &res);

  // Update map memory time
  bool UpdateMemoryTime(pensa_msgs::SetFloat::Request &req,
                        pensa_msgs::SetFloat::Response &res);

  // Update map inflation
  bool MapInflation(pensa_msgs::SetFloat::Request &req,
                    pensa_msgs::SetFloat::Response &res);

  // Reset the map and update the map's memory time through the SetFloat parameter
  bool ResetMap(pensa_msgs::SetFloat::Request &req,
                pensa_msgs::SetFloat::Response &res);

  // Reset the map and load it as the path planning config
  bool InitializeMapToPathPlanningConfig(std_srvs::Trigger::Request &req,
                                         std_srvs::Trigger::Response &res);

  // Save octomap
  bool SaveMap(std_srvs::Trigger::Request &req,
               std_srvs::Trigger::Response &res);

  // Load octomap
  bool LoadMap(std_srvs::Trigger::Request &req,
               std_srvs::Trigger::Response &res);

  // Process PCL data or not
  bool OctomapProcessPCL(std_srvs::SetBool::Request &req,
                         std_srvs::SetBool::Response &res);

  // A* path planning service
  bool AStarService(pensa_msgs::Astar::Request &req,
                    pensa_msgs::Astar::Response &res);

  // Service to clear A* path visualization in Rviz
  bool ClearAstarTrajectoryInRviz(std_srvs::Trigger::Request &req,
                                  std_srvs::Trigger::Response &res);

  // RRG path planning
  bool RRGService(pensa_msgs::RRT_RRG_PRM::Request &req,
                  pensa_msgs::RRT_RRG_PRM::Response &res);

  // Threads (see threads.cc for implementation) -----------------
  // Thread for fading memory of the octomap
  void FadeTask();

  // Thread for collision checking along the robot's path
  void PathCollisionCheckTask();

  // Thread for collision checking around the robot
  void RadiusCollisionCheck();

  // Thread for getting pcl data and populating the octomap
  void OctomappingTask();

  // Thread for getting keyboard messages
  void KeyboardTask();

  // Join all threads
  void WaitForThreadsToEnd();

 private:
  // Declare global variables (structures defined in structs.h)
  globalVariables globals_;  // These variables are all mutex-protected
  mutexStruct mutexes_;
  semaphoreStruct semaphores_;

  // Thread variables
  std::thread h_octo_thread_, h_fade_thread_, h_collision_check_thread_;
  std::thread h_radius_collision_thread_;

  // Subscriber variables
  ros::Subscriber trajectory_sub_, trajectory_status_sub_;
  ros::Subscriber waypoints_sub_;
  std::vector<ros::Subscriber> cameras_sub_;
  std::vector<ros::Subscriber> lidar_sub_;

  // Synchronized subscribers
  boost::shared_ptr<Sync> sync_;
  message_filters::Subscriber<sensor_msgs::PointCloud2> *lidar_sync_sub_;
  message_filters::Subscriber<geometry_msgs::PoseStamped> *base_link_pose_sync_sub_;

  // Octomap services
  ros::ServiceServer resolution_srv_, memory_time_srv_;
  ros::ServiceServer map_inflation_srv_, reset_map_srv_;
  ros::ServiceServer initialize_map_to_path_planning_config_srv_;
  ros::ServiceServer save_map_srv_, load_map_srv_, process_pcl_srv_;
  ros::ServiceServer a_star_path_planning_srv_;
  ros::ServiceServer clear_a_star_visualization_rviz_srv_;

  // Service clients
  ros::ServiceClient load_path_planning_config_client_;

  // Thread rates (hz)
  double fading_memory_update_rate_, collision_check_rate_;

  // Collision checking parameters
  double radius_collision_check_;

  // Path planning services
  ros::ServiceServer rrg_srv_;

  // Node namespace
  std::string ns_;

  // Path strings
  std::string local_path_;

  // Inertial and robot frame ids
  std::string inertial_frame_id_;

  // Variable for transform between base_link and lidar
  tf2::Transform tf_lidar_in_base_link_frame_;

  // Collision publishers
  ros::Publisher obstacle_path_pub_, obstacle_radius_pub_;

  // Marker publishers
  ros::Publisher obstacle_marker_pub_;
  ros::Publisher free_space_marker_pub_;
  ros::Publisher inflated_obstacle_marker_pub_;
  ros::Publisher inflated_free_space_marker_pub_;
  ros::Publisher path_marker_pub_;
  ros::Publisher cam_frustum_pub_;
  ros::Publisher obstacle_radius_marker_pub_;
  ros::Publisher path_planning_config_pub_;
  ros::Publisher path_planning_path_marker_pub_;

  // Path planning publishers
  ros::Publisher graph_tree_marker_pub_;

  // Path planning config (for Rviz visualization)
  pensa_msgs::PathPlanningConfig path_planning_config_;

  // Boolean to terminate all threads
  std::atomic<bool> terminate_node_;
};

}  // namespace mapper
