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
#include <octomap/ColorOcTree.h>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>

// ROS libraries
#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>
#include <tf/transform_broadcaster.h>

// Pensa messages/services
#include <pensa_msgs/SetFloat.h>
#include <pensa_msgs/RRT_RRG_PRM.h>
#include <pensa_msgs/ObstacleInPath.h>

// C++ libraries
#include <fstream>
#include <vector>
#include <string>
#include <exception>
#include <thread>         // std::thread

// Mapper message types
#include "mapper/Segment.h"
#include "mapper/ControlState.h"

// Pensa-ros msg types
#include "pensa_msgs/VecPVA_4d.h"
#include "pensa_msgs/WaypointSet.h"
#include <pensa_msgs/trapezoidal_p2pAction.h>

// Classes
#include "mapper/tf_class.h"
#include "mapper/octoclass.h"
#include "mapper/polynomials.h"
#include "mapper/sampled_trajectory.h"
#include "mapper/mutex_protected_variable.h"

// Data structures
#include "mapper/structs.h"

// My defined libraries
#include "mapper/visualization_functions.h"

namespace mapper {

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

  void PublishMarkers(const visualization_msgs::MarkerArray &collision_markers,
                      const visualization_msgs::MarkerArray &traj_markers,
                      const visualization_msgs::MarkerArray &samples_markers,
                      const visualization_msgs::MarkerArray &compressed_samples_markers);


  // Callbacks (see callbacks.cc for implementation) ----------------
  // Callback for handling incoming camera point cloud messages
  void CameraPclCallback(const sensor_msgs::PointCloud2::ConstPtr &msg,
                         const uint& cam_index);

  // Callback for handling incoming lidar point cloud messages
  void LidarPclCallback(const sensor_msgs::PointCloud2::ConstPtr &msg,
                         const uint& cam_index);

  // Callback for handling incoming new trajectory messages (astrobee type - deprecated)
  void SegmentCallback(const mapper::Segment::ConstPtr &msg);

  // Callback for handling incoming sampled trajectory
  void SampledTrajectoryCallback(const pensa_msgs::VecPVA_4d::ConstPtr &msg);

  // Callback for handling incoming waypoints
  void WaypointsCallback(const pensa_msgs::WaypointSetConstPtr &msg);

  // // Callback to know the current status of trajectory tracking
  void TrajectoryStatusCallback(const pensa_msgs::trapezoidal_p2pActionFeedbackConstPtr &msg);

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

  // Reset the map
  bool ResetMap(std_srvs::Trigger::Request &req,
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

  // RRG path planning
  bool RRGService(pensa_msgs::RRT_RRG_PRM::Request &req,
                  pensa_msgs::RRT_RRG_PRM::Response &res);

  // Threads (see threads.cc for implementation) -----------------
  // Thread for fading memory of the octomap
  void FadeTask();

  // Threads for constantly updating the tfTree values
  void BodyTfTask(const std::string& parent_frame,
                  const std::string& child_frame);
  void CameraTfTask(const std::string& parent_frame,
                    const std::string& child_frame,
                    const uint& index);  // Returns the transform from child to parent frame, expressed in parent frame
  void LidarTfTask(const std::string& parent_frame,
                   const std::string& child_frame,
                   const uint& index);  // Same as before, but for lidar data

  // Thread for collision checking
  void CollisionCheckTask();

  // Thread for getting pcl data and populating the octomap
  void OctomappingTask();

  // Thread for getting keyboard messages
  void KeyboardTask();

 private:
  // Declare global variables (structures defined in structs.h)
  globalVariables globals_;  // These variables are all mutex-protected
  mutexStruct mutexes_;
  semaphoreStruct semaphores_;

  // Thread variables
  std::thread h_octo_thread_, h_fade_thread_, h_collision_check_thread_;
  std::thread h_body_tf_thread_;
  std::vector<std::thread> h_cameras_tf_thread_;
  std::vector<std::thread> h_lidar_tf_thread_;

  // Subscriber variables
  ros::Subscriber trajectory_sub_, trajectory_status_sub_;
  ros::Subscriber waypoints_sub_;
  std::vector<ros::Subscriber> cameras_sub_;
  std::vector<ros::Subscriber> lidar_sub_;

  // Octomap services
  ros::ServiceServer resolution_srv_, memory_time_srv_;
  ros::ServiceServer map_inflation_srv_, reset_map_srv_;
  ros::ServiceServer save_map_srv_, load_map_srv_, process_pcl_srv_;

  // Thread rates (hz)
  double tf_update_rate_, fading_memory_update_rate_, collision_check_rate_;

  // Path planning services
  ros::ServiceServer rrg_srv_;

  // Node namespace
  std::string ns_;

  // Path strings
  std::string local_path_;

  // Inertial and robot frame ids
  std::string inertial_frame_id_, robot_frame_id_;

  // Publishers
  ros::Publisher obstacle_path_pub_;
  ros::Publisher obstacle_marker_pub_;
  ros::Publisher free_space_marker_pub_;
  ros::Publisher inflated_obstacle_marker_pub_;
  ros::Publisher inflated_free_space_marker_pub_;
  ros::Publisher path_marker_pub_;
  ros::Publisher cam_frustum_pub_;

  // Path planning publishers
  ros::Publisher graph_tree_marker_pub_;

  // Boolean to terminate all threads
  MutexProtectedVariable<bool> terminate_node_;
};

}  // namespace mapper
