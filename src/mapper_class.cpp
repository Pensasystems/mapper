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

// Standard includes
#include <mapper/mapper_class.h>

#include <string>
#include <vector>

namespace mapper {

MapperClass::MapperClass() { }

MapperClass::~MapperClass() {
}

void MapperClass::TerminateNode() {
    terminate_node_ = true;

    // Destroy all subscribers
    this->DestroyAllCallbacks();

    // Join all threads
    this->WaitForThreadsToEnd();

    // destroy semaphores
    semaphores_.destroy();
}

void MapperClass::Initialize(ros::NodeHandle *nh) {
    // Load parameters
    double map_resolution, memory_time, max_range, min_range;
    double inflate_radius_xy, inflate_radius_z;
    double cam_fov, aspect_ratio;
    double occupancy_threshold, probability_hit, probability_miss;
    double clamping_threshold_max, clamping_threshold_min;
    double traj_resolution, compression_max_dev;
    bool process_pcl_at_startup, map_3d;
    nh->getParam("map_resolution", map_resolution);
    nh->getParam("max_range", max_range);
    nh->getParam("min_range", min_range);
    nh->getParam("memory_time", memory_time);
    nh->getParam("inflate_radius_xy", inflate_radius_xy);
    nh->getParam("inflate_radius_z", inflate_radius_z);
    nh->getParam("cam_fov", cam_fov);
    nh->getParam("cam_aspect_ratio", aspect_ratio);
    nh->getParam("occupancy_threshold", occupancy_threshold);
    nh->getParam("probability_hit", probability_hit);
    nh->getParam("probability_miss", probability_miss);
    nh->getParam("clamping_threshold_min", clamping_threshold_min);
    nh->getParam("clamping_threshold_max", clamping_threshold_max);
    nh->getParam("traj_compression_max_dev", compression_max_dev);
    nh->getParam("traj_compression_resolution", traj_resolution);
    nh->getParam("tf_update_rate", tf_update_rate_);
    nh->getParam("fading_memory_update_rate", fading_memory_update_rate_);
    nh->getParam("collision_check_rate", collision_check_rate_);

    // Get namespace of current node
    nh->getParam("namespace", ns_);

    // Load radius for radius collision-checking
    nh->getParam("radius_collision_check", radius_collision_check_);

    // Load depth camera names
    std::vector<std::string> depth_cam_names;
    std::string depth_cam_prefix, depth_cam_suffix;
    nh->getParam("depth_cam_names", depth_cam_names);
    nh->getParam("depth_cam_prefix", depth_cam_prefix);
    nh->getParam("depth_cam_suffix", depth_cam_suffix);

    // Load lidar topic names
    std::vector<std::string> lidar_names;
    std::string lidar_prefix, lidar_suffix;
    nh->getParam("lidar_names", lidar_names);
    nh->getParam("lidar_prefix", lidar_prefix);
    nh->getParam("lidar_suffix", lidar_suffix);

    // Load frame ids
    std::vector<std::string> cam_frame_id;
    std::vector<std::string> lidar_frame_id;
    nh->getParam("inertial_frame_id", inertial_frame_id_);
    nh->getParam("robot_frame_id", robot_frame_id_);
    nh->getParam("cam_frame_id", cam_frame_id);
    nh->getParam("lidar_frame_id", lidar_frame_id);

    // Check if number of cameras/lidar added match the number of frame_id for each of them
    if (depth_cam_names.size() != cam_frame_id.size()) {
        ROS_ERROR("Number of cameras is different from camera tf frame_ids!");
    }
    if (lidar_names.size() != lidar_frame_id.size()) {
        ROS_ERROR("Number of lidar topics is different from lidar tf frame_ids!");
    }

    // Load service names
    std::string resolution_srv_name, memory_time_srv_name;
    std::string map_inflation_srv_name, reset_map_srv_name, rrg_srv_name;
    std::string save_map_srv_name, load_map_srv_name, process_pcl_srv_name;
    nh->getParam("update_resolution", resolution_srv_name);
    nh->getParam("update_memory_time", memory_time_srv_name);
    nh->getParam("update_inflation_radius", map_inflation_srv_name);
    nh->getParam("reset_map", reset_map_srv_name);
    nh->getParam("save_map", save_map_srv_name);
    nh->getParam("load_map", load_map_srv_name);
    nh->getParam("process_pcl", process_pcl_srv_name);
    nh->getParam("rrg_service", rrg_srv_name);

    // Load publisher names
    std::string obstacle_markers_topic, free_space_markers_topic;
    std::string inflated_obstacle_markers_topic, inflated_free_space_markers_topic;
    std::string frustum_markers_topic, discrete_trajectory_markers_topic;
    std::string path_obstacle_detection_topic, graph_tree_marker_topic;
    std::string obstacle_radius_detection_topic, obstacle_radius_markers_topic;
    nh->getParam("obstacle_markers", obstacle_markers_topic);
    nh->getParam("free_space_markers", free_space_markers_topic);
    nh->getParam("inflated_obstacle_markers", inflated_obstacle_markers_topic);
    nh->getParam("inflated_free_space_markers", inflated_free_space_markers_topic);
    nh->getParam("frustum_markers", frustum_markers_topic);
    nh->getParam("obstacle_radius_markers", obstacle_radius_markers_topic);
    nh->getParam("discrete_trajectory_markers", discrete_trajectory_markers_topic);
    nh->getParam("path_obstacle_detection", path_obstacle_detection_topic);
    nh->getParam("obstacle_radius_detection", obstacle_radius_detection_topic);
    nh->getParam("graph_tree_marker_topic", graph_tree_marker_topic);

    // Load current package path
    nh->getParam("local_path", local_path_);

    // Parameter defining whether or not the map processes point clouds at startup
    nh->getParam("process_pcl_at_startup", process_pcl_at_startup);

    // Parameter to map in 3d or in planar 2d (z height set to 0)
    nh->getParam("map_3d", map_3d);

    // Set mapper to update on startup
    globals_.update_map = process_pcl_at_startup;

    // Set global variable to map 3d
    globals_.map_3d = map_3d;

    // Shutdown ROS if sigint is detected
    terminate_node_ = false;

    // update tree parameters
    globals_.octomap.SetResolution(map_resolution);
    globals_.octomap.SetMaxRange(max_range);
    globals_.octomap.SetMinRange(min_range);
    globals_.octomap.SetInertialFrame(inertial_frame_id_);
    globals_.octomap.SetMemory(memory_time);
    globals_.octomap.SetMapInflation(inflate_radius_xy, inflate_radius_z);
    globals_.octomap.SetCamFrustum(cam_fov, aspect_ratio);
    globals_.octomap.SetLidarRange(min_range, max_range);
    globals_.octomap.SetOccupancyThreshold(occupancy_threshold);
    globals_.octomap.SetHitMissProbabilities(probability_hit, probability_miss);
    globals_.octomap.SetClampingThresholds(clamping_threshold_min, clamping_threshold_max);
    globals_.octomap.SetMap3d(map_3d);

    // update trajectory discretization parameters (used in collision check)
    globals_.sampled_traj.SetMaxDev(compression_max_dev);
    globals_.sampled_traj.SetResolution(traj_resolution);
    globals_.sampled_traj.SetInertialFrame(inertial_frame_id_);

    // Set tf vector to have as many entries as the number of cameras
    globals_.tf_cameras2world.resize(depth_cam_names.size());
    globals_.tf_lidar2world.resize(lidar_names.size());
    h_cameras_tf_thread_.resize(depth_cam_names.size());
    h_lidar_tf_thread_.resize(lidar_names.size());
    cameras_sub_.resize(depth_cam_names.size());
    lidar_sub_.resize(lidar_names.size());

    // Create services ------------------------------------------
    resolution_srv_ = nh->advertiseService(
        resolution_srv_name, &MapperClass::UpdateResolution, this);
    memory_time_srv_ = nh->advertiseService(
        memory_time_srv_name, &MapperClass::UpdateMemoryTime, this);
    map_inflation_srv_ = nh->advertiseService(
        map_inflation_srv_name, &MapperClass::MapInflation, this);
    reset_map_srv_ = nh->advertiseService(
        reset_map_srv_name, &MapperClass::ResetMap, this);
    save_map_srv_ = nh->advertiseService(
        save_map_srv_name, &MapperClass::SaveMap, this);
    load_map_srv_ = nh->advertiseService(
        load_map_srv_name, &MapperClass::LoadMap, this);
    process_pcl_srv_ = nh->advertiseService(
        process_pcl_srv_name, &MapperClass::OctomapProcessPCL, this);
    rrg_srv_ = nh->advertiseService(
        rrg_srv_name, &MapperClass::RRGService, this);

    // Publishers -----------------------------------------------
    obstacle_path_pub_ =
        nh->advertise<pensa_msgs::ObstacleInPath>(path_obstacle_detection_topic, 10);
    obstacle_radius_pub_ =
        nh->advertise<std_msgs::Float32>(obstacle_radius_detection_topic, 10);
    obstacle_marker_pub_ =
        nh->advertise<visualization_msgs::MarkerArray>(obstacle_markers_topic, 10);
    free_space_marker_pub_ =
        nh->advertise<visualization_msgs::MarkerArray>(free_space_markers_topic, 10);
    inflated_obstacle_marker_pub_ =
        nh->advertise<visualization_msgs::MarkerArray>(inflated_obstacle_markers_topic, 10);
    inflated_free_space_marker_pub_ =
        nh->advertise<visualization_msgs::MarkerArray>(inflated_free_space_markers_topic, 10);
    path_marker_pub_ =
        nh->advertise<visualization_msgs::MarkerArray>(discrete_trajectory_markers_topic, 10);
    cam_frustum_pub_ =
        nh->advertise<visualization_msgs::Marker>(frustum_markers_topic, 10);
    obstacle_radius_marker_pub_ =
        nh->advertise<visualization_msgs::Marker>(obstacle_radius_markers_topic, 10);
    graph_tree_marker_pub_ =
        nh->advertise<visualization_msgs::Marker>(graph_tree_marker_topic, 10);

    // threads --------------------------------------------------
    h_octo_thread_ = std::thread(&MapperClass::OctomappingTask, this);
    h_fade_thread_ = std::thread(&MapperClass::FadeTask, this);
    h_collision_check_thread_ = std::thread(&MapperClass::PathCollisionCheckTask, this);
    h_radius_collision_thread_ = std::thread(&MapperClass::RadiusCollisionCheck, this);
    h_body_tf_thread_ = std::thread(&MapperClass::BodyTfTask, this, inertial_frame_id_, robot_frame_id_);
    // h_keyboard_thread_ = std::thread(&MapperClass::KeyboardTask, this);

    // Subscriber for trajectories
    waypoints_sub_ = nh->subscribe<pensa_msgs::WaypointSet>
        ("/drone_arbiter/check_collision", 1, &MapperClass::WaypointsCallback, this);
    trajectory_status_sub_ = nh->subscribe<pensa_msgs::trapezoidal_p2pActionFeedback>
        ("/trapezoidal_p2p_action/feedback", 1, &MapperClass::TrajectoryStatusCallback, this);

    // Camera subscribers and tf threads ----------------------------------------------
    for (uint i = 0; i < depth_cam_names.size(); i++) {
        std::string cam_topic = depth_cam_prefix + depth_cam_names[i] + depth_cam_suffix;
        cameras_sub_[i] = nh->subscribe<sensor_msgs::PointCloud2>
              (cam_topic, 10, boost::bind(&MapperClass::CameraPclCallback, this, _1, i));
        h_cameras_tf_thread_[i] = std::thread(&MapperClass::CameraTfTask, this, inertial_frame_id_, cam_frame_id[i], i);
        ROS_INFO("[mapper] Subscribed to camera topic: %s", cameras_sub_[i].getTopic().c_str());
    }

    // Lidar subscribers and tf threads ----------------------------------------------
    for (uint i = 0; i < lidar_names.size(); i++) {
        std::string lidar_topic = lidar_prefix + lidar_names[i] + lidar_suffix;
        lidar_sub_[i] = nh->subscribe<sensor_msgs::PointCloud2>
              (lidar_topic, 10, boost::bind(&MapperClass::LidarPclCallback, this, _1, i));
        h_lidar_tf_thread_[i] = std::thread(&MapperClass::LidarTfTask, this, inertial_frame_id_, lidar_frame_id[i], i);
        ROS_INFO("[mapper] Subscribed to lidar topic: %s", lidar_sub_[i].getTopic().c_str());
    }

    // Notify initialization complete
    ROS_DEBUG("Initialization complete");
}

geometry_msgs::Point MapperClass::GetTfBodyToWorld() {
    mutexes_.body_tf.lock();
        const tf::StampedTransform tf_body2world = globals_.tf_body2world;
    mutexes_.body_tf.unlock();
    return msg_conversions::tf_vector3_to_ros_point(tf_body2world.getOrigin());
}

geometry_msgs::Point MapperClass::GetCurrentSetPoint() {
    mutexes_.traj_status.lock();
        pensa_msgs::trapezoidal_p2pFeedback traj_status = globals_.traj_status;
    mutexes_.traj_status.unlock();
    return traj_status.current_position;
}

bool MapperClass::RobotPosProjectedOnTrajectory(const geometry_msgs::Point& robot_position,
                                                geometry_msgs::Point *robot_projected_on_traj) {
    mutexes_.sampled_traj.lock();
        bool success = globals_.sampled_traj.NearestPointInCompressedTraj(robot_position, robot_projected_on_traj);
    mutexes_.sampled_traj.unlock();
    return success;
}

void MapperClass::GetCollidingNodesPcl(const pcl::PointCloud<pcl::PointXYZ>& pcl,
                                       std::vector<octomap::point3d> *colliding_nodes) {
    mutexes_.octomap.lock();
        globals_.octomap.FindCollidingNodesInflated(pcl, colliding_nodes);
    mutexes_.octomap.unlock();
}

void MapperClass::GetOctomapResolution(double *octomap_resolution) {
    mutexes_.octomap.lock();
        *octomap_resolution = globals_.octomap.tree_inflated_.getResolution();
    mutexes_.octomap.unlock();
}

void MapperClass::PublishNearestCollision(const geometry_msgs::Point &nearest_collision,
                                          const double &collision_distance) {
    pensa_msgs::ObstacleInPath msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = inertial_frame_id_;
    msg.obstacle_position = nearest_collision;
    msg.obstacle_distance = collision_distance;
    msg.obstacle_exists = true;
    obstacle_path_pub_.publish(msg);
}

void MapperClass::PublishPathMarkers(const visualization_msgs::MarkerArray &collision_markers,
                                     const visualization_msgs::MarkerArray &traj_markers,
                                     const visualization_msgs::MarkerArray &samples_markers,
                                     const visualization_msgs::MarkerArray &compressed_samples_markers) {
    path_marker_pub_.publish(collision_markers);
    path_marker_pub_.publish(traj_markers);
    path_marker_pub_.publish(samples_markers);
    path_marker_pub_.publish(compressed_samples_markers);
}

void MapperClass::PublishRadiusMarkers(const Eigen::Vector3d &center,
                                       const double &radius) {
    visualization_msgs::Marker obstacle_radius_marker;
    std::string ns = "radius_obstacles";
    std_msgs::ColorRGBA color = visualization_functions::Color::Red();
    visualization_functions::VisualizeRange(center, radius,
        inertial_frame_id_, ns, color, &obstacle_radius_marker);
    obstacle_radius_marker_pub_.publish(obstacle_radius_marker);
}

// void MapperClass::LoadPathPlanningConfig(const std::string &path_planning_config_service) {
//     load_path_planning_config_client_
// }

// PLUGINLIB_EXPORT_CLASS(mapper::MapperClass, nodelet::Nodelet);

}  // namespace mapper
