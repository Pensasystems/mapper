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

// Local includes
#include <mapper/mapper_class.h>

// Pensa includes
#include <pensa_msgs/PathPlanningConfigInFrame.h>

// Cpp includes
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
    nh->getParam("fading_memory_update_rate", fading_memory_update_rate_);
    nh->getParam("collision_check_rate", collision_check_rate_);

    // Load radius for radius collision-checking
    nh->getParam("radius_collision_check", radius_collision_check_);

    // Load lidar and base_link pose topics
    std::string lidar_topic, base_link_pose_topic;
    nh->getParam("lidar_topic", lidar_topic);
    nh->getParam("base_link_pose_topic", base_link_pose_topic);

    // Load frame ids
    std::string lidar_frame_id, robot_base_link_frame_id;
    nh->getParam("inertial_frame_id", inertial_frame_id_);
    nh->getParam("robot_base_link_frame_id", robot_base_link_frame_id);
    nh->getParam("lidar_frame_id", lidar_frame_id);

    // Load transform between base_link and lidar
    if (!helper::LookupTransform(robot_base_link_frame_id, lidar_frame_id, &tf_lidar_in_base_link_frame_)) {
        return;
    }

    // Load service names
    std::string resolution_srv_name, memory_time_srv_name;
    std::string map_inflation_srv_name, reset_map_srv_name, rrg_srv_name;
    std::string save_map_srv_name, load_map_srv_name, process_pcl_srv_name;
    std::string initialize_map_to_path_planning_config_srv_name;
    std::string a_star_path_planning_srv_name;
    std::string clear_a_star_visualization_rviz_srv_name;
    nh->getParam("update_resolution", resolution_srv_name);
    nh->getParam("update_memory_time", memory_time_srv_name);
    nh->getParam("update_inflation_radius", map_inflation_srv_name);
    nh->getParam("reset_map", reset_map_srv_name);
    nh->getParam("save_map", save_map_srv_name);
    nh->getParam("load_map", load_map_srv_name);
    nh->getParam("initialize_map_to_path_planning_config", initialize_map_to_path_planning_config_srv_name);
    nh->getParam("process_pcl", process_pcl_srv_name);
    nh->getParam("a_star_path_planning", a_star_path_planning_srv_name);
    nh->getParam("clear_a_star_visualization_rviz", clear_a_star_visualization_rviz_srv_name);
    nh->getParam("rrg_service", rrg_srv_name);

    // Load publisher names
    std::string obstacle_markers_topic, free_space_markers_topic;
    std::string inflated_obstacle_markers_topic, inflated_free_space_markers_topic;
    std::string frustum_markers_topic, discrete_trajectory_markers_topic;
    std::string path_obstacle_detection_topic, graph_tree_marker_topic;
    std::string obstacle_radius_detection_topic, obstacle_radius_markers_topic;
    std::string path_planning_config_markers_topic, path_planning_path_markers_topic;
    nh->getParam("obstacle_markers", obstacle_markers_topic);
    nh->getParam("free_space_markers", free_space_markers_topic);
    nh->getParam("inflated_obstacle_markers", inflated_obstacle_markers_topic);
    nh->getParam("inflated_free_space_markers", inflated_free_space_markers_topic);
    nh->getParam("frustum_markers", frustum_markers_topic);
    nh->getParam("obstacle_radius_markers", obstacle_radius_markers_topic);
    nh->getParam("discrete_trajectory_markers", discrete_trajectory_markers_topic);
    nh->getParam("path_obstacle_detection", path_obstacle_detection_topic);
    nh->getParam("obstacle_radius_detection", obstacle_radius_detection_topic);
    nh->getParam("path_planning_config_markers", path_planning_config_markers_topic);
    nh->getParam("path_planning_path_markers", path_planning_path_markers_topic);
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

    // Load path planning config
    double desired_obstacle_planning_distance;
    nh->getParam("desired_obstacle_planning_distance", desired_obstacle_planning_distance);
    this->LoadPathPlanningConfig(inertial_frame_id_, &path_planning_config_, nh);

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
    globals_.octomap.SetPathPlanningConfig(path_planning_config_, desired_obstacle_planning_distance);

    // update trajectory discretization parameters (used in collision check)
    globals_.sampled_traj.SetMaxDev(compression_max_dev);
    globals_.sampled_traj.SetResolution(traj_resolution);
    globals_.sampled_traj.SetInertialFrame(inertial_frame_id_);

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
    initialize_map_to_path_planning_config_srv_ = nh->advertiseService(
        initialize_map_to_path_planning_config_srv_name, &MapperClass::InitializeMapToPathPlanningConfig, this);
    process_pcl_srv_ = nh->advertiseService(
        process_pcl_srv_name, &MapperClass::OctomapProcessPCL, this);
    a_star_path_planning_srv_ = nh->advertiseService(
        a_star_path_planning_srv_name, &MapperClass::AStarService, this);
    clear_a_star_visualization_rviz_srv_ = nh->advertiseService(
        clear_a_star_visualization_rviz_srv_name, &MapperClass::ClearAstarTrajectoryInRviz, this);
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
    path_planning_config_pub_ =
        nh->advertise<visualization_msgs::MarkerArray>(path_planning_config_markers_topic, 10, true);
    path_planning_path_marker_pub_ =
        nh->advertise<visualization_msgs::MarkerArray>(path_planning_path_markers_topic, 10, true);

    // Publish no-fly-zones for Rviz visualization
    this->PublishPathPlanningConfigMarkers();

    // threads --------------------------------------------------
    h_octo_thread_ = std::thread(&MapperClass::OctomappingTask, this);
    h_fade_thread_ = std::thread(&MapperClass::FadeTask, this);
    h_collision_check_thread_ = std::thread(&MapperClass::PathCollisionCheckTask, this);
    h_radius_collision_thread_ = std::thread(&MapperClass::RadiusCollisionCheck, this);
    // h_keyboard_thread_ = std::thread(&MapperClass::KeyboardTask, this);

    // Subscribers -----------------------------------------------
    waypoints_sub_ = nh->subscribe<pensa_msgs::WaypointSet>
        ("/drone_arbiter/check_collision", 1, &MapperClass::WaypointsCallback, this);
    trajectory_status_sub_ = nh->subscribe<pensa_msgs::trapezoidal_p2pActionFeedback>
        ("/trapezoidal_p2p_action/feedback", 1, &MapperClass::TrajectoryStatusCallback, this);
    base_link_pose_sub_ = nh->subscribe<geometry_msgs::PoseStamped>
        (base_link_pose_topic, 5, &MapperClass::BaseLinkPoseCallback, this);

    // Lidar synchronized subscriber
    lidar_sync_sub_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(*nh, lidar_topic, 10);
    base_link_pose_sync_sub_ =
        new message_filters::Subscriber<geometry_msgs::PoseStamped>(*nh, base_link_pose_topic, 10);
    sync_.reset(new Sync(SyncPolicyApprox(10), *lidar_sync_sub_, *base_link_pose_sync_sub_));
    sync_->registerCallback(boost::bind(&MapperClass::LidarSyncCallback, this, _1, _2));

    // Notify initialization complete
    ROS_DEBUG("Initialization complete");
}

geometry_msgs::Point MapperClass::GetTfBodyToWorld() {
    mutexes_.body_tf.lock();
        const tf2::Transform tf_body2world = globals_.tf_body2world;
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

void MapperClass::PublishPathPlanningConfigMarkers() {
    static const std::string ns = "path_planning_config";
    static const double thickness = 0.1;
    std_msgs::ColorRGBA no_fly_zone_color = visualization_functions::Color::Red();
    std_msgs::ColorRGBA fly_zone_color = visualization_functions::Color::Green();
    no_fly_zone_color.a = 0.3;  // make them slightly transparent
    fly_zone_color.a = 0.15;    // make them slightly transparent
    visualization_msgs::MarkerArray no_fly_zones_markers;
    visualization_functions::DrawPathPlanningConfig(path_planning_config_, ns, inertial_frame_id_, no_fly_zone_color,
                                                    fly_zone_color, thickness, &no_fly_zones_markers);
    path_planning_config_pub_.publish(no_fly_zones_markers);
}

void MapperClass::PublishPathPlanningPathMarkers(const std::vector<Eigen::Vector3d> &path,
                                                 const std::vector<Eigen::Vector3d> &pruned_path,
                                                 const std::string &inertial_frame_id) {
    visualization_msgs::MarkerArray markers;
    const std_msgs::ColorRGBA color_path = visualization_functions::Color::Yellow();
    const std_msgs::ColorRGBA color_pruned_path = visualization_functions::Color::Purple();
    const std::string ns_path = "Astar";
    const std::string ns_path_pruned = "Astar_pruned";
    visualization_functions::CreatePathMarker(path, color_path, inertial_frame_id, ns_path, &markers);
    visualization_functions::CreatePathMarker(pruned_path, color_pruned_path, inertial_frame_id,
                                              ns_path_pruned, &markers);
    path_planning_path_marker_pub_.publish(markers);
}

void MapperClass::LoadPathPlanningConfig(const std::string &inertial_frame_id,
                                         pensa_msgs::PathPlanningConfig *path_planning_config,
                                         ros::NodeHandle *nh) {
    // Capture service name and wait for it to start existing
    const std::string srv_return_path_planning_config = nh->resolveName("/srv_return_path_planning_config");
    while (ros::ok() && !ros::service::waitForService(srv_return_path_planning_config, ros::Duration(0.1))) {
        ROS_WARN_DELAYED_THROTTLE(5.0, "[mapper]: Waiting for path planning config service...");
    }
    ROS_INFO("[mapper]: Connected to path planning config service");

    // Start client
    load_path_planning_config_client_ =
        nh->serviceClient<pensa_msgs::PathPlanningConfigInFrame>(srv_return_path_planning_config);

    // Call service to capture the planner config
    pensa_msgs::PathPlanningConfigInFrame load_path_planning_config_msg;
    load_path_planning_config_msg.request.frame_id = inertial_frame_id;
    if (!load_path_planning_config_client_.call(load_path_planning_config_msg)) {
        ROS_ERROR("[mapper]: Could not call service to load path planning config!");
        return;
    }
    *path_planning_config = load_path_planning_config_msg.response.config;
}

// PLUGINLIB_EXPORT_CLASS(mapper::MapperClass, nodelet::Nodelet);

}  // namespace mapper
