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

#include <mapper/mapper_class.h>
#include <string>
#include <vector>
#include <algorithm>

namespace mapper {

// Thread for fading memory of the octomap
void MapperClass::FadeTask() {
    ROS_INFO("Fading Memory Thread started with rate %f: ", fading_memory_update_rate_);

    // Rate at which this thread will run
    ros::Rate loop_rate(fading_memory_update_rate_);

    while (ros::ok()) {
        // Get time for when this task started
        const ros::Time t0 = ros::Time::now();

        pthread_mutex_lock(&mutexes_.octomap);
            if (globals_.octomap.memory_time_ > 0) {
                globals_.octomap.FadeMemory(fading_memory_update_rate_);
            }
        pthread_mutex_unlock(&mutexes_.octomap);

        // ros::Duration fade_time = ros::Time::now() - t0;
        // ROS_DEBUG("Fading memory execution time: %f", fade_time.toSec());

        loop_rate.sleep();
    }
    ROS_DEBUG("Exiting Fading Memory Thread...");
}

// Thread for updating the body tfTree values
void MapperClass::BodyTfTask(const std::string& parent_frame,
                             const std::string& child_frame) {
    ROS_DEBUG("robot frame tf Thread started with rate %f: ", tf_update_rate_);
    tf_listener::TfClass obj_body2world;
    ros::Rate loop_rate(tf_update_rate_);

    while (ros::ok()) {
        // Get the transform
        obj_body2world.GetTransform(child_frame, parent_frame);
        // obj_body2world.PrintOrigin();

        pthread_mutex_lock(&mutexes_.body_tf);
            globals_.tf_body2world = obj_body2world.transform_;
        pthread_mutex_unlock(&mutexes_.body_tf);
        loop_rate.sleep();
    }

    ROS_DEBUG("Exiting body tf Thread...");
}

// Thread for updating the tfTree values
void MapperClass::CameraTfTask(const std::string& parent_frame,
                               const std::string& child_frame,
                               const uint& index) {
    ROS_DEBUG("tf Thread from frame `%s` to `%s` started with rate %f: ", 
              child_frame.c_str(), parent_frame.c_str(), tf_update_rate_);
    tf_listener::TfClass obj_tf;
    ros::Rate loop_rate(tf_update_rate_);

    while (ros::ok()) {
        // Get the transform
        obj_tf.GetTransform(child_frame, parent_frame);
        // obj_tf.PrintOrigin();

        pthread_mutex_lock(&mutexes_.cam_tf);
            globals_.tf_cameras2world[index] = obj_tf.transform_;
        pthread_mutex_unlock(&mutexes_.cam_tf);
        loop_rate.sleep();
    }

    ROS_DEBUG("Exiting body tf Thread...");
}

void MapperClass::LidarTfTask(const std::string& parent_frame,
                              const std::string& child_frame,
                              const uint& index) {
    ROS_DEBUG("tf Thread from frame `%s` to `%s` started with rate %f: ", 
              child_frame.c_str(), parent_frame.c_str(), tf_update_rate_);
    tf_listener::TfClass obj_tf;
    ros::Rate loop_rate(tf_update_rate_);

    while (ros::ok()) {
        // Get the transform
        obj_tf.GetTransform(child_frame, parent_frame);
        // obj_tf.PrintOrigin();

        pthread_mutex_lock(&mutexes_.lidar_tf);
            globals_.tf_lidar2world[index] = obj_tf.transform_;
        pthread_mutex_unlock(&mutexes_.lidar_tf);
        loop_rate.sleep();
    }

    ROS_DEBUG("Exiting body tf Thread...");
}

void MapperClass::CollisionCheckTask() {
    ROS_DEBUG("collisionCheck Thread started!");

    // Rate at which the collision checker will run
    // ros::Rate loop_rate(collision_check_rate_);

    // visualization markers
    visualization_msgs::MarkerArray traj_markers, samples_markers;
    visualization_msgs::MarkerArray compressed_samples_markers, collision_markers;

    // robot's transform variable
    tf::StampedTransform tf_body2world;

    // pcl variables
    int cloudsize;

    while (ros::ok()) {
        // Wait until there is a new trajectory or a new update on the map
        sem_wait(&semaphores_.collision_check);

        // Get time for when this task started
        ros::Time time_now = ros::Time::now();

        // Copy trajectory into local point cloud
        pcl::PointCloud< pcl::PointXYZ > point_cloud_traj;
        std::vector<octomap::point3d> colliding_nodes;
        traj_markers.markers.clear();
        collision_markers.markers.clear();
        samples_markers.markers.clear();
        compressed_samples_markers.markers.clear();
        pthread_mutex_lock(&mutexes_.sampled_traj);
            point_cloud_traj = globals_.sampled_traj.point_cloud_traj_;
            std::vector<double> time = globals_.sampled_traj.time_;
            // pcl::PointCloud<pcl::PointXYZ> pos = globals_.sampled_traj.pos_;

            // Send visualization markers
            globals_.sampled_traj.TrajVisMarkers(&traj_markers);
            globals_.sampled_traj.SamplesVisMarkers(&samples_markers);
            globals_.sampled_traj.CompressedVisMarkers(&compressed_samples_markers);
            path_marker_pub_.publish(traj_markers);
            path_marker_pub_.publish(samples_markers);
            path_marker_pub_.publish(compressed_samples_markers);
        pthread_mutex_unlock(&mutexes_.sampled_traj);

        // Get robot's current position
        pthread_mutex_lock(&mutexes_.body_tf);
            tf_body2world = globals_.tf_body2world;
        pthread_mutex_unlock(&mutexes_.body_tf);
        Eigen::Vector3d robot_position = 
            msg_conversions::tf_vector3_to_eigen_vector(tf_body2world.getOrigin());

        // Find point in compressed trajectory that the robot is closest to
        geometry_msgs::Point robot_projected_on_traj;
        pthread_mutex_lock(&mutexes_.sampled_traj);
        bool success = globals_.sampled_traj.NearestPointInCompressedTraj(robot_position, &robot_projected_on_traj);
        pthread_mutex_unlock(&mutexes_.sampled_traj);
        if(!success) {  // this fails if there are not at least two points in the compressed trajectory
            continue;
        }

        // Get trajectory status
        pensa_msgs::trapezoidal_p2pFeedback traj_status;
        pthread_mutex_lock(&mutexes_.traj_status);
            traj_status = globals_.traj_status;
        pthread_mutex_unlock(&mutexes_.traj_status);

        // Set current trajectory to z=0 if working with 2D maps only
        if(!globals_.map_3d) {
            traj_status.current_position.z = 0.0;
        }

        // Stop execution if there are no points in the trajectory structure
        cloudsize = point_cloud_traj.size();
        if (cloudsize <= 0) {
            visualization_functions::DrawCollidingNodes(colliding_nodes, inertial_frame_id_, 0.015, &collision_markers);
            path_marker_pub_.publish(traj_markers);
            path_marker_pub_.publish(collision_markers);
            continue;
        }

        // Stop execution if the current time is beyond the final time of the trajectory
        // if (traj_status.current_time >= traj_status.final_time) {
        //     pthread_mutex_lock(&mutexes_.sampled_traj);
        //         globals_.sampled_traj.ClearObject();
        //         globals_.sampled_traj.TrajVisMarkers(&traj_markers);
        //     pthread_mutex_unlock(&mutexes_.sampled_traj);
        //     visualization_functions::DrawCollidingNodes(colliding_nodes, inertial_frame_id_, 0.015, &collision_markers);
        //     path_marker_pub_.publish(traj_markers);
        //     path_marker_pub_.publish(collision_markers);
        //     continue;
        // }

        // Get visualization marker for current set point and robot position projected on the trajectory
        globals_.sampled_traj.ReferenceVisMarker(traj_status.current_position, &traj_markers);
        globals_.sampled_traj.RobotPosVisMarker(robot_projected_on_traj, &traj_markers);

        // Check if trajectory collides with points in the point-cloud
        pthread_mutex_lock(&mutexes_.octomap);
            double res = globals_.octomap.tree_inflated_.getResolution();
            globals_.octomap.FindCollidingNodesInflated(point_cloud_traj, &colliding_nodes);
        pthread_mutex_unlock(&mutexes_.octomap);
        // ROS_INFO("Colliding nodes: %d", int(colliding_nodes.size()) );

        if (colliding_nodes.size() > 0) {
            // Sort collision time (use kdtree for nearest neighbor)
            std::vector<geometry_msgs::PointStamped> sorted_collisions;
            pthread_mutex_lock(&mutexes_.sampled_traj);
                globals_.sampled_traj.SortCollisionsByDistance(colliding_nodes, robot_projected_on_traj, &sorted_collisions);
            pthread_mutex_unlock(&mutexes_.sampled_traj);

            // double collision_time = (sorted_collisions[0].header.stamp - ros::Time::now()).toSec();
            Eigen::Vector3d p0 = msg_conversions::ros_point_to_eigen_vector(robot_projected_on_traj);
            Eigen::Vector3d p1 = msg_conversions::ros_point_to_eigen_vector(sorted_collisions[0].point);
            double collision_distance = (p1-p0).norm();
            // uint lastCollisionIdx = sorted_collisions.back().header.seq;
            // if (collision_time > 0) {
                // ROS_WARN("Imminent collision within %.3f seconds!", collision_time);
                ROS_WARN("Imminent collision within %.3f meters!", collision_distance);
                sentinel_pub_.publish(sorted_collisions[0]);
                // pthread_mutex_lock(&mutexes_.sampled_traj);
                //     globals_.sampled_traj.ClearObject();
                // pthread_mutex_unlock(&mutexes_.sampled_traj);
            // }
        }

        // Draw colliding markers (delete if none)
        visualization_functions::DrawCollidingNodes(colliding_nodes, "world", 1.01*res, &collision_markers);
        path_marker_pub_.publish(traj_markers);
        path_marker_pub_.publish(collision_markers);

        // ros::Duration solver_time = ros::Time::now() - time_now;
        // ROS_INFO("Collision check time: %f", solver_time.toSec());

        // loop_rate.sleep();
    }

    ROS_DEBUG("Exiting collisionCheck Thread...");
}

void MapperClass::OctomappingTask() {
    ROS_DEBUG("OctomappingTask Thread started!");
    tf::StampedTransform tf_cam2world;
    pcl::PointCloud< pcl::PointXYZ > pcl_world;

    while (ros::ok()) {
        // Wait until there is new pcl data
        sem_wait(&semaphores_.pcl);

        // Get time for when this task started
        const ros::Time t0 = ros::Time::now();

        // Get Point Cloud
        pthread_mutex_lock(&mutexes_.point_cloud);
            // Get data from queue
            pcl::PointCloud< pcl::PointXYZ > point_cloud = globals_.pcl_queue.front().cloud;
            const tf::StampedTransform tf_cam2world = globals_.pcl_queue.front().tf_cam2world;
            const bool is_lidar = globals_.pcl_queue.front().is_lidar;

            // Remove data from queue
            globals_.pcl_queue.pop();
        pthread_mutex_unlock(&mutexes_.point_cloud);

        // Check if a tf message has been received already. If not, return
        if (tf_cam2world.stamp_.toSec() == 0) {
            continue;
        }

        // Get camera transform
        tf::Quaternion q = tf_cam2world.getRotation();
        tf::Vector3 v = tf_cam2world.getOrigin();
        Eigen::Affine3d transform = Eigen::Affine3d::Identity();
        transform.translation() << v.getX(), v.getY(), v.getZ();
        transform.rotate(Eigen::Quaterniond(q.getW(), q.getX(), q.getY(), q.getZ()));

        // Update frustum orientation
        algebra_3d::FrustumPlanes world_frustum;
        pthread_mutex_lock(&mutexes_.octomap);
            globals_.octomap.cam_frustum_.TransformFrustum(transform, &world_frustum);
        pthread_mutex_unlock(&mutexes_.octomap);

        // Should we process PCL data?
        pthread_mutex_lock(&mutexes_.update_map);
            const bool update_map = globals_.update_map;
        pthread_mutex_unlock(&mutexes_.update_map);

        // Process PCL data
        if (update_map) {
            // Transform pcl into world frame
            pcl::transformPointCloud(point_cloud, pcl_world, transform);

            // Save into octomap
            pthread_mutex_lock(&mutexes_.octomap);
                if (is_lidar) {
                    globals_.octomap.PclToRayOctomap(pcl_world, tf_cam2world);
                } else {
                    globals_.octomap.PclToRayOctomap(pcl_world, tf_cam2world, world_frustum);
                }
                globals_.octomap.tree_.prune();   // prune the tree before visualizing
            pthread_mutex_unlock(&mutexes_.octomap);
        }


        // Publish visualization markers iff at least one node is subscribed to it
        bool pub_obstacles, pub_free, pub_obstacles_inflated, pub_free_inflated;
        pub_obstacles = (obstacle_marker_pub_.getNumSubscribers() > 0);
        pub_free = (free_space_marker_pub_.getNumSubscribers() > 0);
        pub_obstacles_inflated = (inflated_obstacle_marker_pub_.getNumSubscribers() > 0);
        pub_free_inflated = (inflated_free_space_marker_pub_.getNumSubscribers() > 0);

        if (pub_obstacles || pub_free) {
            visualization_msgs::MarkerArray obstacle_markers;
            visualization_msgs::MarkerArray free_markers;
            pthread_mutex_lock(&mutexes_.octomap);
                globals_.octomap.TreeVisMarkers(&obstacle_markers, &free_markers);
            pthread_mutex_unlock(&mutexes_.octomap);
            if (pub_obstacles) {
                obstacle_marker_pub_.publish(obstacle_markers);
            }
            if (pub_free) {
                free_space_marker_pub_.publish(free_markers);
            }
        }

        if (pub_obstacles_inflated || pub_free_inflated) {
            visualization_msgs::MarkerArray inflated_markers;
            visualization_msgs::MarkerArray inflated_free_markers;
            pthread_mutex_lock(&mutexes_.octomap);
                globals_.octomap.InflatedVisMarkers(&inflated_markers, &inflated_free_markers);
            pthread_mutex_unlock(&mutexes_.octomap);
            if (pub_obstacles_inflated) {
                inflated_obstacle_marker_pub_.publish(inflated_markers);
            }
            if (pub_free_inflated) {
                inflated_free_space_marker_pub_.publish(inflated_free_markers);
            }
        }

        if ((!is_lidar) && (cam_frustum_pub_.getNumSubscribers() > 0)) {
            visualization_msgs::Marker frustum_markers;
            pthread_mutex_lock(&mutexes_.octomap);
                globals_.octomap.cam_frustum_.VisualizeFrustum(point_cloud.header.frame_id, &frustum_markers);
            pthread_mutex_unlock(&mutexes_.octomap);
            cam_frustum_pub_.publish(frustum_markers);
        } else if ((is_lidar) && (cam_frustum_pub_.getNumSubscribers() > 0)) {
            visualization_msgs::Marker lidar_range_marker;
            Eigen::Vector3d lidar_origin = transform.translation();
            if(!globals_.map_3d) {
                lidar_origin[2] = 0.0;
            }
            pthread_mutex_lock(&mutexes_.octomap);
                globals_.octomap.lidar_range_.VisualizeRange(
                    globals_.octomap.GetInertialFrameId(), lidar_origin,
                    &lidar_range_marker);
            pthread_mutex_unlock(&mutexes_.octomap);
            cam_frustum_pub_.publish(lidar_range_marker);
        }

        // Notify the collision checker to check for collision due to map update
        sem_post(&semaphores_.collision_check);

        // ros::Duration map_time = ros::Time::now() - t0;
        // ROS_INFO("Mapping time: %f", map_time.toSec());
    }

    ROS_DEBUG("Exiting OctomappingTask Thread...");
}

void MapperClass::KeyboardTask() {
    ROS_DEBUG("KeyboardTask Thread started!");

    std::string input_string;
    while (ros::ok()) {
        std::getline(std::cin, input_string);
        ROS_INFO("Received command: %s", input_string.c_str());

        std::string filename1 = local_path_ + "/maps/octomap.ot";
        std::string filename2 = local_path_ + "/maps/octomap_inflated.ot";
        if (input_string == "save_map") {
            pthread_mutex_lock(&mutexes_.octomap);
                globals_.octomap.tree_.write(filename1);
                globals_.octomap.tree_inflated_.write(filename2);
            pthread_mutex_unlock(&mutexes_.octomap);
            
            ROS_INFO("Maps saved in:\n%s \n%s\n", filename1.c_str(), filename2.c_str());
        } else if (input_string == "load_map"){
            octomap::OcTree* tree = dynamic_cast<octomap::OcTree*>(octomap::OcTree::read(filename1));
            octomap::OcTree* tree_inflated = dynamic_cast<octomap::OcTree*>(octomap::OcTree::read(filename2));
            if (tree && tree_inflated) {
                pthread_mutex_lock(&mutexes_.octomap);
                    globals_.octomap.CopyMap(*tree, *tree_inflated);
                pthread_mutex_unlock(&mutexes_.octomap);
            }
        }
    }

    ROS_DEBUG("Exiting KeyboardTask Thread...");
}

}  // namespace mapper
