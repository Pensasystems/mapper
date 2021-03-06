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

#include <string>
#include <vector>
#include "mapper/mapper_class.h"
#include "mapper/pcl_conversions.h"

namespace mapper {

void MapperClass::LidarSyncCallback(const sensor_msgs::PointCloud2::ConstPtr &lidar_msg,
                                    const geometry_msgs::PoseStamped::ConstPtr &base_link_pose_msg) {
    // Get lidar pose in inertial frame
    const tf2::Transform tf_base_link_in_inertial_frame =
            msg_conversions::ros_pose_to_tf2_transform(base_link_pose_msg->pose);
    const tf2::Transform tf_lidar_in_inertial_frame = tf_base_link_in_inertial_frame * tf_lidar_in_base_link_frame_;

    // Structure to include pcl and its frame
    StampedPcl new_pcl;
    const uint max_queue_size = globals_.max_queue_size;

    // Populate StampedPcl
    pcl_conversions::FromROSMsg(*lidar_msg, &new_pcl.cloud);
    new_pcl.tf_pcl2world = tf_lidar_in_inertial_frame;
    new_pcl.is_lidar = true;

    // save into global variables
    mutexes_.point_cloud.lock();
        globals_.pcl_queue.push(new_pcl);
        if (globals_.pcl_queue.size() > max_queue_size) {
            globals_.pcl_queue.pop();
        } else {
            // signal octomap thread to process new pcl data
            sem_post(&semaphores_.pcl);
        }
    mutexes_.point_cloud.unlock();
}

void MapperClass::BaseLinkPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
    // Save drone's base_link into global variable for obstacle detection
    const tf2::Transform tf_base_link_in_inertial_frame = msg_conversions::ros_pose_to_tf2_transform(msg->pose);
    mutexes_.body_tf.lock();
        globals_.tf_body2world = tf_base_link_in_inertial_frame;
    mutexes_.body_tf.unlock();
}


// void MapperClass::SegmentCallback(const mapper::Segment::ConstPtr &msg) {
//     ros::Time t0 = ros::Time::now();
//     while (t0.toSec() == 0) {
//         t0 = ros::Time::now();
//     }

//     // set segments
//     mapper::Segment segments = *msg;

//     if (segments.segment.size() == 0) {  // Empty trajectory
//         return;
//     }

//     // transform message into set of polynomials
//     polynomials::Trajectory3D poly_trajectories(segments);

//     // sample trajectory at 10hz
//     double ts = 0.1;
//     sampled_traj::SampledTrajectory3D sampled_traj(ts, poly_trajectories);
//     // SampledTrajectory3D sampledTraj(Time, Positions); // fake trajectory

//     mutexes_.sampled_traj.lock();
//         globals_.sampled_traj.pos_ = sampled_traj.pos_;
//         globals_.sampled_traj.time_ = sampled_traj.time_;
//         globals_.sampled_traj.n_points_ = sampled_traj.n_points_;

//         // compress trajectory into points with max deviation of 1cm from original trajectory
//         globals_.sampled_traj.CompressSamples();

//         //  Transform compressed trajectory into a set of pixels in octomap
//         //  Octomap insertion avoids repeated points
//         globals_.sampled_traj.thick_traj_.clear();
//         for (int i = 0; i < globals_.sampled_traj.n_compressed_points_-1; i++) {
//             globals_.sampled_traj.ThickBresenham(globals_.sampled_traj.compressed_pos_[i],
//                                                 globals_.sampled_traj.compressed_pos_[i+1]);
//         }

//         // populate trajectory node centers in a point cloud
//         globals_.sampled_traj.ThickTrajToPcl();

//         // populate kdtree for finding nearest neighbor w.r.t. collisions
//         globals_.sampled_traj.CreateKdTree();
//     mutexes_.sampled_traj.unlock();

//     // Notify the collision checker to check for collision
//     // sem_post(&semaphores_.collision_check);

//     // ros::Duration solver_time = ros::Time::now() - t0;
//     // ROS_DEBUG("Time to compute octotraj: %f", solver_time.toSec());
// }

void MapperClass::SampledTrajectoryCallback(const pensa_msgs::VecPVA_4d::ConstPtr &msg) {
    ROS_INFO("New trajectory!");
    ros::Time t0 = ros::Time::now();
    while (t0.toSec() == 0) {
        t0 = ros::Time::now();
    }

    if (msg->pva_vec.size() == 0) {  // Empty trajectory
        return;
    }

    // Transform VecPVA_4d into SampledTrajectory3D
    sampled_traj::SampledTrajectory3D sampled_traj(*msg, globals_.map_3d);
    ROS_INFO("Number of samples in trajectory: %zu", sampled_traj.pos_.size());

    mutexes_.sampled_traj.lock();
        globals_.sampled_traj.pos_ = sampled_traj.pos_;
        globals_.sampled_traj.time_ = sampled_traj.time_;
        globals_.sampled_traj.n_points_ = sampled_traj.n_points_;

        // compress trajectory into points with max deviation of 1cm from original trajectory
        globals_.sampled_traj.CompressSamples();
        ROS_INFO("Number of compressed samples in trajectory: %d", globals_.sampled_traj.n_compressed_points_);

        //  Transform compressed trajectory into a set of pixels in octomap
        //  Octomap insertion avoids repeated points
        globals_.sampled_traj.thick_traj_.clear();
        for (int i = 0; i < globals_.sampled_traj.n_compressed_points_-1; i++) {
            globals_.sampled_traj.ThickBresenham(globals_.sampled_traj.compressed_pos_[i],
                                                globals_.sampled_traj.compressed_pos_[i+1]);
        }

        // populate trajectory node centers in a point cloud
        globals_.sampled_traj.ThickTrajToPcl();

        // populate kdtree for finding nearest neighbor w.r.t. collisions
        globals_.sampled_traj.CreateKdTree();
    mutexes_.sampled_traj.unlock();

    // Notify the collision checker to check for collision
    // sem_post(&semaphores_.collision_check);
}

void MapperClass::WaypointsCallback(const pensa_msgs::WaypointSetConstPtr &msg) {
    ROS_INFO("New waypoints!");
    ros::Time t0 = ros::Time::now();
    while (t0.toSec() == 0) {
        t0 = ros::Time::now();
    }

    if (msg->waypoints.size() == 0) {  // Empty trajectory
        return;
    }

    // Transform WaypointSet into SampledTrajectory3D
    sampled_traj::SampledTrajectory3D sampled_traj(msg->waypoints, globals_.map_3d);
    ROS_INFO("Number of waypoints: %zu", sampled_traj.pos_.size());

    mutexes_.sampled_traj.lock();
        globals_.sampled_traj.pos_ = sampled_traj.pos_;
        globals_.sampled_traj.time_ = sampled_traj.time_;
        globals_.sampled_traj.compressed_pos_ = sampled_traj.compressed_pos_;
        globals_.sampled_traj.compressed_time_ = sampled_traj.compressed_time_;
        globals_.sampled_traj.n_points_ = sampled_traj.n_points_;
        globals_.sampled_traj.n_compressed_points_ = sampled_traj.n_points_;

        //  Transform compressed trajectory into a set of pixels in octomap
        //  Octomap insertion avoids repeated points
        globals_.sampled_traj.thick_traj_.clear();
        for (int i = 0; i < globals_.sampled_traj.n_compressed_points_-1; i++) {
            globals_.sampled_traj.ThickBresenham(globals_.sampled_traj.compressed_pos_[i],
                                                 globals_.sampled_traj.compressed_pos_[i+1]);
        }

        // populate trajectory node centers in a point cloud
        globals_.sampled_traj.ThickTrajToPcl();

        // populate kdtree for finding nearest neighbor w.r.t. collisions
        globals_.sampled_traj.CreateKdTree();
    mutexes_.sampled_traj.unlock();

    // Notify the collision checker to check for collision
    // sem_post(&semaphores_.collision_check);
}

void MapperClass::TrajectoryStatusCallback(const pensa_msgs::trapezoidal_p2pActionFeedbackConstPtr &msg) {
    mutexes_.traj_status.lock();
        globals_.traj_status = msg->feedback;
    mutexes_.traj_status.unlock();
}

void MapperClass::DestroyAllCallbacks() {
    trajectory_status_sub_.shutdown();
    waypoints_sub_.shutdown();
    base_link_pose_sub_.shutdown();
    lidar_sync_sub_->unsubscribe();
    base_link_pose_sync_sub_->unsubscribe();
    ROS_DEBUG("[mapper]: All subscribers have been destroyed!");
}

}  // namespace mapper
