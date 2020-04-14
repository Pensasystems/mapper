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

// PCL specific includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>

// c++ libraries
#include <mutex>
#include <queue>
#include <semaphore.h>
#include <string>
#include <vector>

// Locally defined libraries
#include "mapper/octoclass.h"
#include "mapper/sampled_trajectory.h"

// Pensa-ros msg types
#include <pensa_msgs/trapezoidal_p2pAction.h>

namespace mapper {

struct stampedPcl {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    tf::StampedTransform tf_cam2world;
    bool is_lidar;
};

struct globalVariables {
    // Mutex protected variables
    tf::StampedTransform tf_body2world;
    std::vector<tf::StampedTransform> tf_cameras2world;
    std::vector<tf::StampedTransform> tf_lidar2world;
    octoclass::OctoClass octomap = octoclass::OctoClass(0.05, "map", true);
    sampled_traj::SampledTrajectory3D sampled_traj;
    pensa_msgs::trapezoidal_p2pFeedback traj_status;
    std::queue<stampedPcl> pcl_queue;
    bool update_map;
    bool map_3d;
    const uint max_queue_size = 2;

    globalVariables() {
        traj_status.current_time = 0.0;
        traj_status.final_time = 0.0;
    }
};

class mutexStruct {
 public:
    std::mutex sampled_traj;
    std::mutex traj_status;
    std::mutex body_tf;
    std::mutex cam_tf;
    std::mutex lidar_tf;
    std::mutex octomap;
    std::mutex point_cloud;
    std::mutex update_map;
};

class semaphoreStruct {
 public:
    sem_t pcl;
    // sem_t collision_check;

    // Methods
    semaphoreStruct() {
        sem_init(&pcl, 0, 0);
        // sem_init(&collision_check, 0, 0);
    }
    void destroy() {
        sem_destroy(&pcl);
        // sem_destroy(&collision_check);
    }
};

}  // namespace mapper
