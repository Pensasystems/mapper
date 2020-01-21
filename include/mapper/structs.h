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

#ifndef MAPPER_STRUCTS_H_
#define MAPPER_STRUCTS_H_

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// c++ libraries
#include <pthread.h>
#include <semaphore.h>
#include <queue>
#include <string>

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
    pthread_mutex_t sampled_traj;
    pthread_mutex_t traj_status;
    pthread_mutex_t body_tf;
    pthread_mutex_t cam_tf;
    pthread_mutex_t lidar_tf;
    pthread_mutex_t octomap;
    pthread_mutex_t point_cloud;
    pthread_mutex_t update_map;

    // Methods
    mutexStruct() {
        pthread_mutex_init(&sampled_traj, NULL);
        pthread_mutex_init(&traj_status, NULL);
        pthread_mutex_init(&body_tf, NULL);
        pthread_mutex_init(&cam_tf, NULL);
        pthread_mutex_init(&lidar_tf, NULL);
        pthread_mutex_init(&octomap, NULL);
        pthread_mutex_init(&point_cloud, NULL);
        pthread_mutex_init(&update_map, NULL);
    }
    void destroy() {
        pthread_mutex_destroy(&sampled_traj);
        pthread_mutex_destroy(&traj_status);
        pthread_mutex_destroy(&body_tf);
        pthread_mutex_destroy(&cam_tf);
        pthread_mutex_destroy(&lidar_tf);
        pthread_mutex_destroy(&octomap);
        pthread_mutex_destroy(&point_cloud);
        pthread_mutex_destroy(&update_map);
    }
};

class semaphoreStruct {
 public:
    sem_t pcl;
    sem_t collision_check;

    // Methods
    semaphoreStruct() {
        sem_init(&pcl, 0, 0);
        sem_init(&collision_check, 0, 0);
    }
    void destroy() {
        sem_destroy(&pcl);
        sem_destroy(&collision_check);
    }
};

}  // namespace mapper

#endif  // MAPPER_STRUCTS_H_
