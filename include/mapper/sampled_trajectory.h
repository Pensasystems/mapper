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

#include <ros/ros.h>

#include <Eigen/Dense>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <visualization_msgs/MarkerArray.h>

// Pensa-ros msg types
#include "pensa_msgs/VecPVA_4d.h"

// Classes
#include "mapper/linear_algebra.h"
#include "mapper/polynomials.h"
#include "mapper/visualization_functions.h"

// C++ specific libraries
#include <iostream>
#include <string>
#include <vector>

namespace sampled_traj {

//  Pos has the discretized points in the trajectory,
// and is compressed with the function compressSamples
//  Time has the corresponding times within Pos
//  nPoints has the number of points in the Pos vector
//  ThickTraj is of the octree type to avoid repeated nodes
// that occurs when concatenating trajectories between two
// waypoints
class SampledTrajectory3D{
 public:
    // Sampled trajectory variables
    pcl::PointCloud<pcl::PointXYZ> pos_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr_ =
            pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<double> time_;
    int n_points_;

    // Compressed samples (std::vector can delete entries, pcl::PointCloud isn't that easy)
    std::vector<Eigen::Vector3d> compressed_pos_;
    std::vector<double> compressed_time_;
    int n_compressed_points_;   // Number of points after compression
    double max_dev_;          // Max deviation for compression

    // Thick trajectory variables
    // The reason to use octomap here is to avoid adding
    // the same node multiple times as we thicken the
    // trajectory
    octomap::OcTree thick_traj_ = octomap::OcTree(0.1);  // Create empty tree with resolution 0.1
    pcl::PointCloud<pcl::PointXYZ> point_cloud_traj_;
    double resolution_;
    double thickness_;

    // Kdtree to find nearest obstacles
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_point_cloud_traj_;

    // World frame id
    std::string inertial_frame_id_;


    // Constructor
    // SampledTrajectory3D(const double &dt,
    //                     const polynomials::Trajectory3D &poly_trajectories);
    SampledTrajectory3D(const std::vector<double> &time_vec,
                        const pcl::PointCloud<pcl::PointXYZ> &pos_vec);
    SampledTrajectory3D(const pensa_msgs::VecPVA_4d &pva_vec,
                        const bool &map_3d);
    SampledTrajectory3D(const std::vector<geometry_msgs::Point> &waypoints,
                        const bool &map_3d);
    SampledTrajectory3D();

    // Methods
    void PrintSamples();
    void SetMaxDev(const double &max_dev);
    void SetResolution(const double &resolution);
    double GetResolution();
    void SetInertialFrame(const std::string &inertial_frame_id);
    void DeleteSample(const int &index);
    void CompressSamples();
    bool NearestPointInCompressedTraj(const Eigen::Vector3d &point,
                                      geometry_msgs::Point *nearest_point);
    bool NearestPointInCompressedTraj(const geometry_msgs::Point &point,
                                      geometry_msgs::Point *nearest_point);
    void Bresenham(const Eigen::Vector3d &p0,
                   const Eigen::Vector3d &pf,
                   std::vector<octomap::point3d> *points);  // Bresenham line algorithm por printing a line
    void ThickBresenham(const Eigen::Vector3d &p0,
                        const Eigen::Vector3d &pf);          // Thick bresenham line algorithm por printing a line
    void ThickTrajToPcl();
    void CreateKdTree();
    // void SortCollisionsByTime(const std::vector<octomap::point3d> &colliding_nodes,
    //                           std::vector<geometry_msgs::PointStamped> *samples);
    void SortCollisionsByDistance(const std::vector<octomap::point3d> &colliding_nodes,
                                  const geometry_msgs::Point &origin,
                                  std::vector<geometry_msgs::PointStamped> *samples);
    void TrajVisMarkers(visualization_msgs::MarkerArray* marker_array);
    void SamplesVisMarkers(visualization_msgs::MarkerArray* marker_array);
    void CompressedVisMarkers(visualization_msgs::MarkerArray* marker_array);
    void GetVisMarkers(visualization_msgs::MarkerArray *traj_markers,
                       visualization_msgs::MarkerArray *samples_markers,
                       visualization_msgs::MarkerArray *compressed_samples_markers);
    void ClearObject();  // Clear all the data within this object
};

// Comparison funcion used in sort algorithm
// bool ComparePointStamped(const geometry_msgs::PointStamped &sample1,
//                          const geometry_msgs::PointStamped &sample2);

// Comparison function used in sort algorithm
bool ComparePointDistance(const geometry_msgs::PointStamped &sample1,
                          const geometry_msgs::PointStamped &sample2,
                          const geometry_msgs::Point &origin);

}  // namespace sampled_traj
