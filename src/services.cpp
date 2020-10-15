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

#include "mapper/mapper_class.h"

#include <limits>
#include <string>
#include <vector>

namespace mapper {

// Update resolution of the map
bool MapperClass::UpdateResolution(pensa_msgs::SetFloat::Request &req,
                                   pensa_msgs::SetFloat::Response &res) {
    mutexes_.octomap.lock();
        globals_.octomap.SetResolution(req.data);
    mutexes_.octomap.unlock();

    res.success = true;
    return true;
}

// Update map memory time
bool MapperClass::UpdateMemoryTime(pensa_msgs::SetFloat::Request &req,
                                   pensa_msgs::SetFloat::Response &res) {
    mutexes_.octomap.lock();
        globals_.octomap.SetMemory(req.data);
    mutexes_.octomap.unlock();

    res.success = true;
    return true;
}

bool MapperClass::MapInflation(pensa_msgs::SetFloat::Request &req,
                               pensa_msgs::SetFloat::Response &res) {
    mutexes_.octomap.lock();
        globals_.octomap.SetMapInflation(req.data);
    mutexes_.octomap.unlock();

    res.success = true;
    return true;
}

bool MapperClass::ResetMap(pensa_msgs::SetFloat::Request &req,
                           pensa_msgs::SetFloat::Response &res) {
    mutexes_.octomap.lock();
        globals_.octomap.ResetMap();
        globals_.octomap.SetMemory(req.data);
    mutexes_.octomap.unlock();

    res.success = true;
    return true;
}

bool MapperClass::InitializeMapToPathPlanningConfig(std_srvs::Trigger::Request &req,
                                                    std_srvs::Trigger::Response &res) {
    mutexes_.octomap.lock();
        globals_.octomap.InitializeMapToPathPlanningConfig();
        globals_.octomap.SetMemory(-1.0);  // Infinite memory
    mutexes_.octomap.unlock();

    res.success = true;
    res.message = "Map has been reset to path planning config!";
    return true;
}

bool MapperClass::SaveMap(std_srvs::Trigger::Request &req,
                          std_srvs::Trigger::Response &res) {
    std::string filename1 = local_path_ + "/maps/octomap.ot";
    std::string filename2 = local_path_ + "/maps/octomap_inflated.ot";
    mutexes_.octomap.lock();
        globals_.octomap.tree_.write(filename1);
        globals_.octomap.tree_inflated_.write(filename2);
    mutexes_.octomap.unlock();

    ROS_INFO("Maps saved in:\n%s \n%s\n", filename1.c_str(), filename2.c_str());
    return true;
}

bool MapperClass::LoadMap(std_srvs::Trigger::Request &req,
                          std_srvs::Trigger::Response &res) {
    std::string filename1 = local_path_ + "/maps/octomap.ot";
    std::string filename2 = local_path_ + "/maps/octomap_inflated.ot";
    octomap::OcTree* tree = dynamic_cast<octomap::OcTree*>(octomap::OcTree::read(filename1));
    octomap::OcTree* tree_inflated = dynamic_cast<octomap::OcTree*>(octomap::OcTree::read(filename2));
    if (tree && tree_inflated) {
        mutexes_.octomap.lock();
            globals_.octomap.CopyMap(*tree, *tree_inflated);
        mutexes_.octomap.unlock();
    }
    return true;
}

bool MapperClass::OctomapProcessPCL(std_srvs::SetBool::Request &req,
                                    std_srvs::SetBool::Response &res) {
    mutexes_.update_map.lock();
        globals_.update_map = req.data;
    mutexes_.update_map.unlock();
    if (req.data) {
        ROS_INFO("[mapper]: PCL data will be processed!");
    } else {
        ROS_INFO("[mapper]: PCL data will not be processed!");
    }
    return true;
}

bool MapperClass::AStarService(pensa_msgs::Astar::Request &req,
                               pensa_msgs::Astar::Response &res) {
    res.success = true;
    return true;
}

bool MapperClass::RRGService(pensa_msgs::RRT_RRG_PRM::Request &req,
                             pensa_msgs::RRT_RRG_PRM::Response &res) {
    std::vector<Eigen::Vector3d> e_path;
    visualization_msgs::Marker graph_markers;
    mutexes_.octomap.lock();
    res.success = globals_.octomap.OctoRRG(
        msg_conversions::ros_point_to_eigen_vector(req.origin),
        msg_conversions::ros_point_to_eigen_vector(req.destination),
        msg_conversions::ros_point_to_eigen_vector(req.box_min),
        msg_conversions::ros_point_to_eigen_vector(req.box_max),
        req.max_time, req.max_nodes, req.steer_param, req.free_space_only,
        req.prune_result, req.publish_rviz, &res.planning_time, &res.n_nodes,
        &e_path, &graph_markers);
    mutexes_.octomap.unlock();
    for (uint i = 0 ; i < e_path.size(); i++) {
        res.path.push_back(msg_conversions::eigen_to_ros_point(e_path[i]));
    }

    if (req.publish_rviz) {
        graph_tree_marker_pub_.publish(graph_markers);
    }
    return true;
}

}  // namespace mapper
