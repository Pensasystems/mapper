// Copyright (c) 2020 by Pensa Systems, Inc. -- All rights reserved
// Confidential and Proprietary

#include "mapper/octoclass.h"

#include <algorithm>
#include <limits>
#include <math.h>
#include <vector>

namespace octoclass {

// Delete colinear waypoints from path to minimize final number of waypoints
void OctoClass::DeleteColinearWaypoints(const std::vector<Eigen::Vector3d> &path,
                                        std::vector<Eigen::Vector3d> *compressed_path) {
  // initialize compressed points as all samples
  *compressed_path = path;
  int compressed_points = static_cast<int>(path.size());

  static double epsilon = 0.0001, dist;
  int delete_index;
  int current_node_index = 1;
  Eigen::Vector3d previous_point, current_point, next_point;
  while (true) {
    delete_index = -1;
    for (int i = current_node_index; i < compressed_points-1; i++) {
      previous_point << (*compressed_path)[i-1][0],
                        (*compressed_path)[i-1][1],
                        (*compressed_path)[i-1][2];
      current_point  << (*compressed_path)[i][0],
                        (*compressed_path)[i][1],
                        (*compressed_path)[i][2];
      next_point     << (*compressed_path)[i+1][0],
                        (*compressed_path)[i+1][1],
                        (*compressed_path)[i+1][2];
      algebra_3d::Line3d line(previous_point, next_point);
      line.DistancePoint2Line(current_point, &dist);
      if (dist < epsilon) {
        delete_index = i;
        break;
      }
    }
    if (delete_index > 0) {
      compressed_path->erase(compressed_path->begin() + delete_index);
      compressed_points = compressed_points - 1;
      current_node_index = delete_index;
    } else {
      break;
    }
  }
  // ROS_WARN("Deleted %zd colinear waypoints!", path.size() - compressed_path->size());
}

// Prune a path to minimize waypoints
// TODO(marcelino): make this algorithm O(n) and delete every waypoint that doesn't collide
// or reduce average wall distance. Perhaps we don't need to use an O(n^2) algorithm
void OctoClass::PathPruning(const std::vector<Eigen::Vector3d> &path,
                            const bool &free_space_only,
                            std::vector<Eigen::Vector3d> *compressed_path) {
  // first delete colinear points
  this->DeleteColinearWaypoints(path, compressed_path);
  int compressed_points = static_cast<int>(compressed_path->size());

  // Compress the remaining points
  static uint min_points = 2;
  static bool add_final_waypoint = true;
  static bool do_not_add_final_waypoint = false;
  double dist, max_dist;
  int delete_index, col_check;
  Eigen::Vector3d previous_point, current_point, next_point;
  while (compressed_points > min_points) {
    // first find the point that deviates the least in the whole set
    max_dist = std::numeric_limits<float>::infinity();
    delete_index = -1;
    for (int i = 1; i < compressed_points-1; i++) {
      previous_point << (*compressed_path)[i-1][0],
                        (*compressed_path)[i-1][1],
                        (*compressed_path)[i-1][2];
      current_point  << (*compressed_path)[i][0],
                        (*compressed_path)[i][1],
                        (*compressed_path)[i][2];
      next_point     << (*compressed_path)[i+1][0],
                        (*compressed_path)[i+1][1],
                        (*compressed_path)[i+1][2];

      // Check for collision
      if (free_space_only) {
        col_check = CheckCollision(previous_point, next_point);
      } else {
        col_check = CheckOccupancy(previous_point, next_point);
      }

      if (col_check == 1) {
        continue;
      }

      // If doesn't collide, check if new nodes get closer to walls
      // We don't allow to remove waypoints that decrease the distance to walls
      std::vector<octomap::point3d> prev_to_next_nodes, prev_to_current_to_next_nodes;
      this->GetNodesBetweenWaypoints(previous_point, next_point, add_final_waypoint, &prev_to_next_nodes);
      this->GetNodesBetweenWaypoints(previous_point, current_point, do_not_add_final_waypoint,
                                     &prev_to_current_to_next_nodes);
      this->GetNodesBetweenWaypoints(current_point, next_point, add_final_waypoint, &prev_to_current_to_next_nodes);
      if (this->AverageObstaclePathCost(prev_to_next_nodes) >
          this->AverageObstaclePathCost(prev_to_current_to_next_nodes)) {
        continue;
      }

      // Check how much the current_point deviates from the line from previous_point to next_point
      algebra_3d::Line3d line(previous_point, next_point);
      line.DistancePoint2Line(current_point, &dist);
      if (dist < max_dist) {
        max_dist = dist;
        delete_index = i;
      }
    }
    if (delete_index > 0) {
      compressed_path->erase(compressed_path->begin() + delete_index);
      compressed_points = compressed_points - 1;
    } else {
      break;
    }
  }

  // ROS_INFO("Compressed points: %d", compressed_points);
}

// Function that computes an added cost to traverse a node based on its
// distance to the nearest obstacle
double OctoClass::NearestObstaclePathCost(const octomap::point3d &node_position) {
  // Find neighbor's nearest obstacle within radius
  double nearest_obstacle_distance;
  this->NearestOccNodeWithinBox(node_position, desired_obstacle_planning_distance_,
                                &nearest_obstacle_distance);

  // Compute cost associated with obstacle distance
  // Using same cost as in https://ieeexplore.ieee.org/document/7989419
  const double rho = 1.0, nu = 20.0;
  return rho*exp(-nu*(nearest_obstacle_distance-desired_obstacle_planning_distance_));
}

double OctoClass::AverageObstaclePathCost(const std::vector<octomap::point3d> &node_positions) {
  double accumulated_cost = 0.0;
  for (const auto& node_position : node_positions) {
    accumulated_cost = accumulated_cost + this->NearestObstaclePathCost(node_position);
  }
  return accumulated_cost / static_cast<double>(node_positions.size());
}

bool OctoClass::OctoRRG(const Eigen::Vector3d &p0,
                        const Eigen::Vector3d &pf,
                        const Eigen::Vector3d &box_min,
                        const Eigen::Vector3d &box_max,
                        const double &max_time,
                        const int &max_nodes,
                        const double &steer_param,
                        const bool &free_space_only,
                        const bool &prune_result,
                        const bool &publish_rviz,
                        float *plan_time,
                        int *n_rrg_nodes,
                        std::vector<Eigen::Vector3d> *path,
                        visualization_msgs::Marker *graph_markers) {
  // Check whether initial and final points are within box
  if ((p0[0] < box_min[0]) || (p0[1] < box_min[1]) || (p0[2] < box_min[2]) ||
      (p0[0] > box_max[0]) || (p0[1] > box_max[1]) || (p0[2] > box_max[2])) {
    ROS_WARN("[mapper] RRG Error: Initial point not within box!");
    return false;
  }
  if ((pf[0] < box_min[0]) || (pf[1] < box_min[1]) || (pf[2] < box_min[2]) ||
      (pf[0] > box_max[0]) || (pf[1] > box_max[1]) || (pf[2] > box_max[2])) {
    ROS_WARN("[mapper] RRG Error: Final point not within box!");
    return false;
  }

  ROS_INFO("[mapper] Finding RRG path from [%.2f %.2f %.2f] to [%.2f %.2f %.2f]",
           p0[0], p0[1], p0[2], pf[0], pf[1], pf[2]);

  // Check whether the initial and final points are colliding
  if (free_space_only) {
    if (CheckCollision(p0)) {
      ROS_WARN("[mapper] RRG Error: Initial point is colliding!");
      return false;
    }
    if (CheckCollision(pf)) {
      ROS_WARN("[mapper] RRG Error: Final point is colliding!");
      return false;
    }
  } else {
    if (CheckOccupancy(p0) == 1) {
      ROS_WARN("[mapper] RRG Error: Initial point is colliding!");
      return false;
    }
    if (CheckOccupancy(pf) == 1) {
      ROS_WARN("[mapper] RRG Error: Final point is colliding!");
      return false;
    }
  }

  const ros::Time t0 = ros::Time::now();

  // Get free area volume
  double free_vol;
  if (free_space_only) {
    BBXFreeVolume(box_min, box_max, &free_vol);
  } else {
    Eigen::Vector3d range = box_max - box_min;
    double box_vol = range[0]*range[1]*range[2];
    double occ_vol;
    BBXOccVolume(box_min, box_max, &occ_vol);
    free_vol = box_vol - occ_vol;
  }

  // RRG specific parameters
  const double unit_sphere_vol = M_PI*3.0/4.0;
  const double dim = 3.0;
  const double dim_inv = 1.0/dim;
  const double gamma_star = 2.0*pow(1.0+dim_inv, dim_inv)*pow(free_vol/unit_sphere_vol, dim_inv);
  const double gamma = 1.1*gamma_star;

  // Create RRG class and add initial position to the graph
  uint index;
  RRG obj_rrg(steer_param);
  obj_rrg.AddNode(p0, &index);

  // Run RRG until maximum allowed time
  Eigen::Vector3d sample, neighbor_pos;
  static double cost;
  static uint min_index, n_nodes, final_index = 0;
  n_nodes = obj_rrg.rrgraph_.n_nodes_;
  bool connected_graph = false;  // Becomes true when a path has been found from p0 to pf
  while (((ros::Time::now() - t0).toSec() < max_time) && (n_nodes < max_nodes)) {
    // Get a new sample
    obj_rrg.SampleNodeBox(box_min, box_max, &sample);

    // Find nearest node for sample
    obj_rrg.OctoNN(sample, &min_index, &cost);

    // Steer sample
    obj_rrg.Steer(min_index, &sample, &cost);

    // Check for collision
    int col_check;
    neighbor_pos = obj_rrg.rrgraph_.nodes_[min_index].pos_;
    if (free_space_only) {
      col_check = CheckCollision(neighbor_pos, sample);
    } else {
      col_check = CheckOccupancy(neighbor_pos, sample);
    }

    // Stop if collides with nearest neighbor
    if (col_check == 1) {
      continue;
    }

    // Try to add node: fails if there is another node in the same voxel
    obj_rrg.AddNode(sample, &index);
    if (index == 0) {  // Node was not added succesfully
      continue;
    }

    // Get RRG parameter
    n_nodes = obj_rrg.rrgraph_.n_nodes_;
    const double max_dist = std::min(gamma*pow(log(n_nodes)/n_nodes,
                                    dim_inv), steer_param);

    // Get nodes within max_dist radius
    std::vector<uint> near_nodes;
    std::vector<double> costs;
    obj_rrg.NodesWithinRadius(max_dist, sample, &near_nodes, &costs);

    // Add non-colliding nodes within max_dist radius
    for (uint i = 0; i < near_nodes.size(); i++) {
      neighbor_pos = obj_rrg.rrgraph_.nodes_[near_nodes[i]].pos_;
      if (free_space_only) {
        col_check = CheckCollision(neighbor_pos, sample);
      } else {
        col_check = CheckOccupancy(neighbor_pos, sample);
      }

      if (col_check != 1) {
        obj_rrg.AddEdge(index, near_nodes[i], costs[i]);
      }
    }

    // Go back to 'while' if there is a connection between p0 and pf
    if (connected_graph) {
      continue;
    }

    // Check if new node connects with the final destination
    // This portion does not need to execute after one path has been found between p0 and pf
    cost = obj_rrg.DistanceToNode(index, pf);
    if (cost <= max_dist) {
      if (free_space_only) {
        col_check = CheckCollision(pf, sample);
      } else {
        col_check = CheckOccupancy(pf, sample);
      }

      if (col_check != 1) {
        obj_rrg.AddNode(pf, &final_index);
        obj_rrg.AddEdge(index, final_index, cost);
        connected_graph = true;
        ROS_INFO("[mapper] Found connection to destination!");
      }
    }
  }

  // Publish graph into Rviz if requested
  if (publish_rviz) {
    obj_rrg.rrgraph_.GraphVisualization(graph_markers);
  }

  // Calculate Astar from initial node to final node
  std::vector<Eigen::Vector3d> sol_path;
  if (final_index > 0) {
    std::vector<uint> index_path;
    ROS_INFO("[mapper] Trying A* from node %d to node %d!", 0,
                         static_cast<int>(final_index));
    obj_rrg.rrgraph_.Astar2(0, final_index, index_path);

    // Populate final path
    if (index_path.size() == 0) {
      ROS_WARN("[mapper] RRG Error: Couldn't find solution through Astar!");
      *n_rrg_nodes = n_nodes;
      *plan_time = (ros::Time::now() - t0).toSec();
      return false;
    } else {
      for (uint i = 0; i < index_path.size(); i++) {
        sol_path.push_back(obj_rrg.rrgraph_.nodes_[index_path[i]].pos_);
      }
      // ROS_INFO("Path size: %d", static_cast<int>(sol_path.size()));
    }
  } else {
    ROS_WARN("[mapper] RRG Error: Could not find a path from p0 to pf!");
    *n_rrg_nodes = n_nodes;
    *plan_time = (ros::Time::now() - t0).toSec();
    return false;
  }

  // Prune results if requested
  if (prune_result) {
    this->PathPruning(sol_path, free_space_only, path);
  } else {
    *path = sol_path;
  }

  // Populate time and nodes
  *n_rrg_nodes = n_nodes;
  *plan_time = (ros::Time::now() - t0).toSec();

  // ROS_INFO("[mapper] nNodes: %d", static_cast<int>(obj_rrg.rrgraph_.n_nodes_));
  // ROS_INFO("[mapper] nEdges: %d", static_cast<int>(obj_rrg.rrgraph_.n_edges_));

  return true;
}

bool OctoClass::Astar(const octomap::point3d &p0,
                      const octomap::point3d &pf,
                      const bool &prune_result,
                      double *plan_time_sec,
                      std::vector<Eigen::Vector3d> *path,
                      std::vector<Eigen::Vector3d> *pruned_path) {
  const ros::Time t0 = ros::Time::now();

  // Check whether p0 and pf are free nodes in the octomap
  int is_occ;
  is_occ = this->CheckOccupancy(p0);
  if (is_occ == -1) {
    ROS_INFO("[mapper] A* failed: initial node is unknown in the octomap!");
    *plan_time_sec = (ros::Time::now() - t0).toSec();
    return false;
  } else if (is_occ == 1) {
    ROS_INFO("[mapper] A* failed: initial node is occupied in the octomap!");
    *plan_time_sec = (ros::Time::now() - t0).toSec();
    return false;
  }
  is_occ = this->CheckOccupancy(pf);
  if (is_occ == -1) {
    ROS_INFO("[mapper] A* failed: final node is unknown in the octomap!");
    *plan_time_sec = (ros::Time::now() - t0).toSec();
    return false;
  } else if (is_occ == 1) {
    ROS_INFO("[mapper] A* failed: final node is occupied in the octomap!");
    *plan_time_sec = (ros::Time::now() - t0).toSec();
    return false;
  }

  // Get map info
  IndexedKeySet indexed_free_keys;
  std::vector<double> node_sizes;
  Eigen::Vector3d map_min, map_max;
  tree_inflated_.getMetricMin(map_min[0], map_min[1], map_min[2]);
  tree_inflated_.getMetricMax(map_max[0], map_max[1], map_max[2]);
  this->BBXFreeNodes(map_min, map_max, &indexed_free_keys, &node_sizes);
  ROS_INFO("[mapper] A* computing trajectory from [%.3f, %.3f, %.3f] to [%.3f, %.3f, %.3f]",
           p0.x(), p0.y(), p0.z(), pf.x(), pf.y(), pf.z());
  ROS_DEBUG("[mapper] Number of nodes: %d", static_cast<int>(indexed_free_keys.Size()));

  // Find initial and final keys/indexes
  const octomap::OcTreeKey initial_key = tree_inflated_.coordToKey(p0);
  const octomap::OcTreeKey final_key = tree_inflated_.coordToKey(pf);

  uint initial_index;
  if (!indexed_free_keys.Key2Index(initial_key, &initial_index)) {
    ROS_INFO("[mapper] Astar failed: Error retrieving index for initial node!");
    *plan_time_sec = (ros::Time::now() - t0).toSec();
    return false;
  }

  // Astar algorithm variables
  const uint n_nodes = indexed_free_keys.Size();
  PriorityQueue<octomap::OcTreeKey, double> queue;      // octomap::OcTreeKey index, double cost
  std::vector<double> cost_so_far(n_nodes, std::numeric_limits<float>::infinity());
  std::vector<double> obstacle_path_cost(n_nodes, -1.0);
  std::vector<octomap::OcTreeKey> come_from(n_nodes);

  // Initialize algorithm
  queue.put(initial_key, 0.0);
  come_from[initial_index] = initial_key;
  cost_so_far[initial_index] = 0.0;
  while (!queue.empty()) {
    octomap::OcTreeKey current_key = queue.get();
    uint current_index;
    if (!indexed_free_keys.Key2Index(current_key, &current_index)) {
      ROS_INFO("[mapper] Astar failed: Error retrieving index for current node!");
      *plan_time_sec = (ros::Time::now() - t0).toSec();
      return false;
    }

    // Check whether we reached the goal
    octomap::point3d current_pos = tree_inflated_.keyToCoord(current_key);
    if (current_key == final_key) {
      Eigen::Vector3d pos = Eigen::Vector3d(current_pos.x(), current_pos.y(), current_pos.z());
      path->insert(path->begin(), pos);
      while (current_key != initial_key) {
        if (!indexed_free_keys.Key2Index(current_key, &current_index)) {
          ROS_INFO("[mapper] Astar failed when retrieving path from initial to final point!");
          *plan_time_sec = (ros::Time::now() - t0).toSec();
          return false;
        }
        // ROS_INFO("Come from: %d", int(current_index));
        current_key = come_from[current_index];
        current_pos = tree_inflated_.keyToCoord(current_key);
        pos = Eigen::Vector3d(current_pos.x(), current_pos.y(), current_pos.z());
        path->insert(path->begin(), pos);
      }
      break;
    }

    // Check neighbors
    std::vector<octomap::OcTreeKey> neighbor_keys;
    this->GetNodeNeighbors(current_key, node_sizes[current_index], &neighbor_keys);
    const uint n_neighbors = neighbor_keys.size();
    // ROS_INFO("Current index: %d (%f, %f, %f) has %d neighbors", int(current_index),
    //           current_pos.x(), current_pos.y(), current_pos.z(), int(n_neighbors));

    for (uint i = 0; i < n_neighbors; i++) {
      // Get neighbor index
      uint neighbor_index;
      if (!indexed_free_keys.Key2Index(neighbor_keys[i], &neighbor_index)) {
        ROS_INFO("[mapper] Astar failed: Error retrieving index for neighbor node!");
        *plan_time_sec = (ros::Time::now() - t0).toSec();
        return false;
      }

      // This portion avoids us from computing NearestObstaclePathCost, which is an expensive function call.
      // We can prematurely eliminate a neighbor that won't lead us anywhere
      const octomap::point3d neighbor_pos = tree_inflated_.keyToCoord(neighbor_keys[i]);
      double tentative_cost = cost_so_far[current_index] + (current_pos - neighbor_pos).norm();
      if (tentative_cost >= cost_so_far[neighbor_index]) {
        continue;
      }

      // Compute an added cost based on the distance between the neighbor and the nearest obstacle
      if (obstacle_path_cost[neighbor_index] < 0.0) {
        obstacle_path_cost[neighbor_index] = this->NearestObstaclePathCost(neighbor_pos);
      }

      // Check whether this neighbor leads to a new path
      tentative_cost = tentative_cost + obstacle_path_cost[neighbor_index];
      if (tentative_cost < cost_so_far[neighbor_index]) {
        // Fill up A* information for neighbor
        cost_so_far[neighbor_index] = tentative_cost;
        const double priority = tentative_cost + (neighbor_pos - pf).norm();
        queue.put(neighbor_keys[i], priority);
        come_from[neighbor_index] = current_key;
      }
    }
  }

  // Prune results if requested
  if (prune_result) {
    const bool free_space_only = true;
    this->PathPruning(*path, free_space_only, pruned_path);
    ROS_INFO("[mapper]: Path found with %zu waypoints and then compressed to %zu waypoints",
             path->size(), pruned_path->size());
  } else {
    ROS_INFO("[mapper]: Path found with %zu waypoints", path->size());
    pruned_path = path;
  }
  *plan_time_sec = (ros::Time::now() - t0).toSec();

  return true;
}

}  // namespace octoclass
