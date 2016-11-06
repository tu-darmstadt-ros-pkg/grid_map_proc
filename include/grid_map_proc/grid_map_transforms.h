#pragma once

// Grid Map
#include <grid_map_ros/grid_map_ros.hpp>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ros/ros.h>

#include <queue>

namespace grid_map_transforms{

    bool addDistanceTransformCv(grid_map::GridMap& grid_map,
                            std::string occupancy_layer = "occupancy",
                            std::string dist_trans_layer = "distance_transform");

    bool addExplorationTransform(grid_map::GridMap& grid_map,
                            const std::vector<grid_map::Index>& goal_points,
                            std::string occupancy_layer = "occupancy",
                            std::string dist_trans_layer = "distance_transform",
                            std::string expl_trans_layer = "exploration_transform");

    void touchExplorationCell(grid_map::Matrix& grid_map,
                         grid_map::Matrix& expl_trans_map,
                         int idx_x,
                         int idx_y,
                         float curr_val,
                         float add_cost,
                         std::queue<grid_map::Index>& point_queue)
    {
      //If not free at cell, return right away
      if (grid_map(idx_x, idx_y) != 0)
        return;

      float cost = curr_val + add_cost;

      if (expl_trans_map(idx_x, idx_y) > cost){
        expl_trans_map(idx_x, idx_y) = cost;
        point_queue.push(grid_map::Index(idx_x, idx_y));
      }

    }

} /* namespace */
