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

    bool addDistanceTransform(grid_map::GridMap& grid_map,
                              const grid_map::Index& seed_point,
                              std::string occupancy_layer = "occupancy",
                              std::string dist_trans_layer = "distance_transform");

    bool addExplorationTransform(grid_map::GridMap& grid_map,
                            const std::vector<grid_map::Index>& goal_points,
                            std::string occupancy_layer = "occupancy",
                            std::string dist_trans_layer = "distance_transform",
                            std::string expl_trans_layer = "exploration_transform");


    bool collectReachableObstacleCells(grid_map::GridMap& grid_map,
                                       const grid_map::Index& seed_point,
                                       std::vector<grid_map::Index>& obstacle_cells,
                                       std::vector<grid_map::Index>& frontier_cells,
                                       std::string occupancy_layer = "occupancy",
                                       std::string dist_seed_layer = "dist_seed_transform");

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

    void touchObstacleSearchCell(grid_map::Matrix& grid_map,
                         grid_map::Matrix& expl_trans_map,
                         const grid_map::Index& current_point,
                         int idx_x,
                         int idx_y,
                         float curr_val,
                         float add_cost,
                         std::vector<grid_map::Index>& obstacle_cells,
                         std::vector<grid_map::Index>& frontier_cells,
                         std::queue<grid_map::Index>& point_queue)
    {
      // Free
      if ( (grid_map(idx_x, idx_y) == 0.0) ){
        if (expl_trans_map(idx_x, idx_y) != std::numeric_limits<float>::max()){
          return;
        }else{
          expl_trans_map(idx_x, idx_y) = -3.0;
          point_queue.push(grid_map::Index(idx_x, idx_y));
        }
      // Occupied
      }else if (grid_map(idx_x, idx_y) == 100.0){
        if (expl_trans_map(idx_x, idx_y) == -1.0){
          return;
        }else{
          expl_trans_map(idx_x, idx_y) = -1.0;
          obstacle_cells.push_back(grid_map::Index(idx_x, idx_y));
        }
      // Unknown
      }else{
        if (expl_trans_map(current_point(0), current_point(1)) == -2.0){
          return;
        }else{
          expl_trans_map(current_point(0), current_point(1)) = -2.0;
          frontier_cells.push_back(grid_map::Index(current_point(0), current_point(1)));
        }
      }


    }



} /* namespace */
