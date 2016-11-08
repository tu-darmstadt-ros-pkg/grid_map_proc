#pragma once

// Grid Map
#include <grid_map_ros/grid_map_ros.hpp>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ros/ros.h>

#include <queue>
#include <nav_msgs/Path.h>

namespace grid_map_path_planning{

    bool findPathExplorationTransform(grid_map::GridMap& grid_map,
                            const geometry_msgs::Pose& start_pose,
                            std::vector<geometry_msgs::PoseStamped>& path,
                            std::string occupancy_layer = "occupancy",
                            std::string expl_trans_layer = "exploration_transform");


    void touchGradientCell(const grid_map::Matrix& expl_trans_map,
                         const grid_map::Index& current_point,
                         const int idx_x,
                         const int idx_y,
                         float& lowest_val,
                         grid_map::Index& lowest_index)
    {
      //If no valid expl transform data, return
      if (expl_trans_map(idx_x, idx_y) == std::numeric_limits<float>::max())
        return;

      float this_delta = expl_trans_map(current_point(0),current_point(1)) - expl_trans_map(idx_x, idx_y);

      //std::cout << "\nloop curr_point:\n" << current_point << "\nval: " << expl_trans_map(current_point(0), current_point(1)) << "\n";
      //std::cout << "\ndelta: " << this_delta << " curr: " << expl_trans_map(current_point(0),current_point(1)) << " test: "<< expl_trans_map(idx_x, idx_y) << "\n";

      if (this_delta > lowest_val){
        lowest_val = this_delta;
        lowest_index = grid_map::Index(idx_x, idx_y);
      }
    }

} /* namespace */
