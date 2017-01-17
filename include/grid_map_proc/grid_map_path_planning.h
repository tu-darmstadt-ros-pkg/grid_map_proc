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
                            float* path_cost = 0,
                            const std::string occupancy_layer = "occupancy",
                            const std::string dist_trans_layer = "distance_transform",
                            const std::string expl_trans_layer = "exploration_transform");


    bool findValidClosePoseExplorationTransform(grid_map::GridMap& grid_map,
                            const grid_map::Index& start_index,
                            grid_map::Index& adjusted_index,
                            const float allowed_distance_from_start = 3.0,
                            const float required_final_distance = 6.0,
                            const float desired_final_distance = 12.0,
                            const std::string occupancy_layer = "occupancy",
                            const std::string dist_trans_layer = "distance_transform",
                            const std::string expl_trans_layer = "exploration_transform");

    // @TODO Unused, possibly remove
    bool shortCutPath(grid_map::GridMap& grid_map,
                      const std::vector <grid_map::Index>& path_in,
                      std::vector <grid_map::Index>& path_out,
                      const std::string dist_trans_layer = "distance_transform",
                      const std::string expl_trans_layer = "exploration_transform");


    void touchDistanceField(const grid_map::Matrix& dist_trans_map,
                         const grid_map::Index& current_point,
                         const int idx_x,
                         const int idx_y,
                         float& highest_val,
                         grid_map::Index& highest_index)
    {
      //If no valid expl transform data, return
      if (dist_trans_map(idx_x, idx_y) == std::numeric_limits<float>::max())
        return;

      float this_delta = dist_trans_map(idx_x, idx_y) - dist_trans_map(current_point(0),current_point(1));

      if ( (this_delta > 0.0f) && (this_delta > highest_val) ){
        highest_val = this_delta;
        highest_index = grid_map::Index(idx_x, idx_y);
      }
    }



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

    bool shortCutValid(const grid_map::GridMap& grid_map,
                      const grid_map::Matrix& dist_trans_map,
                      const grid_map::Index& start_point,
                      const grid_map::Index& end_point)
    {

      for (grid_map::LineIterator iterator (grid_map, start_point, end_point);
           !iterator.isPastEnd(); ++iterator) {

         const grid_map::Index index(*iterator);

         if ( (dist_trans_map(index(0), index(1)) < 11.0) &&
              !( (index(0) == end_point(0)) && (index(1) == end_point(1))) ){
           return false;
         }

      }
      return true;

    }

} /* namespace */
