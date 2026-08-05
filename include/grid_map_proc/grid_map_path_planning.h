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

    bool adjustStartPoseIfOccupied(const grid_map::GridMap& grid_map,
                                   const geometry_msgs::Pose& start_pose,
                                   geometry_msgs::Pose& revised_start_pose,
                                   const std::string occupancy_layer = "occupancy",
                                   const std::string dist_trans_layer = "distance_transform",
                                   const std::string expl_trans_layer = "exploration_transform");



    bool findValidClosePoseExplorationTransform(const grid_map::GridMap& grid_map,
                            const grid_map::Index& start_index,
                            grid_map::Index& adjusted_index,
                            const float allowed_distance_from_start = 3.0,
                            const float required_final_distance = 6.0,
                            const float desired_final_distance = 12.0,
                            const std::string occupancy_layer = "occupancy",
                            const std::string dist_trans_layer = "distance_transform",
                            const std::string expl_trans_layer = "exploration_transform");

    /**
     * @brief Stopping criteria for the findCloseIndexMatchingCriteria function.
     * @param grid_map The grid map.
     * @param start_index The start index.
     * @param current_index The current index.
     */
    using StoppingCriteria = std::function<bool(const grid_map::GridMap& grid_map, const grid_map::Index& start_index,
                                                const grid_map::Index& current_index)>;

    struct IndexComparator
    {
      bool operator()(const grid_map::Index& lhs, const grid_map::Index& rhs) const
      {
        if (lhs[0] != rhs[0])
        {
          return lhs[0] < rhs[0];
        }
        return lhs[1] < rhs[1];
      }
    };

    /**
     * @brief Find a close cell (index) that meets the given criteria using a BFS algorithm.
     *
     * @param criteria The criteria to be met by the close cell. See StoppingCriteria.
     * @param close_index The resulting close cell index.
     * @param max_distance_from_start_in_cells The maximum distance from the start cell in cells. If <= 0, the search is
     * unbounded.
     */
    bool findCloseIndexMatchingCriteria(const grid_map::GridMap& grid_map, const grid_map::Index& start_index,
                                        StoppingCriteria criteria, grid_map::Index& close_index,
                                        const double max_distance_from_start_in_cells = 0.0);

    bool shortCutPath(grid_map::GridMap& grid_map,
                      const std::vector <grid_map::Index>& path_in,
                      std::vector <grid_map::Index>& path_out,
                      const std::string dist_trans_layer = "distance_transform",
                      const std::string expl_trans_layer = "exploration_transform",
                      const std::string occupancy_layer = "occupancy");


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
                      const grid_map::Matrix& occupancy_map,
                      const grid_map::Index& start_point,
                      const grid_map::Index& end_point)
    {

      for (grid_map::LineIterator iterator (grid_map, start_point, end_point);
           !iterator.isPastEnd(); ++iterator) {

         const grid_map::Index index(*iterator);

         bool is_end_point = (index(0) == end_point(0)) && (index(1) == end_point(1));

         // Reject the shortcut outright if it runs through a cell that is not
         // confirmed free (occupied OR unknown/NaN). This is required in addition
         // to the distance_transform check below: distance_transform is only
         // populated (via BFS) for obstacle cells that border reachable free space
         // and for free cells reached from there. Cells that are never touched by
         // that wavefront (e.g. the interior of a thick wall, or an obstacle
         // surrounded by unknown/NaN space) are left at FLT_MAX, which incorrectly
         // passes the "< 11.0" check below and would let a shortcut cut straight
         // through such an obstacle.
         if ( (occupancy_map(index(0), index(1)) != 0.0) && !is_end_point ){
           return false;
         }

         if ( (dist_trans_map(index(0), index(1)) < 11.0) && !is_end_point ){
           return false;
         }

      }
      return true;

    }

} /* namespace */
