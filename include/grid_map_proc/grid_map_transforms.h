#pragma once

// Grid Map
#include <grid_map_ros/grid_map_ros.hpp>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ros/ros.h>

#include <queue>

namespace grid_map_transforms{

    /*
     * Adds a inflated layer to the provided grid_map.
     * Note inflation radius is given in map cells.
     */
    bool addInflatedLayer(grid_map::GridMap& map,
                                       const float inflation_radius_map_cells = 6.0,
                                       const std::string occupancy_layer = "occupancy",
                                       const std::string inflated_occupancy_layer = "occupancy_inflated");

    bool addDeflatedLayer(grid_map::GridMap& map,
                                       const float deflation_radius_map_cells = 6.0,
                                       const std::string occupancy_layer = "occupancy",
                                       const std::string deflated_occupancy_layer = "occupancy_deflated");

    bool addDistanceTransformCv(grid_map::GridMap& grid_map,
                            const std::string occupancy_layer = "occupancy",
                            const std::string dist_trans_layer = "distance_transform");

    bool addDistanceTransform(grid_map::GridMap& grid_map,
                              const grid_map::Index& seed_point,
                              std::vector<grid_map::Index>& obstacle_cells,
                              std::vector<grid_map::Index>& frontier_cells,
                              float min_frontier_dist_ = 1.0f,
                              int min_frontier_size = 1,
                              const std::string occupancy_layer = "occupancy",
                              const std::string dist_trans_layer = "distance_transform");

    bool addExplorationTransform(grid_map::GridMap& grid_map,
                            const std::vector<grid_map::Index>& goal_points,
                            const float lethal_dist = 4.0,
                            const float penalty_dist = 12.0,
                            const float penalty_weight = 1.0,
                            const std::string occupancy_layer = "occupancy",
                            const std::string dist_trans_layer = "distance_transform",
                            const std::string expl_trans_layer = "exploration_transform");

    bool collectReachableObstacleCells(grid_map::GridMap& grid_map,
                                       const grid_map::Index& seed_point,
                                       std::vector<grid_map::Index>& obstacle_cells,
                                       std::vector<grid_map::Index>& frontier_cells,
                                       float min_frontier_dist_ = 1.0f,
                                       int min_frontier_size = 1,
                                       const std::string occupancy_layer = "occupancy",
                                       const std::string dist_seed_layer = "dist_seed_transform");

    /**
     * Removes small frontier clusters from the frontier_cell vector.
     * @param expl_trans_map map that was filled in touchObstacleSearchCell()
     * @param frontier_cells
     * @param min_frontier_size all frontiers <= this value are removed
     */
    void filterSmallFrontiersFullySurroundedByKnownCells(grid_map::Matrix& expl_trans_map,
                                                         std::vector<grid_map::Index>& frontier_cells,
                                                         int min_frontier_size);

    void touchExplorationCell(const grid_map::Matrix& grid_map,
                              const grid_map::Matrix& dist_map,
                         grid_map::Matrix& expl_trans_map,
                         const int idx_x,
                         const int idx_y,
                         const float curr_val,
                         const float add_cost,
                         const float lethal_dist,
                         const float penalty_dist,
                         const float penalty_weight,
                         std::queue<grid_map::Index>& point_queue);

    void touchDistCell(const grid_map::Matrix& grid_map,
                         grid_map::Matrix& expl_trans_map,
                         const int idx_x,
                         const int idx_y,
                         const float curr_val,
                         const float add_cost,
                         std::queue<grid_map::Index>& point_queue);

    void touchObstacleSearchCell(const grid_map::Matrix& grid_map,
                         grid_map::Matrix& expl_trans_map,
                         float resolution,
                         const grid_map::Index& start_point,
                         const grid_map::Index& current_point,
                         const int idx_x,
                         const int idx_y,
                         std::vector<grid_map::Index>& obstacle_cells,
                         std::vector<grid_map::Index>& frontier_cells,
                         std::queue<grid_map::Index>& point_queue,
                         float min_distance = 1.0f);

    /**
     * Tests whether the cell corresponding to the given index is a frontier cell.
     * If so it is added to the current frontier_cluster.
     * @param expl_trans_map
     * @param index
     * @param point_queue
     * @param frontier_cell_cluster
     */
    void touchFilterCell(grid_map::Matrix& expl_trans_map,
                         const grid_map::Index& index,
                         std::queue<grid_map::Index>& point_queue,
                         std::vector<grid_map::Index>& frontier_cell_cluster);


    class InflatedLayerProvider
    {
      public:
        bool addInflatedLayer(grid_map::GridMap& map,
                                       const float inflation_radius_map_cells = 6.0,
                                       const std::string occupancy_layer = "occupancy",
                                       const std::string inflated_occupancy_layer = "occupancy_inflated");

      protected:

        void ensureMatSize(cv::Mat& mat, int rows, int cols, int type);

        cv::Mat map_mat_;
        cv::Mat inflated_mat_;

    };
    
} /* namespace */
