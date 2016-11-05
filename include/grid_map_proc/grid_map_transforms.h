#pragma once

// Grid Map
#include <grid_map_ros/grid_map_ros.hpp>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ros/ros.h>


namespace grid_map_transforms{
  
    bool addDistanceTransform(grid_map::GridMap& grid_map,
                            std::string occupancy_layer = "occupancy",
                            std::string dist_trans_layer = "distance_transform");



} /* namespace */
