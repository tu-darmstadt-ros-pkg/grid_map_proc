#pragma once

// Grid Map
#include <grid_map_ros/grid_map_ros.hpp>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <geometry_msgs/PoseStamped.h>

#include <ros/ros.h>

#include <grid_map_core/Polygon.hpp>


namespace grid_map_polygon_tools{
  
  // Symmetric polygon around footprint
  void setFootprintPoly(const double footprint_x, const double footprint_y, grid_map::Polygon& poly, const std::string& frame_id = "base_link");
  
  // Poly from 2 corner points 
  void setFootprintPoly(const Eigen::Vector2d& bottom_left, const Eigen::Vector2d& top_right, grid_map::Polygon& poly, const std::string& frame_id = "base_link");
  
  
  grid_map::Polygon getTransformedPoly(const grid_map::Polygon& poly, const geometry_msgs::Pose& pose, const std::string& frame_id = "world");
  grid_map::Polygon getTransformedPoly(const grid_map::Polygon& poly, const Eigen::Affine3d& pose, const std::string& frame_id = "world");




} /* namespace */
