#pragma once

// Grid Map
#include <grid_map_ros/grid_map_ros.hpp>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <geometry_msgs/PoseStamped.h>

#include <ros/ros.h>

#include <grid_map_core/Polygon.hpp>

#include <nav_msgs/Path.h>


namespace grid_map_polygon_tools{


  grid_map::Polygon fromRosPoints(std::vector<geometry_msgs::Point> points);


  // Symmetric polygon around footprint
  void setFootprintPoly(const double footprint_x, const double footprint_y, grid_map::Polygon& poly, const std::string& frame_id = "base_link");

  // Poly from 2 corner points
  void setFootprintPoly(const Eigen::Vector2d& bottom_left, const Eigen::Vector2d& top_right, grid_map::Polygon& poly, const std::string& frame_id = "base_link");

  void printPolyInfo(const grid_map::Polygon& poly);



  grid_map::Polygon getTransformedPoly(const grid_map::Polygon& poly, const geometry_msgs::Pose& pose, const std::string& frame_id = "world");
  grid_map::Polygon getTransformedPoly(const grid_map::Polygon& poly, const Eigen::Affine3d& pose, const std::string& frame_id = "world");

  //grid_map::Polygon

  bool isPathInCollision(const grid_map::Polygon&  poly,
                         const grid_map::GridMap& grid_map,
                         const nav_msgs::Path& path,
                         const float& lower_threshold = 0.0,
                         const float& upper_threshold = 10.0,
                         const std::string& layer = "occupancy");

  /**
   * Check for each pose of given path if it is in collision based on given map
   * @param poly footprint polygon
   * @param grid_map grid map on which the path is checked
   * @param path path containing poses to be checked
   * @param lower_threshold lower threshold for collision check
   * @param upper_threshold upper threshold for collision check
   * @param layer layer in map to check
   * @return vector containing one bool per pose in path with true = pose is in collision, false = pose is not in collision
   */
  std::vector<bool> isPathInCollisionPerPose(const grid_map::Polygon&  poly,
                                             const grid_map::GridMap& grid_map,
                                             const nav_msgs::Path& path,
                                             const float& lower_threshold = 0.0,
                                             const float& upper_threshold = 10.0,
                                             const std::string& layer = "occupancy");


  bool isPathInCollisionElevation(const grid_map::Polygon&  poly,
                         const grid_map::GridMap& grid_map,
                         const nav_msgs::Path& path,
                         const double robot_height,
                         const double height_threshold,
                         geometry_msgs::Pose& obstacle_pose,
                         const double min_dist = 0.0,
                         const double max_dist = 10.0,
                         const double found_obstacle_height_offset = 0.0,
                         const std::string& layer = "elevation");

  void segmentObstacle(const grid_map::GridMap& grid_map,
                  const geometry_msgs::Pose& obstacle_initial_pose,
                  const double robot_height,
                  const double height_threshold,
                  grid_map::Polygon& obstacle_poly,
                  const std::string& layer);




} /* namespace */
