#include <grid_map_proc/grid_map_polygon_tools.h>

#include <eigen_conversions/eigen_msg.h>

#include <nav_msgs/Path.h>

namespace grid_map_polygon_tools{
  
  
  void setFootprintPoly(const double footprint_x, const double footprint_y, grid_map::Polygon& poly, const std::string& frame_id)
  {
    poly.removeVertices();
    poly.setFrameId(frame_id);
    poly.addVertex(grid_map::Position( footprint_x,  footprint_y));
    poly.addVertex(grid_map::Position(-footprint_x,  footprint_y));
    poly.addVertex(grid_map::Position(-footprint_x, -footprint_y));
    poly.addVertex(grid_map::Position( footprint_x, -footprint_y));
  }

  void printPolyInfo(const grid_map::Polygon& poly)
  {
    const std::vector<grid_map::Position> positions = poly.getVertices();

    ROS_INFO("----");

    for (size_t i = 0; i < positions.size(); ++i){
      ROS_INFO("Vertex %d Position x: %f y: %f", (int)i, positions[i].x(), positions[i].y());
    }
  }
  
  grid_map::Polygon getTransformedPoly(const grid_map::Polygon& poly, const geometry_msgs::Pose& pose, const std::string& frame_id)
  {
    Eigen::Affine3d transform;
    tf::poseMsgToEigen(pose, transform);
    return getTransformedPoly(poly, transform);
  }
  
  
  grid_map::Polygon getTransformedPoly(const grid_map::Polygon& poly, const Eigen::Affine3d& pose, const std::string& frame_id)
  {
    grid_map::Polygon out_poly;
    out_poly.setFrameId("world");

    const std::vector<grid_map::Position>& vertices = poly.getVertices();

    for (size_t i  = 0; i < vertices.size(); ++i){
      Eigen::Vector3d tmp(pose * Eigen::Vector3d(vertices[i](0), vertices[i](1), 0.0));
      out_poly.addVertex(grid_map::Position(tmp.x(), tmp.y()));
    }

    return out_poly;
  }

  bool isPathInCollision(const grid_map::Polygon&  poly,
                         const grid_map::GridMap& grid_map,
                         const nav_msgs::Path& path,
                         const std::string& layer)
  {
    if (!grid_map.exists(layer)){
      ROS_ERROR("Requested layer %s does not exist in grid map, cannot check path for collisions!", layer.c_str());
      return false;
    }

    const grid_map::Matrix& map_data = grid_map[layer];

    for (size_t i = 0; i < path.poses.size(); ++i){
      grid_map::Polygon poly = getTransformedPoly(poly, path.poses[i].pose);

      for (grid_map::PolygonIterator poly_iterator(grid_map, poly); !poly_iterator.isPastEnd(); ++poly_iterator) {
        const grid_map::Index index(*poly_iterator);

        //std::cout << "idx: " << index(0) << " " << index(1) << "\n";

        //map_data(index(0), index(1)) = 100.0;
      }

    }

    return false;
  }

  /**
   * @brief isPathInCollisionElevation checks if a path is in collision based on elevation map data
   * @param poly The footprint polygon
   * @param grid_map The grid (elevation) map
   * @param path The path containg poses to be checked
   * @param robot_elevation The elevation of the robot
   * @param elevation_threshold The maximum allowed difference between elevation of the robot and grid cells
   * @param obstacle_pose The pose where a obstacle has been detected
   * @param layer The layer in the grid map to use
   * @return
   */
  bool isPathInCollisionElevation(const grid_map::Polygon&  poly,
                         const grid_map::GridMap& grid_map,
                         const nav_msgs::Path& path,
                         const double robot_elevation,
                         const double elevation_threshold,
                         geometry_msgs::Pose& obstacle_pose,
                         const double min_dist,
                         const double max_dist,
                         const double found_obstacle_height_offset,
                         const std::string& layer)
  {
    if (!grid_map.exists(layer)){
      ROS_WARN("Requested layer %s does not exist in grid map, cannot check path for collisions!", layer.c_str());
      return false;
    }

    ROS_DEBUG("Checking path with %d poses for collisions.", (int)path.poses.size());

    double dist = 0.0;

    const grid_map::Matrix& elev_data = grid_map[layer];

    for (size_t i = 0; i < path.poses.size(); ++i){
      grid_map::Polygon transformed_poly = getTransformedPoly(poly, path.poses[i].pose);

      //printPolyInfo(transformed_poly);

      //std::cout << i << "\n";


      if (i > 0){
        dist += (Eigen::Vector2d(path.poses[i].pose.position.x, path.poses[i].pose.position.y) -
                 Eigen::Vector2d(path.poses[i-1].pose.position.x, path.poses[i-1].pose.position.y)).norm();
      }

      //ROS_INFO("dist: %f", dist);

      if (dist > max_dist){
        return false;
      }else if (dist > min_dist){

        for (grid_map::PolygonIterator poly_iterator(grid_map, transformed_poly); !poly_iterator.isPastEnd(); ++poly_iterator) {

          const grid_map::Index index(*poly_iterator);


          //if (grid_map.isValid(index)){
          //std::cout << "re: " << robot_elevation << " el " <<elev_data(index(0), index(1)) << "\n";
          if ( std::abs( robot_elevation - elev_data(index(0), index(1)) ) > elevation_threshold ){

            grid_map::Position position;
            grid_map.getPosition(index, position);

            obstacle_pose.position.x = position.x();
            obstacle_pose.position.y = position.y();
            obstacle_pose.position.z = position.z() + found_obstacle_height_offset;

            obstacle_pose.orientation = path.poses[i].pose.orientation;

            return true;
          }


        }
      }



    }

    return false;

  }


  void segmentObstacle(const grid_map::GridMap& grid_map,
                  const geometry_msgs::Pose& obstacle_initial_pose,
                  const double robot_height,
                  const double height_threshold,
                  grid_map::Polygon& obstacle_poly,
                  const std::string& layer)
  {



  }

} /* namespace */
