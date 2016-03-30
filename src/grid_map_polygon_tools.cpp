#include <grid_map_proc/grid_map_polygon_tools.h>

#include <eigen_conversions/eigen_msg.h>

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
      //std::cout << tmp << "\n";
      out_poly.addVertex(grid_map::Position(tmp.x(), tmp.y()));
    }

    return out_poly;
  }

} /* namespace */
