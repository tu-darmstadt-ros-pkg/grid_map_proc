#pragma once

#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/iterators/SubmapIterator.hpp>
#include <grid_map_core/Polygon.hpp>
#include <grid_map_core/TypeDefs.hpp>

#include <Eigen/Core>

#include <iterator>
#include <memory>

namespace grid_map_proc
{
class RectangleIterator
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using iterator_category = std::forward_iterator_tag;
  using value_type = grid_map::Index;
  using difference_type = std::ptrdiff_t;
  using pointer = value_type*;
  using reference = value_type&;

  RectangleIterator(const grid_map::GridMap& map, const grid_map::Polygon& rectangle_polygon);

  RectangleIterator& operator++();

  bool pastEnd() const;

  reference operator*();
  pointer operator->();

  bool operator==(const RectangleIterator& other);
  bool operator!=(const RectangleIterator& other);

private:
  bool nextStep();
  bool isInside();

  grid_map::Polygon polygon_;
  grid_map::Vector map_offset_;
  grid_map::Position map_position_;
  double map_resolution_;
  grid_map::Size map_buffer_size_;
  grid_map::Index map_buffer_start_index_;

  grid_map::Index start_index_;
  grid_map::Index end_index_;
  grid_map::Index index_;

  bool past_end_;
};

}  // namespace grid_map_proc
