#include "grid_map_proc/rectangle_iterator.h"

#include <ros/console.h>

#include <grid_map_core/GridMapMath.hpp>
#include <grid_map_core/SubmapGeometry.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

namespace grid_map_proc
{
RectangleIterator::RectangleIterator(const grid_map::GridMap& map, const grid_map::Polygon& rectangle_polygon)
  : past_end_{ false }
{
  std::vector<grid_map::Position> vertices = rectangle_polygon.getVertices();
  if (vertices.size() != 4U)
  {
    std::string error{ "RectangleIterator needs 4 point polygon, got size " + std::to_string(vertices.size()) };
    ROS_FATAL_STREAM(error);
    throw std::domain_error(error);
  }

  const grid_map::Position& front_point = vertices.front();
  double min_x = front_point.x();
  double max_x = front_point.x();

  double min_y = front_point.y();
  double max_y = front_point.y();

  for (uint8_t i = 1; i < 4U; ++i)
  {
    const grid_map::Position& point = vertices.at(i);
    min_x = std::min(min_x, point.x());
    max_x = std::max(max_x, point.x());

    min_y = std::min(min_y, point.y());
    max_y = std::max(max_y, point.y());
  }

  std::sort(vertices.begin(), vertices.end(),
            [](const grid_map::Position& a, const grid_map::Position& b) { return a.y() < b.y(); });

  // axis:
  // ^ x
  // |
  // ----> y
  const Eigen::Vector2d left_top = vertices.at(0).x() > vertices.at(1).x() ? vertices.at(0) : vertices.at(1);
  const Eigen::Vector2d left_down = vertices.at(0) != left_top ? vertices.at(0) : vertices.at(1);

  const Eigen::Vector2d right_top = vertices.at(2).x() > vertices.at(3).x() ? vertices.at(2) : vertices.at(3);
  const Eigen::Vector2d right_down = vertices.at(2) != right_top ? vertices.at(2) : vertices.at(3);

  x_length_half_ = 0.5 * (left_top - left_down).norm();
  y_length_half_ = 0.5 * (right_top - left_top).norm();
  const Eigen::Vector2d center = left_down + (0.5 * (right_top - left_down));

  const Eigen::Vector2d x_vector = left_top - left_down;
  const double angle = std::atan2(x_vector.y(), x_vector.x());

  rectangle_transform_ = Eigen::Rotation2Dd(-angle) * Eigen::Translation2d(-center);

  const auto get_index = [&map](const double x, const double y) -> grid_map::Index {
    grid_map::Position position{ x, y };
    position = map.getClosestPositionInMap(position);

    grid_map::Index index;
    map.getIndex(position, index);

    return index;
  };

  // Position coordinates and index directions are flipped
  start_index_ = get_index(max_x, max_y);
  end_index_ = get_index(min_x, min_y);

  const grid_map::Vector map_center = 0.5 * map.getLength();
  map_offset_ = (map_center.array() - 0.5 * map.getResolution()).matrix();

  map_position_ = map.getPosition();
  map_resolution_ = map.getResolution();
  map_buffer_size_ = map.getSize();
  map_buffer_start_index_ = map.getStartIndex();

  // Move to first valid position
  index_ = start_index_.array() - 1;
  ++(*this);
}

RectangleIterator& RectangleIterator::operator++()
{
  while (nextStep())
  {
    if (isInside())
    {
      break;
    }
  }

  return *this;
}

bool RectangleIterator::nextStep()
{
  if (index_.x() < end_index_.x())
  {
    ++index_.x();
  }
  else
  {
    index_.x() = start_index_.x();
    ++index_.y();
  }

  past_end_ = index_.y() > end_index_.y();
  return !past_end_;
}

bool RectangleIterator::isInside()
{
  const grid_map::Index unwrappedIndex =
      grid_map::getIndexFromBufferIndex(index_, map_buffer_size_, map_buffer_start_index_);
  const grid_map::Vector index_vector{ -unwrappedIndex.x(), -unwrappedIndex.y() };

  const grid_map::Position position = map_position_ + map_offset_ + map_resolution_ * index_vector;

  const grid_map::Position rect_position = rectangle_transform_ * position;

  return std::abs(rect_position.x()) <= x_length_half_ && std::abs(rect_position.y()) <= y_length_half_;
}

bool RectangleIterator::pastEnd() const
{
  return past_end_;
}

RectangleIterator::reference RectangleIterator::operator*()
{
  return index_;
}
RectangleIterator::pointer RectangleIterator::operator->()
{
  return &index_;
}

bool RectangleIterator::operator==(const RectangleIterator& other)
{
  return index_.x() == other.index_.x() && index_.y() == other.index_.y();
}
bool RectangleIterator::operator!=(const RectangleIterator& other)
{
  return index_.x() != other.index_.x() || index_.y() != other.index_.y();
}

}  // namespace grid_map_proc
