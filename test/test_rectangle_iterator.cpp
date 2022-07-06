#include "grid_map_proc/rectangle_iterator.h"

#include <catch_ros/catch.hpp>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_core/Polygon.hpp>
#include <ros/console.h>

#include <algorithm>
#include <iostream>
#include <vector>

using namespace grid_map_proc;

// Eigen objects can't be easily compared so std::find doesn't work
static bool contains(const std::vector<grid_map::Index>& indices, const grid_map::Index& index)
{
  for (const auto& index_in_vector : indices)
  {
    if (index.x() == index_in_vector.x() && index.y() == index_in_vector.y())
    {
      return true;
    }
  }
  return false;
}

TEST_CASE("constructor", "RectangleTterator")
{
  grid_map::Position zero_pos;
  grid_map::Polygon polygon({ zero_pos, zero_pos, zero_pos, zero_pos });
  REQUIRE_NOTHROW(RectangleIterator({}, polygon));
}

TEST_CASE("even_size", "RectangleTterator")
{
  const grid_map::Length length{ 6.0, 8.0 };
  const double resolution = 1.0;

  grid_map::GridMap map;
  map.setGeometry(length, resolution);
  std::vector<grid_map::Position> vertices{ { 2.1, 2.6 }, { 2.1, -2.6 }, { -2.1, -2.6 }, { -2.1, 2.6 } };

  std::vector<grid_map::Index> indices;
  for (grid_map_proc::RectangleIterator it(map, { vertices }); !it.pastEnd(); ++it)
  {
    indices.push_back(*it);
  }

  REQUIRE(indices.size() == 24);

  // for (const auto& index_in_vector : indices)
  // {
  //   ROS_ERROR_STREAM("index: " << index_in_vector.transpose());
  // }

  for (size_t x = 1; x < 5; ++x)
  {
    for (size_t y = 1; y < 6; ++y)
    {
      REQUIRE(contains(indices, { x, y }));
    }
  }

  for (size_t x = 0; x < 6; ++x)
  {
    REQUIRE_FALSE(contains(indices, { x, 0 }));
    REQUIRE_FALSE(contains(indices, { x, 7 }));
  }

  for (size_t y = 0; y < 8; ++y)
  {
    REQUIRE_FALSE(contains(indices, { 0, y }));
    REQUIRE_FALSE(contains(indices, { 5, y }));
  }
}

TEST_CASE("odd_size", "RectangleTterator")
{
  const grid_map::Length length{ 5.0, 7.0 };
  const double resolution = 1.0;

  grid_map::GridMap map;
  map.setGeometry(length, resolution);
  std::vector<grid_map::Position> vertices{ { 2.1, 2.6 }, { 2.1, -2.6 }, { -2.1, -2.6 }, { -2.1, 2.6 } };

  std::vector<grid_map::Index> indices;
  for (grid_map_proc::RectangleIterator it(map, { vertices }); !it.pastEnd(); ++it)
  {
    indices.push_back(*it);
  }

  REQUIRE(indices.size() == 25);

  // for (const auto& index_in_vector : indices)
  // {
  //   ROS_ERROR_STREAM("index: " << index_in_vector.transpose());
  // }

  for (size_t x = 0; x < 4; ++x)
  {
    for (size_t y = 1; y < 5; ++y)
    {
      REQUIRE(contains(indices, { x, y }));
    }
  }

  for (size_t x = 0; x < 4; ++x)
  {
    REQUIRE_FALSE(contains(indices, { x, 0 }));
    REQUIRE_FALSE(contains(indices, { x, 6 }));
  }
}

TEST_CASE("45_degrees", "RectangleTterator")
{
  const grid_map::Length length{ 6.0, 8.0 };
  const double resolution = 1.0;

  grid_map::GridMap map;
  map.setGeometry(length, resolution);
  const double value = 2.01;
  std::vector<grid_map::Position> vertices{ { value, 0.0 }, { 0.0, -value }, { -value, 0.0 }, { 0.0, value } };

  std::vector<grid_map::Index> indices;
  for (grid_map_proc::RectangleIterator it(map, { vertices }); !it.pastEnd(); ++it)
  {
    indices.push_back(*it);
  }

  REQUIRE(indices.size() == 12);

  REQUIRE(contains(indices, { 1, 3 }));
  REQUIRE(contains(indices, { 1, 4 }));

  REQUIRE(contains(indices, { 2, 2 }));
  REQUIRE(contains(indices, { 2, 3 }));
  REQUIRE(contains(indices, { 2, 4 }));
  REQUIRE(contains(indices, { 2, 5 }));

  REQUIRE(contains(indices, { 3, 2 }));
  REQUIRE(contains(indices, { 3, 3 }));
  REQUIRE(contains(indices, { 3, 4 }));
  REQUIRE(contains(indices, { 3, 5 }));

  REQUIRE(contains(indices, { 4, 3 }));
  REQUIRE(contains(indices, { 4, 4 }));

  REQUIRE_FALSE(contains(indices, { 1, 2 }));
  REQUIRE_FALSE(contains(indices, { 1, 5 }));
  REQUIRE_FALSE(contains(indices, { 4, 2 }));
  REQUIRE_FALSE(contains(indices, { 4, 5 }));
}
