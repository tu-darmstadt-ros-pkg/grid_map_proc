// Regression test for FND-2063: "Mission path shown as straight line between POIs".
//
// Root cause: grid_map_path_planning::shortCutValid only rejected a shortcut segment
// if a cell on the line had distance_transform < 11.0. distance_transform is only
// populated (via BFS, grid_map_transforms::addDistanceTransform) for obstacle cells
// that border reachable free space, and for free cells reached from there - the BFS
// only ever propagates through cells whose occupancy value is exactly 0.0 (free).
//
// An obstacle that is entirely surrounded by unknown/NaN cells (never touching a
// reachable free cell) is therefore never added to the reachable-obstacle seed set
// and its distance_transform value is left at FLT_MAX forever - exactly the
// "isolated wall in NaN sea" scenario from the ticket's investigation (comment
// attachment 106_isolated_wall_in_nan_sea.png). shortCutValid treated
// "FLT_MAX < 11.0" (false) as "safe to cross", so a shortcut whose line ran through
// such an obstacle was incorrectly accepted, producing a straight-line path through
// a wall.
//
// This test builds a map that is free everywhere except for a block of unknown
// (NaN) cells with a small solid obstacle island buried in its center (touching
// only NaN neighbours, never a reachable free cell). It then computes the real
// distance_transform via addDistanceTransform and calls the public shortCutPath()
// API on a 3-point path that detours around the NaN/obstacle blob. shortCutPath
// must NOT collapse this into a straight 2-point path whose line cuts through the
// obstacle island.
#include <catch_ros/catch.hpp>
#include <grid_map_proc/grid_map_path_planning.h>
#include <grid_map_proc/grid_map_transforms.h>

using namespace grid_map;

namespace
{
grid_map::GridMap createIsolatedObstacleInNanSeaMap()
{
  GridMap map({ "occupancy" });
  map.setGeometry(Length(6.0, 6.0), 0.1);  // 60 x 60 cells
  map["occupancy"].setZero();

  // Unknown/NaN blob, disconnected from the reachable free space (the BFS in
  // addDistanceTransform only ever propagates through cells with occupancy == 0.0).
  const int nan_x_start = 20, nan_x_end = 40;
  const int nan_y_start = 20, nan_y_end = 40;
  for (int x = nan_x_start; x < nan_x_end; ++x)
    for (int y = nan_y_start; y < nan_y_end; ++y)
      map["occupancy"](x, y) = std::numeric_limits<float>::quiet_NaN();

  // Solid obstacle island buried in the center of the NaN blob - it only touches
  // NaN neighbours, so it never borders reachable free space and is never added to
  // the distance_transform's obstacle seed set.
  const int obs_x_start = 28, obs_x_end = 32;
  const int obs_y_start = 28, obs_y_end = 32;
  for (int x = obs_x_start; x < obs_x_end; ++x)
    for (int y = obs_y_start; y < obs_y_end; ++y)
      map["occupancy"](x, y) = 100.0;

  return map;
}
}  // namespace

TEST_CASE("shortCutPath does not cut through an obstacle isolated in a NaN sea (FND-2063)",
          "[regression][FND-2063]")
{
  GridMap map = createIsolatedObstacleInNanSeaMap();

  // Seed point in the free space, well away from the NaN/obstacle blob.
  grid_map::Index seed_index(5, 5);

  std::vector<grid_map::Index> obstacle_cells;
  std::vector<grid_map::Index> frontier_cells;
  REQUIRE(grid_map_transforms::addDistanceTransform(map, seed_index, obstacle_cells, frontier_cells));

  // shortCutPath() unconditionally looks up an "exploration_transform" layer (even
  // though shortCutValid itself doesn't use it) - add a dummy one so the call below
  // doesn't throw.
  map.add("exploration_transform", 0.0);

  const grid_map::Matrix& dist_data = map["distance_transform"];
  const grid_map::Matrix& occupancy_data = map["occupancy"];

  // Sanity-check the test setup: the isolated obstacle island must be occupied but
  // never touched by the distance_transform wavefront (still FLT_MAX), since it
  // never borders a reachable free cell. If this fails, the test map no longer
  // reproduces the bug's precondition.
  grid_map::Index isolated_obstacle_cell(30, 30);
  REQUIRE(occupancy_data(isolated_obstacle_cell(0), isolated_obstacle_cell(1)) == 100.0);
  REQUIRE(dist_data(isolated_obstacle_cell(0), isolated_obstacle_cell(1)) == std::numeric_limits<float>::max());

  // A path that correctly detours around the NaN/obstacle blob: left of it -> below
  // it -> right of it.
  std::vector<grid_map::Index> path_in = {
    grid_map::Index(10, 30),  // P0: free space, left of the blob
    grid_map::Index(30, 5),   // P1: free space, below the blob (detour)
    grid_map::Index(50, 30),  // P2: free space, right of the blob
  };
  std::vector<grid_map::Index> path_out;

  REQUIRE(grid_map_path_planning::shortCutPath(map, path_in, path_out));

  // The straight line from P0 to P2 (at y=30) runs directly through the isolated
  // obstacle island buried in the NaN blob. shortCutPath must keep the detour point
  // P1 instead of discarding it, which would mean the planner thinks it's fine to
  // walk straight through the obstacle. Note: when the (buggy) shortcut is wrongly
  // accepted, shortCutPath's own bookkeeping still produces a 3-element path
  // ([P0, P2, P2], i.e. a duplicated final point) rather than collapsing to size 2 -
  // so checking path_out.size() alone is NOT a sufficient regression check here;
  // explicitly verify the detour point survived instead.
  const grid_map::Index detour_point = path_in[1];
  bool detour_point_present = false;
  for (const auto& idx : path_out)
  {
    if (idx(0) == detour_point(0) && idx(1) == detour_point(1))
    {
      detour_point_present = true;
      break;
    }
  }
  CHECK(detour_point_present);
}
