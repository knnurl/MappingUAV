// filepath: src/map_interface/test/test_map_core.cpp
// WP-C unit tests for MapCore: bounded insertion, occupancy tri-state,
// clearance sanity on a synthetic room+pillar scene. No ROS graph needed.
#include <cmath>
#include <vector>

#include <gtest/gtest.h>
#include "map_interface/map_core.hpp"

using map_interface::MapCore;
using map_interface::Bounds;
using map_interface::Occ;
using octomap::point3d;

namespace
{

Bounds roomBounds()
{
  return {-5.0, 5.0, -5.0, 5.0, -0.5, 3.0};
}

// A vertical square pillar of half-width `hw` centred at (px, py), sampled
// on its surface every ~5 cm from z 0..2.5. hw defaults to 0.25 so the faces
// (e.g. x = 1.75 for px = 2.0) sit mid-voxel at 0.2 m resolution: probing
// exactly on voxel boundaries is float-rounding roulette.
std::vector<point3d> pillarSurface(double px, double py, double hw = 0.25)
{
  std::vector<point3d> pts;
  for (double z = 0.0; z <= 2.5; z += 0.05) {
    for (double t = -hw; t <= hw; t += 0.05) {
      pts.emplace_back(px + t, py - hw, z);
      pts.emplace_back(px + t, py + hw, z);
      pts.emplace_back(px - hw, py + t, z);
      pts.emplace_back(px + hw, py + t, z);
    }
  }
  return pts;
}

}  // namespace

TEST(MapCore, StartsUnknownEverywhere)
{
  MapCore core(0.2, roomBounds());
  EXPECT_EQ(core.occupancy(0, 0, 1.0), Occ::UNKNOWN);
  EXPECT_EQ(core.occupancy(2, 2, 1.0), Occ::UNKNOWN);
}

TEST(MapCore, OutsideBoundsIsUnknownAndClearanceRefused)
{
  MapCore core(0.2, roomBounds());
  EXPECT_EQ(core.occupancy(99, 0, 1.0), Occ::UNKNOWN);
  EXPECT_FLOAT_EQ(core.clearance(99, 0, 1.0), -1.0f);
}

TEST(MapCore, PillarBecomesOccupiedRayPathBecomesFree)
{
  MapCore core(0.2, roomBounds());
  const point3d origin(0.0, 0.0, 1.0);
  const size_t n = core.insertCloud(pillarSurface(2.0, 0.0), origin);
  ASSERT_GT(n, 0u);

  // Pillar surface voxel occupied (face plane x=1.75, probe mid-voxel):
  EXPECT_EQ(core.occupancy(1.75, 0.05, 1.05), Occ::OCCUPIED);
  // Ray path between sensor and pillar carved free:
  EXPECT_EQ(core.occupancy(1.05, 0.05, 1.05), Occ::FREE);
  // Behind the pillar: never observed -> unknown:
  EXPECT_EQ(core.occupancy(3.55, 0.05, 1.05), Occ::UNKNOWN);
}

TEST(MapCore, PointsOutsideBoundsAreDropped)
{
  MapCore core(0.2, roomBounds());
  std::vector<point3d> pts = {{50.0, 0.0, 1.0}, {0.0, 50.0, 1.0}};
  EXPECT_EQ(core.insertCloud(pts, point3d(0, 0, 1)), 0u);
}

TEST(MapCore, MaxRangeCutsFarPoints)
{
  Bounds wide{-50, 50, -50, 50, -0.5, 3.0};
  MapCore core(0.2, wide);
  std::vector<point3d> pts = {{20.0, 0.0, 1.0}};
  core.insertCloud(pts, point3d(0, 0, 1), 8.0);
  // The far endpoint must NOT be marked occupied (range-cut inserts free
  // space along the truncated ray instead).
  EXPECT_NE(core.occupancy(20.0, 0.0, 1.0), Occ::OCCUPIED);
}

TEST(MapCore, ClearanceDecreasesTowardPillar)
{
  MapCore core(0.2, roomBounds());
  const point3d origin(0.0, 0.0, 1.0);
  core.insertCloud(pillarSurface(2.0, 0.0), origin);
  core.updateDistanceMap();

  const float far_d = core.clearance(-1.0, 0.0, 1.0);   // ~2.8 m from pillar
  const float near_d = core.clearance(1.0, 0.0, 1.0);   // ~0.8 m from pillar
  ASSERT_GE(far_d, 0.0f);
  ASSERT_GE(near_d, 0.0f);
  EXPECT_GT(far_d, near_d);
  // Near-point clearance should be ~0.8 m within a coarse-voxel tolerance:
  EXPECT_NEAR(near_d, 0.8f, 0.35f);
  // On the pillar itself: ~0.
  EXPECT_LT(core.clearance(2.0, 0.0, 1.0), 0.3f);
}

TEST(MapCore, ClearanceClampsAtCap)
{
  MapCore core(0.2, roomBounds(), /*clearance_cap=*/1.5);
  core.insertCloud(pillarSurface(4.0, 4.0), point3d(0, 0, 1));
  core.updateDistanceMap();
  const float d = core.clearance(-4.0, -4.0, 1.0);  // ~11 m away
  ASSERT_GE(d, 0.0f);
  // dynamicEDT quantizes the cap to whole voxels: 1.5 m at 0.2 m -> 1.6 m.
  EXPECT_LE(d, 1.5f + 0.2f + 0.01f);
}

TEST(MapCore, DirtyFlagLifecycle)
{
  MapCore core(0.2, roomBounds());
  EXPECT_FALSE(core.distanceMapDirty());
  core.insertCloud(pillarSurface(2.0, 0.0), point3d(0, 0, 1));
  EXPECT_TRUE(core.distanceMapDirty());
  core.updateDistanceMap();
  EXPECT_FALSE(core.distanceMapDirty());
}

TEST(MapCore, OccupiedCellsEnumerationMatchesQueries)
{
  MapCore core(0.2, roomBounds());
  core.insertCloud(pillarSurface(2.0, 0.0), point3d(0, 0, 1));
  const auto cells = core.occupiedCells();
  ASSERT_GT(cells.size(), 10u);
  for (const auto & c : cells) {
    EXPECT_EQ(core.occupancy(c.x(), c.y(), c.z()), Occ::OCCUPIED);
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
