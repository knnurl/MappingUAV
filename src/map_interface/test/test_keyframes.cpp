// filepath: src/map_interface/test/test_keyframes.cpp
#include <cmath>

#include <gtest/gtest.h>
#include "map_interface/keyframes.hpp"

using map_interface::KeyframeSelector;
using map_interface::Pose;

namespace
{
Pose yawPose(double x, double y, double z, double yaw)
{
  Pose p;
  p.x = x; p.y = y; p.z = z;
  p.qz = std::sin(yaw / 2); p.qw = std::cos(yaw / 2);
  return p;
}
}  // namespace

TEST(Keyframes, WorldToBodyUndoesThePose)
{
  // Vehicle at (2, 1, 1) facing +y (yaw 90 deg). A point 3 m ahead of it is at
  // world (2, 4, 1) and must come back as body (3, 0, 0).
  const Pose p = yawPose(2, 1, 1, M_PI / 2);
  const auto b = map_interface::worldToBody(p, 2, 4, 1);
  EXPECT_NEAR(b[0], 3.0, 1e-9);
  EXPECT_NEAR(b[1], 0.0, 1e-9);
  EXPECT_NEAR(b[2], 0.0, 1e-9);
  const auto left = map_interface::worldToBody(p, 1, 1, 1.5);   // 1 m to its left, 0.5 up
  EXPECT_NEAR(left[0], 0.0, 1e-9);
  EXPECT_NEAR(left[1], 1.0, 1e-9);
  EXPECT_NEAR(left[2], 0.5, 1e-9);
}

TEST(Keyframes, RoundTripWithATiltedPose)
{
  Pose p;                                    // 30 deg about the axis (1,1,0)/sqrt2
  const double h = M_PI / 12, s = std::sin(h) / std::sqrt(2.0);
  p.qx = s; p.qy = s; p.qz = 0; p.qw = std::cos(h);
  p.x = -1; p.y = 0.5; p.z = 2;
  const auto b = map_interface::worldToBody(p, 3.0, -2.0, 0.7);
  // rotate b forward again and compare
  Pose inv = p; inv.qx = -p.qx; inv.qy = -p.qy; inv.qz = -p.qz;
  const auto w = map_interface::rotateInverse(inv, b[0], b[1], b[2]);
  EXPECT_NEAR(w[0] + p.x, 3.0, 1e-9);
  EXPECT_NEAR(w[1] + p.y, -2.0, 1e-9);
  EXPECT_NEAR(w[2] + p.z, 0.7, 1e-9);
}

TEST(Keyframes, SelectsOnDistanceOrAngleAndStopsAtTheBound)
{
  KeyframeSelector sel(0.5, 15.0 * M_PI / 180.0, 4);
  EXPECT_TRUE(sel.select(yawPose(0, 0, 0, 0)));          // first
  EXPECT_FALSE(sel.select(yawPose(0.3, 0, 0, 0.1)));     // 0.3 m, 5.7 deg
  EXPECT_TRUE(sel.select(yawPose(0.5, 0, 0, 0)));        // 0.5 m
  EXPECT_FALSE(sel.select(yawPose(0.6, 0, 0, 0.2)));     // 11 deg
  EXPECT_TRUE(sel.select(yawPose(0.6, 0, 0, 0.3)));      // 17 deg
  EXPECT_TRUE(sel.select(yawPose(0.6, 0, 0.6, 0.3)));    // climb 0.6 m
  EXPECT_TRUE(sel.full());
  EXPECT_FALSE(sel.select(yawPose(9, 9, 9, 3)));         // bound reached
  EXPECT_EQ(sel.count(), 4);
}

TEST(Keyframes, AngleIsSignFreeForEquivalentQuaternions)
{
  Pose a = yawPose(0, 0, 0, 0.4), b = a;
  b.qx = -b.qx; b.qy = -b.qy; b.qz = -b.qz; b.qw = -b.qw;   // same rotation
  EXPECT_NEAR(map_interface::angleBetween(a, b), 0.0, 1e-6);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
