// filepath: src/map_interface/test/test_voxel_accumulator.cpp
#include <cstdio>
#include <fstream>
#include <limits>
#include <string>

#include <gtest/gtest.h>
#include "map_interface/voxel_accumulator.hpp"

using map_interface::VoxelAccumulator;

namespace
{
std::string tmpPath(const char * name)
{
  return std::string(testing::TempDir()) + name;
}
}  // namespace

TEST(VoxelAccumulator, OnePointPerVoxel)
{
  VoxelAccumulator acc(0.1);
  EXPECT_TRUE(acc.add(0.01f, 0.01f, 0.01f));
  EXPECT_FALSE(acc.add(0.09f, 0.09f, 0.09f));     // same voxel
  EXPECT_TRUE(acc.add(0.11f, 0.01f, 0.01f));      // neighbour
  EXPECT_TRUE(acc.add(-0.01f, 0.01f, 0.01f));     // across zero: floor, not trunc
  EXPECT_EQ(acc.totalVoxels(), 3u);
  EXPECT_EQ(acc.pendingPoints(), 3u);
}

TEST(VoxelAccumulator, RejectsNonFiniteAndFarPoints)
{
  VoxelAccumulator acc(0.05);
  EXPECT_FALSE(acc.add(std::numeric_limits<float>::quiet_NaN(), 0.f, 0.f));
  EXPECT_FALSE(acc.add(0.f, std::numeric_limits<float>::infinity(), 0.f));
  EXPECT_FALSE(acc.add(1e9f, 0.f, 0.f));
  EXPECT_EQ(acc.totalVoxels(), 0u);
  EXPECT_EQ(acc.droppedRange(), 1u);
}

TEST(VoxelAccumulator, BoundedByMaxVoxels)
{
  VoxelAccumulator acc(1.0, 2);
  EXPECT_TRUE(acc.add(0.5f, 0.f, 0.f));
  EXPECT_TRUE(acc.add(1.5f, 0.f, 0.f));
  EXPECT_FALSE(acc.add(2.5f, 0.f, 0.f));
  EXPECT_TRUE(acc.full());
  EXPECT_EQ(acc.droppedFull(), 1u);
}

TEST(VoxelAccumulator, FlushWritesOnlyNewPointsAndLeavesNoTmp)
{
  VoxelAccumulator acc(0.1);
  acc.add(1.0f, 2.0f, 3.0f);
  acc.add(4.0f, 5.0f, 6.0f);
  const std::string a = tmpPath("va_a.pcd");
  EXPECT_EQ(acc.flush(a), 2);
  EXPECT_EQ(acc.pendingPoints(), 0u);
  EXPECT_FALSE(std::ifstream(a + ".tmp").good());
  EXPECT_EQ(acc.flush(tmpPath("va_none.pcd")), 0);   // nothing pending
  EXPECT_FALSE(std::ifstream(tmpPath("va_none.pcd")).good());

  acc.add(1.0f, 2.0f, 3.0f);                          // already saved
  acc.add(7.0f, 8.0f, 9.0f);
  const std::string b = tmpPath("va_b.pcd");
  EXPECT_EQ(acc.flush(b), 1);

  std::ifstream f(b, std::ios::binary);
  std::string line, header;
  while (std::getline(f, line)) {
    header += line + "\n";
    if (line.rfind("DATA", 0) == 0) {break;}
  }
  EXPECT_NE(header.find("POINTS 1\n"), std::string::npos);
  EXPECT_NE(header.find("DATA binary\n"), std::string::npos);
  float xyz[3];
  f.read(reinterpret_cast<char *>(xyz), sizeof(xyz));
  EXPECT_FLOAT_EQ(xyz[0], 7.0f);
  EXPECT_FLOAT_EQ(xyz[2], 9.0f);
  std::remove(a.c_str());
  std::remove(b.c_str());
}

TEST(VoxelAccumulator, FailedWriteKeepsPendingPoints)
{
  VoxelAccumulator acc(0.1);
  acc.add(1.0f, 2.0f, 3.0f);
  EXPECT_EQ(acc.flush("/nonexistent_dir_for_test/x.pcd"), -1);
  EXPECT_EQ(acc.pendingPoints(), 1u);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
