// filepath: src/map_interface/include/map_interface/voxel_accumulator.hpp
// In-flight 3D map product (operator request 2026-11-16): keep one point per
// voxel of everything /cloud_registered has shown, and write what is new since
// the last checkpoint as a binary PCD chunk. No ROS types: unit-testable.
//
// Memory is bounded by max_voxels (about 50 bytes per voxel in the set).
// Chunks are written to "<name>.tmp" and renamed, so a file that exists is
// complete: a crash or a power cut loses at most one checkpoint period.
#pragma once

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

namespace map_interface
{

class VoxelAccumulator
{
public:
  explicit VoxelAccumulator(double voxel_size, size_t max_voxels = 4000000)
  : inv_(1.0 / voxel_size), max_voxels_(max_voxels)
  {
    if (!(voxel_size > 0.0)) {
      throw std::invalid_argument("voxel_size must be > 0");
    }
  }

  // Returns true if the point opened a new voxel (and was kept).
  bool add(float x, float y, float z)
  {
    if (!(std::isfinite(x) && std::isfinite(y) && std::isfinite(z))) {
      return false;
    }
    const int64_t ix = static_cast<int64_t>(std::floor(x * inv_));
    const int64_t iy = static_cast<int64_t>(std::floor(y * inv_));
    const int64_t iz = static_cast<int64_t>(std::floor(z * inv_));
    const int64_t lim = (int64_t{1} << 20);          // +-1,048,576 voxels per axis
    if (ix < -lim || ix >= lim || iy < -lim || iy >= lim || iz < -lim || iz >= lim) {
      ++dropped_range_;
      return false;
    }
    if (seen_.size() >= max_voxels_) {
      ++dropped_full_;
      return false;
    }
    const uint64_t key = (static_cast<uint64_t>(ix + lim) << 42) |
      (static_cast<uint64_t>(iy + lim) << 21) | static_cast<uint64_t>(iz + lim);
    if (!seen_.insert(key).second) {
      return false;
    }
    pending_.push_back(x);
    pending_.push_back(y);
    pending_.push_back(z);
    return true;
  }

  size_t pendingPoints() const {return pending_.size() / 3;}
  size_t totalVoxels() const {return seen_.size();}
  size_t droppedFull() const {return dropped_full_;}
  size_t droppedRange() const {return dropped_range_;}
  bool full() const {return seen_.size() >= max_voxels_;}

  // Write the pending points to `path` (binary PCD, x y z float32) through a
  // temporary file, then forget them. Returns the number written; 0 with
  // nothing pending; -1 on an I/O error (pending points are kept for a retry).
  long flush(const std::string & path)
  {
    const size_t n = pendingPoints();
    if (n == 0) {
      return 0;
    }
    const std::string tmp = path + ".tmp";
    FILE * f = std::fopen(tmp.c_str(), "wb");
    if (f == nullptr) {
      return -1;
    }
    bool ok = std::fprintf(f,
        "# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\n"
        "FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
        "WIDTH %zu\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %zu\nDATA binary\n",
        n, n) > 0;
    ok = ok && std::fwrite(pending_.data(), sizeof(float), pending_.size(), f) ==
      pending_.size();
    ok = (std::fflush(f) == 0) && ok;
    ok = (std::fclose(f) == 0) && ok;
    if (!ok || std::rename(tmp.c_str(), path.c_str()) != 0) {
      std::remove(tmp.c_str());
      return -1;
    }
    pending_.clear();
    return static_cast<long>(n);
  }

private:
  double inv_;
  size_t max_voxels_;
  std::unordered_set<uint64_t> seen_;
  std::vector<float> pending_;
  size_t dropped_full_{0};
  size_t dropped_range_{0};
};

}  // namespace map_interface
