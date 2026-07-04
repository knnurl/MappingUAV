// filepath: src/map_interface/include/map_interface/map_core.hpp
// Pure mapping core (WP-C cpu_grid backend): octomap occupancy + dynamicEDT3D
// clearance, bounded to a world-frame box. No ROS types — unit-testable
// without a graph. Frames [FIXED]: everything here is odom/ENU; planners
// never consume NED (masterplan §1.2).
#pragma once

#include <memory>
#include <vector>

#include <octomap/octomap.h>
#include <dynamicEDT3D/dynamicEDTOctomap.h>

namespace map_interface
{

enum class Occ : uint8_t { FREE = 0, OCCUPIED = 1, UNKNOWN = 2 };

struct Bounds
{
  double x_min, x_max, y_min, y_max, z_min, z_max;
  bool contains(double x, double y, double z) const
  {
    return x >= x_min && x <= x_max && y >= y_min && y <= y_max &&
           z >= z_min && z <= z_max;
  }
};

class MapCore
{
public:
  // clearance_cap: distances are exact up to this value and clamp at it.
  MapCore(double resolution, const Bounds & b, double clearance_cap = 4.0,
          bool unknown_as_occupied = false)
  : bounds_(b), clearance_cap_(clearance_cap)
  {
    tree_ = std::make_unique<octomap::OcTree>(resolution);
    edt_ = std::make_unique<DynamicEDTOctomap>(
      static_cast<float>(clearance_cap), tree_.get(),
      octomap::point3d(b.x_min, b.y_min, b.z_min),
      octomap::point3d(b.x_max, b.y_max, b.z_max),
      unknown_as_occupied);
  }

  // Insert a world-frame cloud taken from sensor_origin. Points outside the
  // bounds are dropped BEFORE insertion (the fence is the map's universe).
  // max_range < 0 disables the range cut. Returns points actually inserted.
  size_t insertCloud(const std::vector<octomap::point3d> & pts,
                     const octomap::point3d & sensor_origin,
                     double max_range = -1.0)
  {
    octomap::Pointcloud pc;
    for (const auto & p : pts) {
      if (bounds_.contains(p.x(), p.y(), p.z())) {
        pc.push_back(p);
      }
    }
    if (pc.size() == 0) {
      return 0;
    }
    tree_->insertPointCloud(pc, sensor_origin, max_range,
                            /*lazy_eval=*/true, /*discretize=*/true);
    tree_->updateInnerOccupancy();
    edt_dirty_ = true;
    return pc.size();
  }

  // Recompute the distance map. Incremental inside dynamicEDT3D; call after
  // a batch of insertions, not per cloud.
  void updateDistanceMap()
  {
    edt_->update(true);
    edt_dirty_ = false;
  }

  bool distanceMapDirty() const { return edt_dirty_; }

  Occ occupancy(double x, double y, double z) const
  {
    if (!bounds_.contains(x, y, z)) {
      return Occ::UNKNOWN;  // outside the mapped universe
    }
    const octomap::OcTreeNode * n = tree_->search(x, y, z);
    if (n == nullptr) {
      return Occ::UNKNOWN;
    }
    return tree_->isNodeOccupied(n) ? Occ::OCCUPIED : Occ::FREE;
  }

  // Metres to the nearest occupied voxel, clamped at clearance_cap.
  // Returns -1.0 outside the bounds (no answer, not "far").
  float clearance(double x, double y, double z) const
  {
    if (!bounds_.contains(x, y, z)) {
      return -1.0f;
    }
    float d = edt_->getDistance(octomap::point3d(x, y, z));
    if (d == DynamicEDTOctomap::distanceValue_Error) {
      return -1.0f;
    }
    return d;
  }

  std::vector<octomap::point3d> occupiedCells() const
  {
    std::vector<octomap::point3d> out;
    for (auto it = tree_->begin_leafs(); it != tree_->end_leafs(); ++it) {
      if (tree_->isNodeOccupied(*it)) {
        out.push_back(it.getCoordinate());
      }
    }
    return out;
  }

  octomap::OcTree & tree() { return *tree_; }
  double clearanceCap() const { return clearance_cap_; }
  const Bounds & bounds() const { return bounds_; }

private:
  Bounds bounds_;
  double clearance_cap_;
  std::unique_ptr<octomap::OcTree> tree_;
  std::unique_ptr<DynamicEDTOctomap> edt_;
  bool edt_dirty_{false};
};

}  // namespace map_interface
