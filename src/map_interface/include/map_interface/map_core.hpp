// filepath: src/map_interface/include/map_interface/map_core.hpp
// Pure mapping core (WP-C cpu_grid backend): octomap occupancy + dynamicEDT3D
// clearance, bounded to a world-frame box. No ROS types — unit-testable
// without a graph. Frames [FIXED]: everything here is odom/ENU; planners
// never consume NED (masterplan §1.2).
#pragma once

#include <algorithm>
#include <cmath>
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

  // Insert a world-frame cloud taken from sensor_origin. max_range < 0
  // disables the range cut. Returns the number of points inside the bounds
  // (the ones that can become occupied voxels).
  //
  // A point OUTSIDE the bounds is not discarded: the beam still crossed the
  // box on its way out, so the cells along the part of the ray inside the box
  // are updated as free. Nothing is marked occupied for it. Without this, the
  // space in front of a wall beyond the box, or under a ceiling above z_max,
  // stays unknown for ever and an explorer sees frontiers that never clear.
  // If the sensor itself is outside the box such points are skipped.
  size_t insertCloud(const std::vector<octomap::point3d> & pts,
                     const octomap::point3d & sensor_origin,
                     double max_range = -1.0)
  {
    const bool origin_inside =
      bounds_.contains(sensor_origin.x(), sensor_origin.y(), sensor_origin.z());
    octomap::Pointcloud pc;
    octomap::KeySet exit_keys;                 // one free ray per exit voxel
    std::vector<octomap::point3d> exits;
    for (const auto & p : pts) {
      if (bounds_.contains(p.x(), p.y(), p.z())) {
        pc.push_back(p);
      } else if (origin_inside) {
        octomap::point3d e;
        octomap::OcTreeKey k;
        if (clipToBounds(sensor_origin, p, max_range, e) &&
          tree_->coordToKeyChecked(e, k) && exit_keys.insert(k).second)
        {
          exits.push_back(e);
        }
      }
    }
    if (pc.size() == 0 && exits.empty()) {
      return 0;
    }
    if (pc.size() > 0) {
      tree_->insertPointCloud(pc, sensor_origin, max_range,
                              /*lazy_eval=*/true, /*discretize=*/true);
    }
    if (!exits.empty()) {
      // Occupied endpoints of this scan win over free rays of the same scan,
      // as inside octomap's own insertPointCloud.
      octomap::KeySet occupied;
      for (size_t i = 0; i < pc.size(); ++i) {
        octomap::OcTreeKey k;
        const octomap::point3d & p = pc[i];
        const bool in_range = max_range < 0.0 ||
          (p - sensor_origin).norm() <= max_range;
        if (in_range && tree_->coordToKeyChecked(p, k)) {
          occupied.insert(k);
        }
      }
      octomap::KeySet free_cells;
      octomap::KeyRay ray;
      for (const auto & e : exits) {
        if (tree_->computeRayKeys(sensor_origin, e, ray)) {
          free_cells.insert(ray.begin(), ray.end());
        }
        octomap::OcTreeKey k;
        if (tree_->coordToKeyChecked(e, k)) {
          free_cells.insert(k);                // computeRayKeys excludes the end
        }
      }
      for (const auto & k : free_cells) {
        if (occupied.find(k) == occupied.end()) {
          tree_->updateNode(k, false, /*lazy_eval=*/true);
        }
      }
    }
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
  // Last point of the segment origin->p that is still inside the bounds
  // (origin must be inside), pulled 1 mm back from the face and shortened to
  // max_range when that is set. False if the segment has no usable length.
  bool clipToBounds(const octomap::point3d & o, const octomap::point3d & p,
                    double max_range, octomap::point3d & out) const
  {
    const double d[3] = {p.x() - o.x(), p.y() - o.y(), p.z() - o.z()};
    const double oo[3] = {o.x(), o.y(), o.z()};
    const double lo[3] = {bounds_.x_min, bounds_.y_min, bounds_.z_min};
    const double hi[3] = {bounds_.x_max, bounds_.y_max, bounds_.z_max};
    double t = 1.0;
    for (int i = 0; i < 3; ++i) {
      if (d[i] > 0.0) {
        t = std::min(t, (hi[i] - oo[i]) / d[i]);
      } else if (d[i] < 0.0) {
        t = std::min(t, (lo[i] - oo[i]) / d[i]);
      }
    }
    const double len = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
    if (!(len > 0.0) || !std::isfinite(len)) {
      return false;
    }
    double dist = t * len - 1e-3;
    if (max_range >= 0.0) {
      dist = std::min(dist, max_range);
    }
    if (dist <= 0.0) {
      return false;
    }
    const double s = dist / len;
    out = octomap::point3d(
      static_cast<float>(oo[0] + d[0] * s),
      static_cast<float>(oo[1] + d[1] * s),
      static_cast<float>(oo[2] + d[2] * s));
    return bounds_.contains(out.x(), out.y(), out.z());
  }

  Bounds bounds_;
  double clearance_cap_;
  std::unique_ptr<octomap::OcTree> tree_;
  std::unique_ptr<DynamicEDTOctomap> edt_;
  bool edt_dirty_{false};
};

}  // namespace map_interface
