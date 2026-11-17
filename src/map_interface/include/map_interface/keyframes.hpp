// filepath: src/map_interface/include/map_interface/keyframes.hpp
// Keyframes for offline map refinement (docs/later_upgrades_spec.md, U-C stage
// C-1). A keyframe is one scan in the BODY frame plus the odometry pose it was
// taken at, so a refined pose can re-place it later. No ROS types.
//
// The scan arrives in the world frame (that is what LIO publishes); it is
// brought back to the body frame with the inverse of the pose of the same
// stamp, which is exact for the pose LIO used to register it.
#pragma once

#include <array>
#include <cmath>

namespace map_interface
{

struct Pose
{
  double x{0}, y{0}, z{0};
  double qx{0}, qy{0}, qz{0}, qw{1};
};

// v rotated by the inverse of unit quaternion q.
inline std::array<double, 3> rotateInverse(const Pose & q, double vx, double vy, double vz)
{
  // inverse rotation = rotation by the conjugate (-qx, -qy, -qz, qw)
  const double x = -q.qx, y = -q.qy, z = -q.qz, w = q.qw;
  const double tx = 2.0 * (y * vz - z * vy);
  const double ty = 2.0 * (z * vx - x * vz);
  const double tz = 2.0 * (x * vy - y * vx);
  return {vx + w * tx + (y * tz - z * ty),
          vy + w * ty + (z * tx - x * tz),
          vz + w * tz + (x * ty - y * tx)};
}

inline std::array<double, 3> worldToBody(const Pose & pose, double wx, double wy, double wz)
{
  return rotateInverse(pose, wx - pose.x, wy - pose.y, wz - pose.z);
}

// Angle in radians between the orientations of two poses.
inline double angleBetween(const Pose & a, const Pose & b)
{
  double dot = std::fabs(a.qx * b.qx + a.qy * b.qy + a.qz * b.qz + a.qw * b.qw);
  if (dot > 1.0) {dot = 1.0;}
  return 2.0 * std::acos(dot);
}

// Decides which scans become keyframes: the first one, then whenever the
// vehicle has moved distance_m or turned angle_rad since the last keyframe,
// up to max_keyframes.
class KeyframeSelector
{
public:
  KeyframeSelector(double distance_m, double angle_rad, int max_keyframes)
  : distance_m_(distance_m), angle_rad_(angle_rad), max_(max_keyframes) {}

  bool select(const Pose & pose)
  {
    if (count_ >= max_) {
      return false;
    }
    if (count_ > 0) {
      const double d = std::sqrt(
        (pose.x - last_.x) * (pose.x - last_.x) + (pose.y - last_.y) * (pose.y - last_.y) +
        (pose.z - last_.z) * (pose.z - last_.z));
      if (d < distance_m_ && angleBetween(pose, last_) < angle_rad_) {
        return false;
      }
    }
    last_ = pose;
    ++count_;
    return true;
  }

  int count() const {return count_;}
  bool full() const {return count_ >= max_;}

private:
  double distance_m_, angle_rad_;
  int max_;
  int count_{0};
  Pose last_;
};

}  // namespace map_interface
