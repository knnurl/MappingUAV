// filepath: src/map_interface/src/map_saver_node.cpp
// Saves the 3D map WHILE flying (operator request 2026-11-16). Subscribes to
// /cloud_registered (FAST-LIO world frame), keeps one point per voxel, and
// every checkpoint_period_s writes the voxels that are new since the last
// checkpoint as chunk_NNNNN.pcd in a per-session directory. Concatenating the
// chunks gives the whole map; scripts/map_sync.sh mirrors the directory off
// the vehicle. Raw clouds are still never recorded in flight: this is a
// voxel-thinned product, a few MB per room.
//
// Bounds: max_voxels (memory), min_free_gb (disk; no writes below it).
#include <chrono>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <memory>
#include <deque>
#include <string>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "map_interface/keyframes.hpp"
#include "map_interface/voxel_accumulator.hpp"

namespace fs = std::filesystem;

namespace map_interface
{

class MapSaverNode : public rclcpp::Node
{
public:
  MapSaverNode()
  : Node("map_saver")
  {
    const double voxel = declare_parameter<double>("voxel_size", 0.05);
    const int max_voxels = declare_parameter<int>("max_voxels", 4000000);
    const double period = declare_parameter<double>("checkpoint_period_s", 20.0);
    min_free_gb_ = declare_parameter<double>("min_free_gb", 4.0);
    std::string root = declare_parameter<std::string>("output_dir", "");
    const std::string topic =
      declare_parameter<std::string>("lio_cloud_topic", "/cloud_registered");
    if (root.empty()) {
      const char * home = std::getenv("HOME");
      root = std::string(home != nullptr ? home : "/tmp") + "/map_exports";
    }

    char stamp[32];
    const std::time_t t = std::time(nullptr);
    std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", std::localtime(&t));
    session_dir_ = fs::path(root) / (std::string("session_") + stamp);
    fs::create_directories(session_dir_);
    if (freeGb() < min_free_gb_) {
      RCLCPP_FATAL(get_logger(), "%.1f GB free under %s, below min_free_gb %.1f: not saving",
        freeGb(), root.c_str(), min_free_gb_);
      throw std::runtime_error("map_saver: not enough free disk");
    }

    // Keyframes for offline refinement: a body-frame scan and its pose every
    // keyframe_distance_m or keyframe_angle_deg (docs/later_upgrades_spec.md C-1).
    keyframes_ = declare_parameter<bool>("keyframes", true);
    keyframe_voxel_ = declare_parameter<double>("keyframe_voxel_m", 0.1);
    selector_ = std::make_unique<KeyframeSelector>(
      declare_parameter<double>("keyframe_distance_m", 0.5),
      declare_parameter<double>("keyframe_angle_deg", 15.0) * M_PI / 180.0,
      declare_parameter<int>("max_keyframes", 2000));
    const std::string odom_topic =
      declare_parameter<std::string>("lio_odom_topic", "/Odometry");
    if (keyframes_) {
      fs::create_directories(session_dir_ / "keyframes");
      odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, rclcpp::QoS(rclcpp::KeepLast(20)).best_effort(),
        [this](const nav_msgs::msg::Odometry::SharedPtr m) {
          Pose p;
          p.x = m->pose.pose.position.x; p.y = m->pose.pose.position.y;
          p.z = m->pose.pose.position.z;
          p.qx = m->pose.pose.orientation.x; p.qy = m->pose.pose.orientation.y;
          p.qz = m->pose.pose.orientation.z; p.qw = m->pose.pose.orientation.w;
          poses_.emplace_back(rclcpp::Time(m->header.stamp).seconds(), p);
          while (poses_.size() > 30) {poses_.pop_front();}
        });
    }

    acc_ = std::make_unique<VoxelAccumulator>(voxel, static_cast<size_t>(max_voxels));
    auto qos = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort();
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      topic, qos, std::bind(&MapSaverNode::onCloud, this, std::placeholders::_1));
    timer_ = create_wall_timer(
      std::chrono::duration<double>(period), std::bind(&MapSaverNode::checkpoint, this));
    RCLCPP_INFO(get_logger(),
      "saving %s at %.2f m voxels to %s every %.0f s (max %d voxels)",
      topic.c_str(), voxel, session_dir_.c_str(), period, max_voxels);
  }

  ~MapSaverNode() override
  {
    checkpoint();   // final chunk on a clean shutdown
  }

private:
  double freeGb() const
  {
    std::error_code ec;
    const auto info = fs::space(session_dir_, ec);
    return ec ? 0.0 : static_cast<double>(info.available) / (1024.0 * 1024.0 * 1024.0);
  }

  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    frame_id_ = msg->header.frame_id;
    sensor_msgs::PointCloud2ConstIterator<float> ix(*msg, "x"), iy(*msg, "y"), iz(*msg, "z");
    for (; ix != ix.end(); ++ix, ++iy, ++iz) {
      acc_->add(*ix, *iy, *iz);
    }
    ++clouds_;
    if (keyframes_) {
      maybeKeyframe(*msg);
    }
    if (acc_->full() && !full_logged_) {
      full_logged_ = true;
      RCLCPP_ERROR(get_logger(),
        "max_voxels reached: new areas are no longer added to the saved map");
    }
  }

  // The pose LIO registered this cloud with: the odometry sample of the same stamp.
  bool poseAt(double stamp, Pose & out) const
  {
    for (auto it = poses_.rbegin(); it != poses_.rend(); ++it) {
      if (std::fabs(it->first - stamp) <= 0.005) {
        out = it->second;
        return true;
      }
    }
    return false;
  }

  void maybeKeyframe(const sensor_msgs::msg::PointCloud2 & msg)
  {
    const double stamp = rclcpp::Time(msg.header.stamp).seconds();
    Pose pose;
    if (!poseAt(stamp, pose) || selector_->full() || freeGb() < min_free_gb_) {
      return;
    }
    if (!selector_->select(pose)) {
      return;
    }
    VoxelAccumulator thin(keyframe_voxel_, 200000);
    sensor_msgs::PointCloud2ConstIterator<float> ix(msg, "x"), iy(msg, "y"), iz(msg, "z");
    for (; ix != ix.end(); ++ix, ++iy, ++iz) {
      const auto b = worldToBody(pose, *ix, *iy, *iz);
      thin.add(static_cast<float>(b[0]), static_cast<float>(b[1]), static_cast<float>(b[2]));
    }
    char name[32];
    const int n = selector_->count() - 1;
    std::snprintf(name, sizeof(name), "kf_%05d.pcd", n);
    const long points = thin.flush((session_dir_ / "keyframes" / name).string());
    if (points <= 0) {
      return;
    }
    // One line per keyframe: index, stamp, position, orientation (x y z w), points.
    std::ofstream poses(session_dir_ / "keyframes" / "poses.txt", std::ios::app);
    char line[256];
    std::snprintf(line, sizeof(line), "%d %.6f %.6f %.6f %.6f %.9f %.9f %.9f %.9f %ld\n",
      n, stamp, pose.x, pose.y, pose.z, pose.qx, pose.qy, pose.qz, pose.qw, points);
    poses << line;
  }

  void checkpoint()
  {
    if (acc_ == nullptr || acc_->pendingPoints() == 0) {
      return;
    }
    if (freeGb() < min_free_gb_) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 30000,
        "%.1f GB free, below min_free_gb %.1f: checkpoint skipped", freeGb(), min_free_gb_);
      return;
    }
    char name[32];
    std::snprintf(name, sizeof(name), "chunk_%05d.pcd", chunk_);
    const fs::path path = session_dir_ / name;
    const long n = acc_->flush(path.string());
    if (n < 0) {
      RCLCPP_ERROR(get_logger(), "could not write %s; will retry", path.c_str());
      return;
    }
    ++chunk_;
    std::ofstream manifest(session_dir_ / "manifest.txt", std::ios::app);
    manifest << name << " points=" << n << " total_voxels=" << acc_->totalVoxels()
             << " clouds=" << clouds_ << " frame=" << frame_id_
             << " unix=" << std::time(nullptr) << "\n";
    RCLCPP_INFO(get_logger(), "checkpoint %s: %ld new points, %zu voxels in total",
      name, n, acc_->totalVoxels());
  }

  std::unique_ptr<VoxelAccumulator> acc_;
  std::unique_ptr<KeyframeSelector> selector_;
  std::deque<std::pair<double, Pose>> poses_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  bool keyframes_{true};
  double keyframe_voxel_{0.1};
  fs::path session_dir_;
  double min_free_gb_{4.0};
  int chunk_{0};
  size_t clouds_{0};
  bool full_logged_{false};
  std::string frame_id_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace map_interface

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<map_interface::MapSaverNode>());
  rclcpp::shutdown();
  return 0;
}
