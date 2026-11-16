// filepath: src/map_interface/src/map_server_node.cpp
// WP-C map server: /cloud_registered -> bounded coarse occupancy (cpu_grid
// backend = octomap + dynamicEDT3D per masterplan §5, Bonxai verified
// immature 2026-07-04). Backend selection is a wrapper contract:
//   map_backend:=cpu_grid  (implemented)
//   map_backend:=nvblox    (refused until decision_dp1_mapping = NVBLOX;
//                           see docs/dp1_evidence.md)
// Sensor origin comes from /Odometry (FAST-LIO pose), not TF: the perception
// pipeline already publishes the pose at cloud rate and this node must not
// stall on TF availability.
#include <algorithm>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>

#include "map_interface/map_core.hpp"
#include "map_interface/srv/query_map.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

namespace map_interface
{

class MapServerNode : public rclcpp::Node
{
public:
  MapServerNode()
  : Node("map_interface")
  {
    const std::string backend =
      declare_parameter<std::string>("map_backend", "cpu_grid");
    if (backend != "cpu_grid") {
      RCLCPP_FATAL(get_logger(),
        "map_backend '%s' is not available: decision_dp1_mapping is "
        "UNDECIDED (see docs/dp1_evidence.md). Only cpu_grid is implemented.",
        backend.c_str());
      throw std::runtime_error("unimplemented map backend: " + backend);
    }

    Bounds b;
    b.x_min = declare_parameter<double>("x_min", -10.0);
    b.x_max = declare_parameter<double>("x_max", 10.0);
    b.y_min = declare_parameter<double>("y_min", -10.0);
    b.y_max = declare_parameter<double>("y_max", 10.0);
    b.z_min = declare_parameter<double>("z_min", -0.5);
    b.z_max = declare_parameter<double>("z_max", 3.0);
    const double resolution = declare_parameter<double>("resolution", 0.2);
    const double clearance_cap = declare_parameter<double>("clearance_cap", 4.0);
    const bool unknown_occ = declare_parameter<bool>("unknown_as_occupied", false);
    insert_period_s_ = declare_parameter<double>("insert_period_s", 0.5);
    max_insert_range_ = declare_parameter<double>("max_insert_range", 8.0);
    const double edt_period = declare_parameter<double>("edt_period_s", 2.0);
    const double publish_period = declare_parameter<double>("map_publish_period_s", 2.0);

    core_ = std::make_unique<MapCore>(resolution, b, clearance_cap, unknown_occ);

    auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(2)).best_effort();
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/cloud_registered", sensor_qos,
      std::bind(&MapServerNode::onCloud, this, _1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/Odometry", sensor_qos,
      std::bind(&MapServerNode::onOdom, this, _1));

    auto latched = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
    cells_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/map_interface/occupied_cells", latched);
    octomap_pub_ = create_publisher<octomap_msgs::msg::Octomap>(
      "/map_interface/octomap", latched);

    query_srv_ = create_service<srv::QueryMap>(
      "/map_interface/query",
      std::bind(&MapServerNode::onQuery, this, _1, _2));

    edt_timer_ = create_wall_timer(
      std::chrono::duration<double>(edt_period),
      std::bind(&MapServerNode::onEdtTimer, this));
    publish_timer_ = create_wall_timer(
      std::chrono::duration<double>(publish_period),
      std::bind(&MapServerNode::publishMap, this));

    RCLCPP_INFO(get_logger(),
      "cpu_grid map: res %.2f m, bounds x[%.1f,%.1f] y[%.1f,%.1f] "
      "z[%.1f,%.1f] (odom/ENU), insert every %.1f s, range cut %.1f m",
      resolution, b.x_min, b.x_max, b.y_min, b.y_max, b.z_min, b.z_max,
      insert_period_s_, max_insert_range_);
  }

private:
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    origin_ = octomap::point3d(
      msg->pose.pose.position.x,
      msg->pose.pose.position.y,
      msg->pose.pose.position.z);
    have_origin_ = true;
  }

  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (!have_origin_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000,
        "cloud received but no /Odometry yet; skipping insertion");
      return;
    }
    // Rate-limit insertions: 10 Hz clouds at full raycast would blow the
    // 2.0-core budget. The map is coarse; 2 Hz insertion is plenty.
    const rclcpp::Time stamp = now();
    if (last_insert_.nanoseconds() != 0 &&
        (stamp - last_insert_).seconds() < insert_period_s_)
    {
      return;
    }
    last_insert_ = stamp;

    std::vector<octomap::point3d> pts;
    pts.reserve(msg->width * msg->height);
    sensor_msgs::PointCloud2ConstIterator<float> ix(*msg, "x"), iy(*msg, "y"),
      iz(*msg, "z");
    for (; ix != ix.end(); ++ix, ++iy, ++iz) {
      if (std::isfinite(*ix) && std::isfinite(*iy) && std::isfinite(*iz)) {
        pts.emplace_back(*ix, *iy, *iz);
      }
    }
    const size_t inserted = core_->insertCloud(pts, origin_, max_insert_range_);
    RCLCPP_DEBUG(get_logger(), "inserted %zu/%zu points", inserted, pts.size());
  }

  void onEdtTimer()
  {
    if (core_->distanceMapDirty()) {
      core_->updateDistanceMap();
    }
  }

  void onQuery(const srv::QueryMap::Request::SharedPtr req,
               srv::QueryMap::Response::SharedPtr res)
  {
    if (core_->distanceMapDirty()) {
      core_->updateDistanceMap();
    }
    res->occupancy.reserve(req->points.size());
    res->clearance.reserve(req->points.size());
    for (const auto & p : req->points) {
      res->occupancy.push_back(static_cast<uint8_t>(
        core_->occupancy(p.x, p.y, p.z)));
      res->clearance.push_back(core_->clearance(p.x, p.y, p.z));
    }
  }

  void publishMap()
  {
    const auto cells = core_->occupiedCells();

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = now();
    cloud.header.frame_id = "odom";
    sensor_msgs::PointCloud2Modifier mod(cloud);
    mod.setPointCloud2FieldsByString(1, "xyz");
    mod.resize(cells.size());
    sensor_msgs::PointCloud2Iterator<float> ox(cloud, "x"), oy(cloud, "y"),
      oz(cloud, "z");
    for (const auto & c : cells) {
      *ox = c.x(); *oy = c.y(); *oz = c.z();
      ++ox; ++oy; ++oz;
    }
    cells_pub_->publish(cloud);

    octomap_msgs::msg::Octomap om;
    om.header = cloud.header;
    if (octomap_msgs::binaryMapToMsg(core_->tree(), om)) {
      octomap_pub_->publish(om);
    }
  }

  std::unique_ptr<MapCore> core_;
  octomap::point3d origin_{0.0f, 0.0f, 0.0f};
  bool have_origin_{false};
  double insert_period_s_{0.5};
  double max_insert_range_{8.0};
  rclcpp::Time last_insert_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cells_pub_;
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_pub_;
  rclcpp::Service<srv::QueryMap>::SharedPtr query_srv_;
  rclcpp::TimerBase::SharedPtr edt_timer_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

}  // namespace map_interface

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<map_interface::MapServerNode>());
  rclcpp::shutdown();
  return 0;
}
