// door_pass_node: ROS 2 front-end of pgr_door (C core). Takes a differential-drive robot through a narrow door
// in one go: the door is located in the 2D scan and tracked in the odometry frame, the PGR law runs on the robot
// pose relative to the door and publishes cmd_vel. License: Apache-2.0
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

extern "C" {
#include "pgr_door.h"
}

using namespace std::chrono_literals;

namespace {
pgrd_pose make_pose(double x, double y, double th) { return pgrd_pose{x, y, th}; }

double yaw_of(const geometry_msgs::msg::Quaternion &q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

pgrd_pose tf_to_pose(const geometry_msgs::msg::TransformStamped &t) {
  return make_pose(t.transform.translation.x, t.transform.translation.y, yaw_of(t.transform.rotation));
}

geometry_msgs::msg::PoseStamped to_msg(const pgrd_pose &p, const std::string &frame, const rclcpp::Time &stamp) {
  geometry_msgs::msg::PoseStamped m;
  m.header.frame_id = frame;
  m.header.stamp = stamp;
  m.pose.position.x = p.x;
  m.pose.position.y = p.y;
  m.pose.orientation.z = std::sin(p.th / 2.0);
  m.pose.orientation.w = std::cos(p.th / 2.0);
  return m;
}
}  // namespace

class DoorPassNode : public rclcpp::Node {
 public:
  DoorPassNode() : Node("door_pass") {
    base_frame_ = declare_parameter("base_frame", "base_link");
    odom_frame_ = declare_parameter("odom_frame", "odom");
    map_frame_ = declare_parameter("map_frame", "map");
    door_map_ = make_pose(declare_parameter("door.x", 0.0), declare_parameter("door.y", 0.0),
                          declare_parameter("door.yaw", 0.0));
    use_detection_ = declare_parameter("use_detection", true);
    autostart_ = declare_parameter("autostart", false);
    double rate = declare_parameter("rate", 10.0);

    pgrd_default_params(&p_);
    p_.xmin = declare_parameter("footprint.xmin", p_.xmin);
    p_.xmax = declare_parameter("footprint.xmax", p_.xmax);
    p_.hw = declare_parameter("footprint.half_width", p_.hw);
    p_.wall_t = declare_parameter("door.wall_thickness", p_.wall_t);
    p_.n = declare_parameter("pgr.n", p_.n);
    p_.k = declare_parameter("pgr.k", p_.k);
    p_.v_far = declare_parameter("v_far", p_.v_far);
    p_.v_near = declare_parameter("v_near", p_.v_near);
    p_.w_max = declare_parameter("w_max", p_.w_max);
    p_.pre = declare_parameter("pgr.pre", p_.pre);
    p_.exit_dist = declare_parameter("exit_distance", p_.exit_dist);
    p_.goal_tol = declare_parameter("goal_tolerance", p_.goal_tol);
    p_.dt = 1.0 / rate;
    pgrd_detect_default(&dp_);
    dp_.wall_t = p_.wall_t;
    dp_.wmin = declare_parameter("detect.min_width", dp_.wmin);
    dp_.wmax = declare_parameter("detect.max_width", dp_.wmax);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(declare_parameter("cmd_vel_topic", "cmd_vel"), 10);
    state_pub_ = create_publisher<std_msgs::msg::String>("~/state", 10);
    door_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("~/door_pose", 10);
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        declare_parameter("scan_topic", "scan"), rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::LaserScan::SharedPtr m) { on_scan(m); });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        declare_parameter("odom_topic", "odom"), 10,
        [this](nav_msgs::msg::Odometry::SharedPtr m) { v_meas_ = m->twist.twist.linear.x; });
    door_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "~/door", 10, [this](geometry_msgs::msg::PoseStamped::SharedPtr m) {   // set the door (map frame) and go
          door_map_ = make_pose(m->pose.position.x, m->pose.position.y, yaw_of(m->pose.orientation));
          start();
        });
    start_srv_ = create_service<std_srvs::srv::Trigger>(
        "~/start", [this](std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr r) {
          start();
          r->success = true;
        });
    stop_srv_ = create_service<std_srvs::srv::Trigger>(
        "~/stop", [this](std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr r) {
          active_ = false;
          cmd_pub_->publish(geometry_msgs::msg::Twist());
          r->success = true;
        });
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate), [this] { step(); });
    if (autostart_) start();
    RCLCPP_INFO(get_logger(), "door_pass ready: pre-door well %.2f m (%s)", p_.pre > 0 ? p_.pre : pgrd_derived_pre(&p_),
                p_.pre > 0 ? "set" : "derived from the footprint");
  }

 private:
  void start() {
    pgrd_ctrl_init(&ctrl_, &p_);
    pgrd_tracker_reset(&tracker_);
    oriented_ = false;
    active_ = true;
  }

  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr &m) {
    geometry_msgs::msg::TransformStamped t;
    try {
      t = tf_buffer_->lookupTransform(base_frame_, m->header.frame_id, tf2::TimePointZero);
    } catch (const tf2::TransformException &) {
      return;
    }
    const pgrd_pose s = tf_to_pose(t);                           // sensor in the robot frame
    std::lock_guard<std::mutex> lk(scan_mtx_);
    pts_.clear();
    ang_.clear();
    rng_.clear();
    for (size_t i = 0; i < m->ranges.size(); ++i) {
      const double a = m->angle_min + m->angle_increment * static_cast<double>(i), r = m->ranges[i];
      const bool ok = std::isfinite(r) && r > m->range_min && r < m->range_max - 1e-3;
      ang_.push_back(pgrd_wrap(a + s.th));                        // beam direction in the robot frame
      rng_.push_back(ok ? r : INFINITY);
      if (!ok) continue;
      const pgrd_pose q = pgrd_compose(s, make_pose(r * std::cos(a), r * std::sin(a), 0.0));
      pts_.push_back(q.x);
      pts_.push_back(q.y);
    }
    sensor_ = s;
    scan_stamp_ = now();
  }

  void step() {
    if (!active_) return;
    pgrd_pose robot_odom, map_odom = make_pose(0, 0, 0);
    try {
      robot_odom = tf_to_pose(tf_buffer_->lookupTransform(odom_frame_, base_frame_, tf2::TimePointZero));
      if (map_frame_ != odom_frame_)
        map_odom = tf_to_pose(tf_buffer_->lookupTransform(map_frame_, odom_frame_, tf2::TimePointZero));
    } catch (const tf2::TransformException &e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "waiting for TF: %s", e.what());
      return;
    }
    if (!oriented_) {                                             // door frame x must point away from the robot side
      const pgrd_pose door_odom0 = pgrd_compose(pgrd_inverse(map_odom), door_map_);
      if (pgrd_compose(pgrd_inverse(door_odom0), robot_odom).x > 0) door_map_.th = pgrd_wrap(door_map_.th + M_PI);
      oriented_ = true;
    }
    pgrd_pose door_odom = tracker_.n_det > 0 ? tracker_.door_odom : pgrd_compose(pgrd_inverse(map_odom), door_map_);
    if (use_detection_) {
      const pgrd_pose door_rel = pgrd_compose(pgrd_inverse(robot_odom), door_odom);
      std::lock_guard<std::mutex> lk(scan_mtx_);
      const bool fresh = (now() - scan_stamp_).seconds() < 0.3;
      if (fresh && std::hypot(door_rel.x, door_rel.y) < 4.0 && door_rel.x > 0.25 && !pts_.empty()) {
        pgrd_pose det;
        double width;
        if (pgrd_detect(&dp_, pts_.data(), static_cast<int>(pts_.size() / 2), door_rel, ang_.data(), rng_.data(),
                        static_cast<int>(ang_.size()), sensor_.x, sensor_.y, &seed_, &det, &width)) {
          pgrd_tracker_add(&tracker_, robot_odom, det);
          door_odom = tracker_.door_odom;
        }
      }
    }
    const pgrd_pose rel = pgrd_compose(pgrd_inverse(door_odom), robot_odom);
    double v, w;
    const int st = pgrd_ctrl_step(&ctrl_, rel, v_meas_, &v, &w);
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = v;
    cmd.angular.z = w;
    cmd_pub_->publish(cmd);
    door_pub_->publish(to_msg(door_odom, odom_frame_, now()));
    std_msgs::msg::String s;
    s.data = st == PGRD_APPROACH ? "approach" : (st == PGRD_EXIT ? "crossing" : "done");
    s.data += " detections=" + std::to_string(tracker_.n_det);
    state_pub_->publish(s);
    if (st == PGRD_DONE) {
      active_ = false;
      RCLCPP_INFO(get_logger(), "door passed");
    }
  }

  std::string base_frame_, odom_frame_, map_frame_;
  pgrd_pose door_map_{}, sensor_{};
  bool use_detection_{true}, autostart_{false}, active_{false}, oriented_{false};
  pgrd_params p_{};
  pgrd_detect_params dp_{};
  pgrd_ctrl ctrl_{};
  pgrd_tracker tracker_{};
  unsigned seed_{12345u};
  double v_meas_{0.0};
  std::mutex scan_mtx_;
  std::vector<double> pts_, ang_, rng_;
  rclcpp::Time scan_stamp_{0, 0, RCL_ROS_TIME};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr door_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr door_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_, stop_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DoorPassNode>());
  rclcpp::shutdown();
  return 0;
}
