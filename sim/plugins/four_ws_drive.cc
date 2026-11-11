// SPDX-License-Identifier: Apache-2.0
// four_ws_drive: Gazebo-classic model plugin for a 4D4T base (four driven wheels, four independently steered).
// cmd_vel (geometry_msgs/Twist: linear.x, linear.y, angular.z in the body frame) -> per wheel module the velocity of
// its contact point v_i = (vx - wz * y_i, vy + wz * x_i): steering angle atan2 (flipped by pi beyond +-90 deg) and
// wheel speed |v_i| / r. Steering joints follow position targets and wheels velocity targets through Gazebo's
// JointController PIDs (forces on the joints; the motion of the base comes out of the contact physics).
// Publishes odom (nav_msgs/Odometry) and the odom -> base TF from the body pose (ideal odometry for the demo).
// License: Apache-2.0
#include <array>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>

#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace gazebo {
class FourWsDrive : public ModelPlugin {
 public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override {
    model_ = model;
    ros_ = gazebo_ros::Node::Get(sdf);
    const char *names[4] = {"fl", "fr", "rl", "rr"};
    for (int i = 0; i < 4; ++i) {
      steer_[i] = model->GetJoint(sdf->Get<std::string>(std::string("steer_") + names[i]));
      wheel_[i] = model->GetJoint(sdf->Get<std::string>(std::string("wheel_") + names[i]));
      if (!steer_[i] || !wheel_[i]) { gzerr << "four_ws_drive: missing joint for " << names[i] << "\n"; return; }
      const auto p = steer_[i]->GetChild()->RelativePose().Pos() - model->GetLink(sdf->Get<std::string>("base_link"))
                                                                          ->RelativePose().Pos();
      px_[i] = p.X(); py_[i] = p.Y();
    }
    radius_ = sdf->Get<double>("wheel_radius");
    base_ = model->GetLink(sdf->Get<std::string>("base_link"));
    odom_frame_ = sdf->Get<std::string>("odom_frame");
    base_frame_ = sdf->Get<std::string>("base_frame");
    auto jc = model->GetJointController();
    for (int i = 0; i < 4; ++i) {
      jc->SetPositionPID(steer_[i]->GetScopedName(), common::PID(60.0, 0.0, 2.0, 0, 0, 40.0, -40.0));
      jc->SetVelocityPID(wheel_[i]->GetScopedName(), common::PID(30.0, 5.0, 0.0, 10.0, -10.0, 40.0, -40.0));
      jc->SetPositionTarget(steer_[i]->GetScopedName(), 0.0);
      jc->SetVelocityTarget(wheel_[i]->GetScopedName(), 0.0);
    }
    sub_ = ros_->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, [this](geometry_msgs::msg::Twist::SharedPtr m) {
          std::lock_guard<std::mutex> lk(mtx_);
          vx_ = m->linear.x; vy_ = m->linear.y; wz_ = m->angular.z;
        });
    odom_pub_ = ros_->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    tfb_ = std::make_shared<tf2_ros::TransformBroadcaster>(ros_);
    conn_ = event::Events::ConnectWorldUpdateBegin(std::bind(&FourWsDrive::OnUpdate, this));
    gzmsg << "four_ws_drive: wheel radius " << radius_ << "\n";
  }

 private:
  void OnUpdate() {
    double vx, vy, wz;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      vx = vx_; vy = vy_; wz = wz_;
    }
    auto jc = model_->GetJointController();
    for (int i = 0; i < 4; ++i) {
      const double ux = vx - wz * py_[i], uy = vy + wz * px_[i];
      double speed = std::hypot(ux, uy), ang = speed > 1e-4 ? std::atan2(uy, ux) : last_ang_[i];
      if (ang > M_PI / 2) { ang -= M_PI; speed = -speed; }
      if (ang < -M_PI / 2) { ang += M_PI; speed = -speed; }
      last_ang_[i] = ang;
      jc->SetPositionTarget(steer_[i]->GetScopedName(), ang);
      jc->SetVelocityTarget(wheel_[i]->GetScopedName(), speed / radius_);
    }
    jc->Update();
    const common::Time now = model_->GetWorld()->SimTime();
    if ((now - last_pub_).Double() < 0.02) return;
    last_pub_ = now;
    const auto pose = base_->WorldPose();
    const auto lin = base_->RelativeLinearVel();
    const auto ang = base_->WorldAngularVel();
    rclcpp::Time stamp(now.sec, now.nsec, RCL_ROS_TIME);
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = stamp;
    t.header.frame_id = odom_frame_;
    t.child_frame_id = base_frame_;
    t.transform.translation.x = pose.Pos().X();
    t.transform.translation.y = pose.Pos().Y();
    t.transform.rotation.x = pose.Rot().X(); t.transform.rotation.y = pose.Rot().Y();
    t.transform.rotation.z = pose.Rot().Z(); t.transform.rotation.w = pose.Rot().W();
    tfb_->sendTransform(t);
    nav_msgs::msg::Odometry o;
    o.header = t.header;
    o.child_frame_id = base_frame_;
    o.pose.pose.position.x = pose.Pos().X(); o.pose.pose.position.y = pose.Pos().Y();
    o.pose.pose.orientation = t.transform.rotation;
    o.twist.twist.linear.x = lin.X(); o.twist.twist.linear.y = lin.Y(); o.twist.twist.angular.z = ang.Z();
    odom_pub_->publish(o);
  }

  physics::ModelPtr model_;
  physics::LinkPtr base_;
  std::array<physics::JointPtr, 4> steer_, wheel_;
  std::array<double, 4> px_{}, py_{}, last_ang_{};
  double radius_{0.1}, vx_{0}, vy_{0}, wz_{0};
  std::string odom_frame_, base_frame_;
  std::mutex mtx_;
  gazebo_ros::Node::SharedPtr ros_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tfb_;
  event::ConnectionPtr conn_;
  common::Time last_pub_;
};
GZ_REGISTER_MODEL_PLUGIN(FourWsDrive)
}  // namespace gazebo
