// Copyright 2020 F1TENTH Foundation
//
// Redistribution and use in source and binary forms, with or without modification, are permitted
// provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this list of conditions
//    and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice, this list
//    of conditions and the following disclaimer in the documentation and/or other materials
//    provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its contributors may be used
//    to endorse or promote products derived from this software without specific prior
//    written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
// IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
// FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
// WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

// -*- mode:c++; fill-column: 100; -*-

#include "vesc_ackermann/vesc_to_odom.h"

#include <cmath>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

namespace vesc_ackermann
{

template <typename T>
bool getRequiredParam(const std::shared_ptr<rclcpp::Node>& node, const std::string& name, T* value);

VescToOdom::VescToOdom(std::shared_ptr<rclcpp::Node> node) :
  node_(node), odom_frame_("odom"), base_frame_("base_link"),
  use_servo_cmd_(true), publish_tf_(false), x_(0.0), y_(0.0), yaw_(0.0)
{
  // get ROS parameters
  node_->declare_parameter("odom_frame", odom_frame_);
  node_->declare_parameter("base_frame", base_frame_);
  node_->declare_parameter("use_servo_cmd_to_calc_angular_velocity", use_servo_cmd_);
  node_->declare_parameter("publish_tf", publish_tf_);
  
  if (!getRequiredParam(node_, "speed_to_erpm_gain", &speed_to_erpm_gain_))
    return;
  if (!getRequiredParam(node_, "speed_to_erpm_offset", &speed_to_erpm_offset_))
    return;
  if (use_servo_cmd_)
  {
    if (!getRequiredParam(node_, "steering_angle_to_servo_gain", &steering_to_servo_gain_))
      return;
    if (!getRequiredParam(node_, "steering_angle_to_servo_offset", &steering_to_servo_offset_))
      return;
    if (!getRequiredParam(node_, "wheelbase", &wheelbase_))
      return;
  }

  // create odom publisher
  odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

  // create tf broadcaster
  if (publish_tf_)
  {
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
  }

  // subscribe to vesc state and optionally, servo command
  vesc_state_sub_ = node_->create_subscription<vesc_msgs::msg::VescStateStamped>(
      "sensors/core", 10, std::bind(&VescToOdom::vescStateCallback, this, std::placeholders::_1));
  if (use_servo_cmd_)
  {
    servo_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
        "sensors/servo_position_command", 10, std::bind(&VescToOdom::servoCmdCallback, this, std::placeholders::_1));
  }
}

void VescToOdom::vescStateCallback(const vesc_msgs::msg::VescStateStamped::SharedPtr state)
{
  // check that we have a last servo command if we are depending on it for angular velocity
  if (use_servo_cmd_ && !last_servo_cmd_)
    return;

  // convert to engineering units
  double current_speed = (-state->state.speed - speed_to_erpm_offset_) / speed_to_erpm_gain_;
  if (std::fabs(current_speed) < 0.05)
  {
    current_speed = 0.0;
  }
  double current_steering_angle(0.0), current_angular_velocity(0.0);
  if (use_servo_cmd_)
  {
    current_steering_angle =
      (last_servo_cmd_->data - steering_to_servo_offset_) / steering_to_servo_gain_;
    current_angular_velocity = current_speed * tan(current_steering_angle) / wheelbase_;
  }

  // use current state as last state if this is our first time here
  if (!last_state_)
    last_state_ = state;

  // calc elapsed time
  rclcpp::Duration dt = state->header.stamp - last_state_->header.stamp;

  // propagate odometry
  double x_dot = current_speed * cos(yaw_);
  double y_dot = current_speed * sin(yaw_);
  x_ += x_dot * dt.seconds();
  y_ += y_dot * dt.seconds();
  if (use_servo_cmd_)
    yaw_ += current_angular_velocity * dt.seconds();

  // save state for next time
  last_state_ = state;

  // publish odometry message
  auto odom = std::make_shared<nav_msgs::msg::Odometry>();
  odom->header.frame_id = odom_frame_;
  odom->header.stamp = state->header.stamp;
  odom->child_frame_id = base_frame_;

  // Position
  odom->pose.pose.position.x = x_;
  odom->pose.pose.position.y = y_;
  odom->pose.pose.orientation.z = sin(yaw_ / 2.0);
  odom->pose.pose.orientation.w = cos(yaw_ / 2.0);

  // Position uncertainty
  odom->pose.covariance[0]  = 0.2;  ///< x
  odom->pose.covariance[7]  = 0.2;  ///< y
  odom->pose.covariance[35] = 0.4;  ///< yaw

  // Velocity
  odom->twist.twist.linear.x = current_speed;
  odom->twist.twist.angular.z = current_angular_velocity;

  if (publish_tf_)
  {
    geometry_msgs::msg::TransformStamped tf;
    tf.header.frame_id = odom_frame_;
    tf.child_frame_id = base_frame_;
    tf.header.stamp = node_->get_clock()->now();
    tf.transform.translation.x = x_;
    tf.transform.translation.y = y_;
    tf.transform.rotation = odom->pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf);
  }

  odom_pub_->publish(*odom);
}

void VescToOdom::servoCmdCallback(const std_msgs::msg::Float64::SharedPtr servo)
{
  last_servo_cmd_ = servo;
}

template <typename T>
bool getRequiredParam(const std::shared_ptr<rclcpp::Node>& node, const std::string& name, T* value)
{
  if (node->get_parameter(name, *value))
    return true;

  RCLCPP_FATAL(node->get_logger(), "VescToOdom: Parameter %s is required.", name.c_str());
  return false;
}

}  // namespace vesc_ackermann
