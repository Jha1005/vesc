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

#include "vesc_ackermann/ackermann_to_vesc.h"
#include <cmath>
#include <sstream>
#include <string>

namespace vesc_ackermann
{

AckermannToVesc::AckermannToVesc() : Node("ackermann_to_vesc")
{
  declareAndGetParameters();

  // Create publishers to VESC electric-RPM (speed), current, brake, and servo commands
  erpm_pub_ = this->create_publisher<std_msgs::msg::Float64>("commands/motor/speed", 10);
  current_pub_ = this->create_publisher<std_msgs::msg::Float64>("commands/motor/current", 10);
  brake_pub_ = this->create_publisher<std_msgs::msg::Float64>("commands/motor/brake", 10);
  servo_pub_ = this->create_publisher<std_msgs::msg::Float64>("commands/servo/position", 10);

  // Subscribe to ackermann_cmd topic
  ackermann_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
    "ackermann_cmd", 10, std::bind(&AckermannToVesc::ackermannCmdCallback, this, std::placeholders::_1));
}

void AckermannToVesc::declareAndGetParameters()
{
  // Declare parameters
  this->declare_parameter("speed_to_erpm_gain", 4.5);
  this->declare_parameter("speed_to_erpm_offset", 0.0);
  this->declare_parameter("accel_to_current_gain", 12.0);
  this->declare_parameter("accel_to_brake_gain", 12.0);
  this->declare_parameter("steering_angle_to_servo_gain", 0.5);
  this->declare_parameter("steering_angle_to_servo_offset", 0.5);

  // Get parameter values
  this->get_parameter("speed_to_erpm_gain", speed_to_erpm_gain_);
  this->get_parameter("speed_to_erpm_offset", speed_to_erpm_offset_);
  this->get_parameter("accel_to_current_gain", accel_to_current_gain_);
  this->get_parameter("accel_to_brake_gain", accel_to_brake_gain_);
  this->get_parameter("steering_angle_to_servo_gain", steering_to_servo_gain_);
  this->get_parameter("steering_angle_to_servo_offset", steering_to_servo_offset_);
}

void AckermannToVesc::ackermannCmdCallback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr cmd)
{
  // Calculate VESC electric RPM (speed)
  auto erpm_msg = std::make_shared<std_msgs::msg::Float64>();
  erpm_msg->data = speed_to_erpm_gain_ * cmd->drive.speed + speed_to_erpm_offset_;

  // Calculate VESC current/brake (acceleration)
  bool is_positive_accel = true;
  auto current_msg = std::make_shared<std_msgs::msg::Float64>();
  auto brake_msg = std::make_shared<std_msgs::msg::Float64>();
  current_msg->data = 0;
  brake_msg->data = 0;
  if (cmd->drive.acceleration < 0)
  {
    brake_msg->data = accel_to_brake_gain_ * cmd->drive.acceleration;
    is_positive_accel = false;
  }
  else
  {
    current_msg->data = accel_to_current_gain_ * cmd->drive.acceleration;
  }

  // Calculate steering angle (servo)
  auto servo_msg = std::make_shared<std_msgs::msg::Float64>();
  servo_msg->data = steering_to_servo_gain_ * cmd->drive.steering_angle + steering_to_servo_offset_;

  // Publish the commands
  if (erpm_msg->data != 0 || previous_mode_speed_)
  {
    erpm_pub_->publish(*erpm_msg);
  }
  else
  {
    if (is_positive_accel)
    {
      current_pub_->publish(*current_msg);
    }
    else
    {
      brake_pub_->publish(*brake_msg);
    }
  }
  servo_pub_->publish(*servo_msg);

  // Determine the current mode to maintain until a new message forces a switch
  if (erpm_msg->data != 0)
  {
    previous_mode_speed_ = true;
  }
  else if (current_msg->data != 0 || brake_msg->data != 0)
  {
    previous_mode_speed_ = false;
  }
}

}  // namespace vesc_ackermann
