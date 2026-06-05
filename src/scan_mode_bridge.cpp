// Copyright 2026 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "easynav_experiments/scan_mode_bridge.hpp"

#include <algorithm>
#include <cinttypes>
#include <cmath>
#include <string>
#include <utility>

namespace easynav_experiments
{

namespace
{
constexpr const char * mode_to_string(ScanModeBridge::Mode mode)
{
  switch (mode) {
    case ScanModeBridge::Mode::BRIDGE:
      return "BRIDGE";
    case ScanModeBridge::Mode::BLOCKED:
      return "BLOCKED";
    default:
      return "UNKNOWN";
  }
}
}  // namespace

ScanModeBridge::ScanModeBridge(const rclcpp::NodeOptions & options)
: rclcpp::Node("scan_mode_bridge", options)
{
  declare_parameter("dist_blocked", 0.05);
  get_parameter("dist_blocked", dist_blocked_);

  declare_parameter("stop_lin_eps", stop_lin_eps_);
  get_parameter("stop_lin_eps", stop_lin_eps_);
  declare_parameter("stop_ang_eps", stop_ang_eps_);
  get_parameter("stop_ang_eps", stop_ang_eps_);

  if (!std::isfinite(dist_blocked_) || dist_blocked_ < 0.0f) {
    RCLCPP_WARN(get_logger(), "Invalid parameter dist_blocked=%.6f; using default 0.05",
        dist_blocked_);
    dist_blocked_ = 0.35f;
  }

  scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("scan_bridged",
      rclcpp::SensorDataQoS());

  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "scan_raw", rclcpp::SensorDataQoS(),
      std::bind(&ScanModeBridge::on_scan, this, std::placeholders::_1));

  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", rclcpp::SystemDefaultsQoS(),
      std::bind(&ScanModeBridge::on_cmd_vel, this, std::placeholders::_1));

  trigger_srv_ = create_service<std_srvs::srv::Trigger>(
      "trigger_mode",
      std::bind(&ScanModeBridge::on_trigger_mode, this, std::placeholders::_1,
      std::placeholders::_2));

  RCLCPP_INFO(get_logger(), "Started in mode: %s", mode_to_string(mode_));
}

void ScanModeBridge::on_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  if (mode_ == Mode::BRIDGE) {
    scan_pub_->publish(*msg);
    return;
  }

  {
    std::scoped_lock<std::mutex> lock(measurement_mutex_);
    if (stop_measurement_active_ && !stop_measurement_started_) {
      stop_measurement_start_time_ = steady_clock_.now();
      stop_measurement_started_ = true;
      RCLCPP_INFO(get_logger(), "Stop-timer started (mode=BLOCKED)");
    }
  }

  auto blocked = make_blocked(*msg);
  scan_pub_->publish(blocked);
}

void ScanModeBridge::on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  if (!msg) {
    return;
  }

  std::scoped_lock<std::mutex> lock(measurement_mutex_);
  if (mode_ != Mode::BLOCKED || !stop_measurement_active_ || !stop_measurement_started_) {
    return;
  }

  const double lin =
    std::sqrt(msg->linear.x * msg->linear.x + msg->linear.y * msg->linear.y + msg->linear.z *
      msg->linear.z);
  const double ang =
    std::sqrt(msg->angular.x * msg->angular.x + msg->angular.y * msg->angular.y + msg->angular.z *
      msg->angular.z);

  const bool stopped = (lin <= stop_lin_eps_) && (ang <= stop_ang_eps_);
  if (!stopped) {
    return;
  }

  const auto end_time = steady_clock_.now();
  const auto dt = end_time - stop_measurement_start_time_;
  const int64_t dt_us = dt.nanoseconds() / 1000;
  RCLCPP_INFO(get_logger(),
      "Robot stop detected after %" PRId64 " us (lin=%.4f, ang=%.4f; eps=(%.4f, %.4f))", dt_us,
              lin, ang, stop_lin_eps_, stop_ang_eps_);

  stop_measurement_active_ = false;
}

void ScanModeBridge::on_trigger_mode(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>/*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  mode_ = (mode_ == Mode::BRIDGE) ? Mode::BLOCKED : Mode::BRIDGE;

  {
    std::scoped_lock<std::mutex> lock(measurement_mutex_);
    if (mode_ == Mode::BLOCKED) {
      stop_measurement_active_ = true;
      stop_measurement_started_ = false;
      stop_measurement_start_time_ = rclcpp::Time(0, 0, RCL_STEADY_TIME);
    } else {
      stop_measurement_active_ = false;
      stop_measurement_started_ = false;
    }
  }

  if (response) {
    response->success = true;
    response->message = std::string("Mode switched to ") + mode_to_string(mode_);
  }

  RCLCPP_INFO(get_logger(), "Mode switched to: %s", mode_to_string(mode_));
}

sensor_msgs::msg::LaserScan ScanModeBridge::make_blocked(
  const sensor_msgs::msg::LaserScan & input) const
{
  sensor_msgs::msg::LaserScan out = input;

  const float min_r = std::isfinite(input.range_min) ? input.range_min : 0.0f;
  const float max_r = std::isfinite(input.range_max) ? input.range_max : min_r;
  const float blocked = std::clamp(dist_blocked_, min_r, max_r);

  for (auto & r : out.ranges) {
    r = blocked;
  }

  return out;
}

}  // namespace easynav_experiments
