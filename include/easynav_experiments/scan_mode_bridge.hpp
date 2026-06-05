// Copyright 2025 Intelligent Robotics Lab
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

#ifndef EASYNAV_EXPERIMENTS__SCAN_MODE_BRIDGE_HPP_
#define EASYNAV_EXPERIMENTS__SCAN_MODE_BRIDGE_HPP_

#include <mutex>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace easynav_experiments
{

class ScanModeBridge final : public rclcpp::Node
{
public:
  enum class Mode
  {
    BRIDGE = 0,
    BLOCKED = 1,
  };

  explicit ScanModeBridge(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg);

  void on_trigger_mode(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  sensor_msgs::msg::LaserScan make_blocked(const sensor_msgs::msg::LaserScan & input) const;

  Mode mode_{Mode::BRIDGE};
  float dist_blocked_{0.35f};

  double stop_lin_eps_{0.01};
  double stop_ang_eps_{0.01};

  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

  mutable std::mutex measurement_mutex_;
  bool stop_measurement_active_{false};
  bool stop_measurement_started_{false};
  rclcpp::Time stop_measurement_start_time_{0, 0, RCL_STEADY_TIME};

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_srv_;
};

}  // namespace easynav_experiments

#endif  // EASYNAV_EXPERIMENTS__SCAN_MODE_BRIDGE_HPP_
