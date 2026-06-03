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

#ifndef EASYNAV_EXPERIMENTS__NAV_METRICS_NODE_HPP_
#define EASYNAV_EXPERIMENTS__NAV_METRICS_NODE_HPP_

#include <limits>
#include <string>
#include <vector>

#include "action_msgs/msg/goal_status_array.hpp"
#include "easynav_interfaces/msg/navigation_control.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace easynav_experiments
{

class BenchmarkEvaluator : public rclcpp::Node
{
public:
  BenchmarkEvaluator();

private:
  struct Sample
  {
    double t;
    unsigned long long cpu_ticks;
    double mem;
  };

  bool active_ = false;
  double start_time_ = 0;
  double end_time_ = 0;
  double min_dist_to_obstacle_ = std::numeric_limits<double>::infinity();
  std::vector<Sample> samples_;

  int pid_;
  int run_id_ = 0;
  long clk_tck_ = 1;
  int cpu_cores_ = 1;
  std::string output_dir_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr status_sub_;
  rclcpp::Subscription<easynav_interfaces::msg::NavigationControl>::SharedPtr easynav_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  double now_sec();
  unsigned long long read_cpu_ticks();
  double get_mem();

  void scan_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void status_cb(const action_msgs::msg::GoalStatusArray::SharedPtr msg);
  void easynav_cb(const easynav_interfaces::msg::NavigationControl::SharedPtr msg);
  void sample();
  void export_json();
};

}  // namespace easynav_experiments

#endif  // EASYNAV_EXPERIMENTS__NAV_METRICS_NODE_HPP_
