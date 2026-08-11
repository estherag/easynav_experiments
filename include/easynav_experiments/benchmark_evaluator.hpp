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

#ifndef EASYNAV_EXPERIMENTS__BENCHMARK_EVALUATOR_HPP_
#define EASYNAV_EXPERIMENTS__BENCHMARK_EVALUATOR_HPP_

#include <chrono>
#include <cstddef>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "easynav_interfaces/msg/navigation_control.hpp"
#include "easynav_system/GoalManagerClient.hpp"

namespace easynav_experiments
{

/// Benchmark node for evaluating navigation frameworks.
class BenchmarkEvaluator : public rclcpp::Node
{
public:
  explicit BenchmarkEvaluator(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  enum class BenchmarkState
  {
    IDLE,
    WAITING_DISCOVERY,
    SENDING_GOAL,
    NAVIGATING,
    NEXT_GOAL,
    FINISHED,
    ERROR
  };

  struct BenchmarkSample
  {
    double time{0.0};

    unsigned long long cpu_ticks{0};
    double memory{0.0};

    double obstacle_distance{std::numeric_limits<double>::infinity()};
    double cmd_vel_frequency{0.0};
    double distance_travelled{0.0};
  };

  void initialize();
  void cycle();
  void resolve_target_pid();

  // Benchmark
  void start_measurement();
  void sample();

  // Metrics
  unsigned long long read_cpu_ticks();
  double get_memory_usage();

  // Callbacks
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void cmd_vel_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void control_callback(const easynav_interfaces::msg::NavigationControl::SharedPtr msg);

  // Navigation
  bool send_goal();
  bool goal_succeeded();
  bool goal_failed();
  void cancel_goal();

  // Parameters
  std::string navigation_;
  std::string target_;
  std::string frame_id_;
  std::string output_file_;

  std::size_t total_cycles_{20};

  // State
  BenchmarkState state_{BenchmarkState::IDLE};
  bool initialized_{false};
  bool error_handled_{false};
  rclcpp::Time discovery_wait_start_;

  std::size_t current_cycle_{0};
  std::size_t current_waypoint_index_{0};
  pid_t pid_{-1};

  // Waypoints
  std::vector<geometry_msgs::msg::PoseStamped> waypoints_;

  // Samples
  std::vector<BenchmarkSample> samples_;

  // Runtime
  rclcpp::Time start_time_;
  rclcpp::Time last_sample_time_;
  long clk_tck_{100};
  static constexpr std::chrono::milliseconds SAMPLING_PERIOD{200};

  double distance_travelled_{0.0};
  double current_obstacle_distance_{std::numeric_limits<double>::infinity()};
  double current_cmd_vel_frequency_{0.0};

  bool has_last_odom_{false};
  geometry_msgs::msg::Point last_odom_position_;

  std::vector<rclcpp::Time> cmd_vel_stamps_;

  // ROS interfaces
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_sub_;

  // Navigation interfaces
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr   nav2_client_;

  rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr nav2_goal_handle_;

  bool nav2_goal_finished_{false};
  bool nav2_goal_success_{false};

  std::shared_ptr<easynav::GoalManagerClient> goal_manager_client_;
};

}  // namespace easynav_experiments

#endif  // EASYNAV_EXPERIMENTS__BENCHMARK_EVALUATOR_HPP_
