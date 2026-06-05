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

#ifndef EASYNAV_EXPERIMENTS__BENCHMARK_EVALUATOR_NODE_HPP_
#define EASYNAV_EXPERIMENTS__BENCHMARK_EVALUATOR_NODE_HPP_

#include <fstream>
#include <limits>
#include <string>
#include <vector>

#include "action_msgs/msg/goal_status_array.hpp"
#include "easynav_interfaces/msg/navigation_control.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace easynav_experiments
{

/**
 * @brief ROS 2 node that records CPU, RAM and laser safety metrics during a
 *        single navigation run and exports them to CSV + JSON summary.
 *
 * Structure:
 *   1. Waits for a goal accepted event (EasyNav: NavigationControl::ACCEPT,
 *      nav2: GoalStatusArray status == EXECUTING).
 *   2. Samples CPU%, RAM (MB) and min laser distance at 5 Hz until the run ends.
 *   3. On goal reached, writes:
 *        - <output_dir>/benchmark_evaluator_<mode>_<run_id>.csv  (time-series)
 *        - <output_dir>/benchmark_evaluator_<mode>_<run_id>_summary.json
 *
 * Parameters:
 *   - target_pid  (int)    : PID of the navigation process to monitor.
 *   - run_id      (int)    : Identifier appended to output filenames.
 *   - nav_mode    (string) : "easynav" or "nav2".
 *   - output_dir  (string) : Directory where results are saved.
 */
class BenchmarkEvaluator : public rclcpp::Node
{
public:
  BenchmarkEvaluator();

private:
  /// One resource snapshot taken at certain freq (200 ms) while navigation is active.
  struct Sample
  {
    double t;                      ///< Wall-clock time (seconds).
    unsigned long long cpu_ticks;  ///< Cumulative utime+stime from /proc/<pid>/stat.
    double mem;                    ///< Resident set size in MB (VmRSS).
    double min_dist_scan;          ///< Minimum laser range in the latest scan (m).
  };

  bool active_ = false;    ///< True while a navigation run is being recorded.
  double start_time_ = 0;  ///< Wall-clock time at run start (s).
  double end_time_ = 0;    ///< Wall-clock time at run end (s).
  double min_dist_to_obstacle_ =
    std::numeric_limits<double>::infinity();    ///< Global minimum laser range over the whole run (m).
  double last_scan_min_dist_ =
    std::numeric_limits<double>::infinity();    ///< Min range from the most recent laser scan (m).
  std::vector<Sample> samples_;                 ///< All samples collected during the run.

  int pid_;                 ///< PID of the monitored navigation process.
  int run_id_ = 0;          ///< Run identifier used in output filenames.
  long clk_tck_ = 1;        ///< Kernel clock ticks per second (_SC_CLK_TCK).
  int cpu_cores_ = 1;       ///< Number of logical CPU cores (for normalisation).
  std::string output_dir_;  ///< Directory where CSV and JSON are written.
  std::string nav_mode_;    ///< "easynav" or "nav2", used in filenames.

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr status_sub_;
  rclcpp::Subscription<easynav_interfaces::msg::NavigationControl>::SharedPtr easynav_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  /// @returns Current wall-clock time in seconds.
  double now_sec();
  /// @returns Cumulative CPU ticks (utime+stime) for pid_ from /proc.
  unsigned long long read_cpu_ticks();
  /// @returns Resident memory of pid_ in MB from /proc.
  double get_mem();

  /// Updates min laser distance; active only while active_ is true.
  void scan_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  /// Tracks nav2 action status: starts/stops recording on EXECUTING/SUCCEEDED.
  void status_cb(const action_msgs::msg::GoalStatusArray::SharedPtr msg);
  /// Tracks EasyNav control messages: starts/stops recording on ACCEPT/FINISHED.
  void easynav_cb(const easynav_interfaces::msg::NavigationControl::SharedPtr msg);
  /// Called at 5 Hz; appends one Sample to samples_ while active_.
  void sample();
  /// Writes time-series CSV and summary JSON, then calls rclcpp::shutdown().
  void export_csv();
};

}  // namespace easynav_experiments

#endif  // EASYNAV_EXPERIMENTS__BENCHMARK_EVALUATOR_NODE_HPP_
