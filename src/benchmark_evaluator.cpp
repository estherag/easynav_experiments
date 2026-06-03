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

#include "easynav_experiments/benchmark_evaluator.hpp"

#include <unistd.h>

#include <chrono>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>

#include "easynav_interfaces/msg/navigation_control.hpp"

using std::placeholders::_1;

namespace easynav_experiments
{

BenchmarkEvaluator::BenchmarkEvaluator() : Node("nav_metrics_node")
{
  declare_parameter<int>("target_pid", -1);
  declare_parameter<int>("run_id", 0);
  declare_parameter<std::string>("nav_mode", "nav2");  // "nav2" or "easynav"
  declare_parameter<std::string>("output_dir", ".");

  pid_ = get_parameter("target_pid").get_value<int>();
  run_id_ = get_parameter("run_id").get_value<int>();
  auto nav_mode = get_parameter("nav_mode").get_value<std::string>();
  output_dir_ = get_parameter("output_dir").get_value<std::string>();
  clk_tck_ = sysconf(_SC_CLK_TCK);
  cpu_cores_ = sysconf(_SC_NPROCESSORS_ONLN);

  scan_sub_ =
      create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&BenchmarkEvaluator::scan_cb, this, _1));

  if (nav_mode == "easynav")
  {
    easynav_sub_ = create_subscription<easynav_interfaces::msg::NavigationControl>(
        "easynav_control", 100, std::bind(&BenchmarkEvaluator::easynav_cb, this, _1));
  }
  else
  {
    status_sub_ = create_subscription<action_msgs::msg::GoalStatusArray>(
        "/navigate_to_pose/_action/status", 10, std::bind(&BenchmarkEvaluator::status_cb, this, _1));
  }

  timer_ = create_wall_timer(std::chrono::milliseconds(200), std::bind(&BenchmarkEvaluator::sample, this));
}

double BenchmarkEvaluator::now_sec()
{
  return this->get_clock()->now().seconds();
}

unsigned long long BenchmarkEvaluator::read_cpu_ticks()
{
  std::ifstream file("/proc/" + std::to_string(pid_) + "/stat");
  if (!file.is_open())
  {
    return 0;
  }

  std::string tmp;
  for (int i = 0; i < 13; i++)
  {
    file >> tmp;
  }

  unsigned long utime, stime;
  file >> utime >> stime;
  return static_cast<unsigned long long>(utime + stime);
}

double BenchmarkEvaluator::get_mem()
{
  std::ifstream file("/proc/" + std::to_string(pid_) + "/status");
  std::string line;

  while (std::getline(file, line))
  {
    if (line.rfind("VmRSS:", 0) == 0)
    {
      std::istringstream iss(line);
      std::string key, unit;
      double value;
      iss >> key >> value >> unit;
      return value / 1024.0;
    }
  }
  return 0.0;
}

void BenchmarkEvaluator::scan_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  if (!active_)
  {
    return;
  }

  for (auto r : msg->ranges)
  {
    if (r > msg->range_min && r < msg->range_max)
    {
      min_dist_to_obstacle_ = std::min(min_dist_to_obstacle_, (double)r);
    }
  }
}

void BenchmarkEvaluator::status_cb(const action_msgs::msg::GoalStatusArray::SharedPtr msg)
{
  if (msg->status_list.empty())
  {
    return;
  }

  int status = msg->status_list.back().status;

  // EXECUTING
  if (status == 2 && !active_)
  {
    active_ = true;
    start_time_ = now_sec();
    min_dist_to_obstacle_ = std::numeric_limits<double>::infinity();
    samples_.clear();
    RCLCPP_INFO(get_logger(), "Navigation run started");
  }

  // SUCCEEDED
  if (status == 4 && active_)
  {
    active_ = false;
    end_time_ = now_sec();
    export_json();
    RCLCPP_INFO(get_logger(), "Navigation run finished (%.2f s)", end_time_ - start_time_);
  }

  // CANCELED or ABORTED
  if ((status == 5 || status == 6) && active_)
  {
    active_ = false;
    RCLCPP_WARN(get_logger(), "Navigation run canceled or aborted, discarding data");
  }
}

void BenchmarkEvaluator::easynav_cb(const easynav_interfaces::msg::NavigationControl::SharedPtr msg)
{
  using NC = easynav_interfaces::msg::NavigationControl;

  // Accepted goal
  if (msg->type == NC::ACCEPT && !active_)
  {
    active_ = true;
    start_time_ = now_sec();
    min_dist_to_obstacle_ = std::numeric_limits<double>::infinity();
    samples_.clear();
    RCLCPP_INFO(get_logger(), "Navigation run started (EasyNav)");
  }

  // Nav finished
  if (msg->type == NC::FINISHED && active_)
  {
    active_ = false;
    end_time_ = now_sec();
    export_json();
    RCLCPP_INFO(get_logger(), "Navigation run finished (%.2f s)", end_time_ - start_time_);
  }

  // Nav cancelled
  if ((msg->type == NC::CANCELLED || msg->type == NC::FAILED) && active_)
  {
    active_ = false;
    RCLCPP_WARN(get_logger(), "Navigation run cancelled or failed, discarding data");
  }
}

void BenchmarkEvaluator::sample()
{
  if (!active_ || pid_ <= 0)
  {
    return;
  }

  Sample s;
  s.t = now_sec();
  s.cpu_ticks = read_cpu_ticks();
  s.mem = get_mem();

  samples_.push_back(s);
}

// Logger
void BenchmarkEvaluator::export_json()
{
  std::string filename = output_dir_ + "/nav_metrics_" + std::to_string(run_id_) + ".json";
  std::ofstream f(filename);

  double cpu_sum = 0, mem_sum = 0;
  double cpu_max = 0;

  for (size_t i = 1; i < samples_.size(); ++i)
  {
    double dt = samples_[i].t - samples_[i - 1].t;
    if (dt <= 0.0)
    {
      continue;
    }
    double cpu = 100.0 * ((double)(samples_[i].cpu_ticks - samples_[i - 1].cpu_ticks) / clk_tck_) / dt / cpu_cores_;
    cpu_sum += cpu;
    cpu_max = std::max(cpu_max, cpu);
    mem_sum += samples_[i].mem;
  }

  double n = static_cast<double>(std::max<size_t>(1, samples_.size() - 1));

  f << "{" << "\"run_id\":" << run_id_ << "," << "\"duration\":" << (end_time_ - start_time_) << ","
    << "\"cpu_mean\":" << cpu_sum / n << "," << "\"cpu_max\":" << cpu_max << "," << "\"mem_mean_mb\":" << mem_sum / n
    << "," << "\"min_dist_to_obstacle\":" << min_dist_to_obstacle_ << "}\n";

  RCLCPP_INFO(get_logger(), "Metrics saved to %s", filename.c_str());
}

}  // namespace easynav_experiments
