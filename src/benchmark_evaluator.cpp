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

#include <algorithm>
#include <fstream>
#include <string>
#include <unistd.h>
#include <dirent.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace easynav_experiments
{

using namespace std::chrono_literals;

BenchmarkEvaluator::BenchmarkEvaluator(const rclcpp::NodeOptions & options)
: Node("benchmark_evaluator", options)
{
  // Benchmark parameters
  declare_parameter<std::string>("navigation", "nav2");
  declare_parameter<std::string>("target", "component_conta");
  declare_parameter<int>("cycles", 20);
  declare_parameter<std::string>("output_file", "benchmark.csv");
  declare_parameter<std::string>("frame_id", "map");
  declare_parameter<std::vector<std::string>>("waypoints", std::vector<std::string>{});

  // Timer benchmark state machine
  timer_ = create_wall_timer(50ms, std::bind(&BenchmarkEvaluator::cycle, this));
}
void BenchmarkEvaluator::initialize()
{
  // Load benchmark parameters
  get_parameter("navigation", navigation_);
  get_parameter("target", target_);
  get_parameter("cycles", total_cycles_);
  get_parameter("output_file", output_file_);
  get_parameter("frame_id", frame_id_);

  std::vector<std::string> waypoint_names;
  get_parameter("waypoints", waypoint_names);

  if (waypoint_names.empty()) {
    RCLCPP_ERROR(get_logger(), "No waypoints provided");
    return;
  }

  waypoints_.clear();

  for (const auto & waypoint_name : waypoint_names) {
    std::vector<double> waypoint_values;

    declare_parameter(waypoint_name, waypoint_values);
    get_parameter(waypoint_name, waypoint_values);

    if (waypoint_values.size() != 3) {
      RCLCPP_ERROR(get_logger(), "Waypoint '%s' must contain x, y and yaw", waypoint_name.c_str());
      continue;
    }

    geometry_msgs::msg::PoseStamped waypoint;
    waypoint.header.frame_id = frame_id_;
    waypoint.header.stamp = now();

    waypoint.pose.position.x = waypoint_values[0];
    waypoint.pose.position.y = waypoint_values[1];

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, waypoint_values[2]);
    waypoint.pose.orientation = tf2::toMsg(q);

    waypoints_.push_back(waypoint);
  }

  clk_tck_ = sysconf(_SC_CLK_TCK);

  // Create subscriptions
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "scan_raw", 10, std::bind(&BenchmarkEvaluator::scan_callback, this, std::placeholders::_1));

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, std::bind(&BenchmarkEvaluator::odom_callback, this, std::placeholders::_1));

  cmd_vel_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      "cmd_vel", 10, std::bind(&BenchmarkEvaluator::cmd_vel_callback, this, std::placeholders::_1));
}

void BenchmarkEvaluator::start_measurement()
{
  // Find the navigation process
  pid_ = -1;
  resolve_target_pid();

  if (pid_ <= 0) {
    RCLCPP_ERROR(get_logger(), "Unable to find navigation process");
    state_ = BenchmarkState::ERROR;
    return;
  }

  // Reset benchmark data
  samples_.clear();

  distance_travelled_ = 0.0;
  current_cmd_vel_frequency_ = 0.0;
  current_obstacle_distance_ = std::numeric_limits<double>::infinity();

  has_last_odom_ = false;
  cmd_vel_stamps_.clear();

  start_time_ = now();
  last_sample_time_ = start_time_;

}

void BenchmarkEvaluator::cycle()
{
  switch (state_) {
    case BenchmarkState::IDLE:

      initialize();

      if (!initialized_) {
        RCLCPP_INFO(get_logger(), "Initializing benchmark evaluator");

        if (navigation_ == "nav2") {
          nav2_client_ =
            rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(shared_from_this(),
            "navigate_to_pose");
        } else {
          goal_manager_client_ = std::make_shared<easynav::GoalManagerClient>(shared_from_this());
        }

        initialized_ = true;

        if (waypoints_.empty()) {
          RCLCPP_ERROR(get_logger(), "Cannot start benchmark with no waypoints");
          state_ = BenchmarkState::ERROR;
          break;
        }
      }

      discovery_wait_start_ = now();
      state_ = BenchmarkState::WAITING_DISCOVERY;
      break;

    case BenchmarkState::WAITING_DISCOVERY:
      {
        bool ready = false;

        if (navigation_ == "nav2") {
          ready = nav2_client_->action_server_is_ready();
        } else {
          ready = goal_manager_client_->is_connected();
        }

        if (!ready) {
          if ((now() - discovery_wait_start_).seconds() > 5.0) {
            RCLCPP_ERROR(get_logger(), "Timed out waiting for navigation server discovery");
            state_ = BenchmarkState::ERROR;
          }
          break;
        }

        RCLCPP_INFO(get_logger(), "Navigation backend discovered");

        start_measurement();
        if (state_ == BenchmarkState::ERROR) {
          break;
        }

        state_ = BenchmarkState::SENDING_GOAL;
        break;
      }

    case BenchmarkState::SENDING_GOAL:
      if (send_goal()) {
        state_ = BenchmarkState::NAVIGATING;
      } else {
        state_ = BenchmarkState::ERROR;
      }
      break;

    case BenchmarkState::NAVIGATING:
      // Sample metrics at a lower, fixed rate than control loop
      if ((now() - last_sample_time_) > rclcpp::Duration(SAMPLING_PERIOD)) {
        sample();
        last_sample_time_ = now();
      }

      if (goal_succeeded()) {
        state_ = BenchmarkState::NEXT_GOAL;
      } else if (goal_failed()) {
        state_ = BenchmarkState::ERROR;
      }
      break;

    case BenchmarkState::NEXT_GOAL:
      if (navigation_ == "easynav") {
        goal_manager_client_->reset();
      }

      ++current_waypoint_index_;

      if (current_waypoint_index_ >= waypoints_.size()) {
        current_waypoint_index_ = 0;
        ++current_cycle_;
      }

      state_ = (current_cycle_ >=
        total_cycles_) ? BenchmarkState::FINISHED : BenchmarkState::SENDING_GOAL;
      break;

    case BenchmarkState::FINISHED:
      store_results();
      RCLCPP_INFO(get_logger(), "Benchmark finished");
      rclcpp::shutdown();
      break;

    case BenchmarkState::ERROR:
      cancel_goal();
      store_results();
      rclcpp::shutdown();
      break;
  }
}

void BenchmarkEvaluator::sample()
{
  BenchmarkSample sample;

  // Collect current metrics
  sample.time = (now() - start_time_).seconds();
  sample.cpu_ticks = read_cpu_ticks();
  sample.memory = get_memory_usage();
  sample.obstacle_distance = current_obstacle_distance_;
  sample.cmd_vel_frequency = current_cmd_vel_frequency_;
  sample.distance_travelled = distance_travelled_;

  samples_.push_back(sample);

}

void BenchmarkEvaluator::store_results()
{
  std::ofstream file(output_file_);

  if (!file.is_open()) {
    RCLCPP_ERROR(get_logger(), "Failed to open output file '%s'", output_file_.c_str());
    return;
  }

  file << "time,cpu,memory,"
    "obstacle_distance,cmd_vel_frequency,"
    "distance_travelled\n";

  for (std::size_t i = 0; i < samples_.size(); ++i) {
    double cpu = 0.0;

    // Compute CPU usage from consecutive samples
    if (i > 0) {
      const double dt = samples_[i].time - samples_[i - 1].time;

      if (dt > 0.0) {
        cpu = 100.0 *
          (static_cast<double>(samples_[i].cpu_ticks - samples_[i - 1].cpu_ticks) /
          static_cast<double>(clk_tck_)) /
          dt;
      }
    }

    file << samples_[i].time << ',' << cpu << ',' << samples_[i].memory << ',' <<
      samples_[i].obstacle_distance << ','
         << samples_[i].cmd_vel_frequency << ',' << samples_[i].distance_travelled << '\n';
  }

  RCLCPP_INFO(get_logger(), "Saved %zu samples to '%s'", samples_.size(), output_file_.c_str());
}

bool BenchmarkEvaluator::send_goal()
{
  if (navigation_ == "nav2") {
    if (!nav2_client_->wait_for_action_server(5s)) {
      RCLCPP_ERROR(get_logger(), "NavigateToPose action server not available");
      return false;
    }

    nav2_goal_finished_ = false;
    nav2_goal_success_ = false;

    nav2_msgs::action::NavigateToPose::Goal goal;
    goal.pose = waypoints_[current_waypoint_index_];

    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions options;

    options.goal_response_callback =
      [this](rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr
      goal_handle) {
        nav2_goal_handle_ = goal_handle;

        if (!goal_handle) {
          nav2_goal_finished_ = true;
          nav2_goal_success_ = false;
        }
      };

    options.result_callback =
      [this](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult
      & result) {
        nav2_goal_finished_ = true;
        nav2_goal_success_ = result.code == rclcpp_action::ResultCode::SUCCEEDED;
      };

    nav2_client_->async_send_goal(goal, options);
    return true;
  }
  RCLCPP_INFO(get_logger(), "Sending waypoint %.2f %.2f",
      waypoints_[current_waypoint_index_].pose.position.x,
              waypoints_[current_waypoint_index_].pose.position.y);

  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = frame_id_;
  goal.header.stamp = now();
  goal.pose = waypoints_[current_waypoint_index_].pose;

  goal_manager_client_->send_goal(goal);

  return true;
}

bool BenchmarkEvaluator::goal_succeeded()
{
  if (navigation_ == "nav2") {
    return nav2_goal_finished_ && nav2_goal_success_;
  }

  return goal_manager_client_->get_state() ==
         easynav::GoalManagerClient::State::NAVIGATION_FINISHED;
}

void BenchmarkEvaluator::cancel_goal()
{
  if (navigation_ == "nav2") {
    if (nav2_goal_handle_) {
      nav2_client_->async_cancel_goal(nav2_goal_handle_);
    }

    return;
  }

  goal_manager_client_->cancel();
}

bool BenchmarkEvaluator::goal_failed()
{
  if (navigation_ == "nav2") {
    return nav2_goal_finished_ && !nav2_goal_success_;
  }

  auto state = goal_manager_client_->get_state();

  return state == easynav::GoalManagerClient::State::NAVIGATION_FAILED ||
         state == easynav::GoalManagerClient::State::NAVIGATION_CANCELLED ||
         state == easynav::GoalManagerClient::State::ERROR;
}

void BenchmarkEvaluator::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (!has_last_odom_) {
    last_odom_position_ = msg->pose.pose.position;
    has_last_odom_ = true;
    return;
  }

  const double dx = msg->pose.pose.position.x - last_odom_position_.x;
  const double dy = msg->pose.pose.position.y - last_odom_position_.y;

  distance_travelled_ += std::hypot(dx, dy);

  last_odom_position_ = msg->pose.pose.position;
}

void BenchmarkEvaluator::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  // Keep the minimum valid obstacle distance
  for (const auto & range : msg->ranges) {
    if (!std::isfinite(range) || range < msg->range_min || range > msg->range_max) {
      continue;
    }

    current_obstacle_distance_ = std::min(current_obstacle_distance_, static_cast<double>(range));
  }
}

void BenchmarkEvaluator::cmd_vel_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  const auto stamp = msg->header.stamp;

  cmd_vel_stamps_.push_back(stamp);

  // Keep a sliding window of timestamps
  while (cmd_vel_stamps_.size() > 50) {
    cmd_vel_stamps_.erase(cmd_vel_stamps_.begin());
  }

  if (cmd_vel_stamps_.size() >= 2) {
    const double dt = (cmd_vel_stamps_.back() - cmd_vel_stamps_.front()).seconds();

    if (dt > 0.0) {
      current_cmd_vel_frequency_ = static_cast<double>(cmd_vel_stamps_.size() - 1) / dt;
    }
  }
}
unsigned long long BenchmarkEvaluator::read_cpu_ticks()
{
  if (pid_ <= 0) {
    return 0;
  }

  std::ifstream file("/proc/" + std::to_string(pid_) + "/stat");

  if (!file.is_open()) {
    return 0;
  }

  std::string tmp;

  // Skip the first 13 fields to reach utime (field 14)
  for (int i = 0; i < 13; ++i) {
    file >> tmp;
  }

  unsigned long utime;
  unsigned long stime;

  file >> utime >> stime;

  return static_cast<unsigned long long>(utime + stime);
}

double BenchmarkEvaluator::get_memory_usage()
{
  if (pid_ <= 0) {
    return 0.0;
  }

  std::ifstream file("/proc/" + std::to_string(pid_) + "/status");

  if (!file.is_open()) {
    return 0.0;
  }

  std::string line;

  while (std::getline(file, line)) {
    if (line.rfind("VmRSS:", 0) == 0) {
      std::istringstream iss(line);

      std::string key;
      std::string unit;
      double value;

      iss >> key >> value >> unit;

      return value / 1024.0;  // kB to MB
    }
  }

  return 0.0;
}

void BenchmarkEvaluator::resolve_target_pid()
{
  pid_ = -1;

  DIR * dir = opendir("/proc");

  if (dir == nullptr) {
    RCLCPP_ERROR(get_logger(), "Unable to open /proc");
    return;
  }

  struct dirent * entry;

  while ((entry = readdir(dir)) != nullptr) {
    std::string pid_str(entry->d_name);

    if (!std::all_of(pid_str.begin(), pid_str.end(), ::isdigit)) {
      continue;
    }

    std::ifstream file("/proc/" + pid_str + "/comm");

    std::string process_name;
    std::getline(file, process_name);

    if (process_name == target_) {
      pid_ = std::stoi(pid_str);

      RCLCPP_INFO(get_logger(), "Monitoring process '%s' (PID %d)", target_.c_str(), pid_);

      break;
    }
  }

  closedir(dir);

  if (pid_ < 0) {
    RCLCPP_WARN(get_logger(), "Unable to find process '%s'", target_.c_str());
  }
}

}  // namespace easynav_experiments
