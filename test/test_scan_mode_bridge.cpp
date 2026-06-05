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

#include <gtest/gtest.h>

#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "rclcpp/executors/single_threaded_executor.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;

namespace
{

sensor_msgs::msg::LaserScan make_scan(const std::vector<float> & ranges)
{
  sensor_msgs::msg::LaserScan msg;
  msg.header.frame_id = "laser";
  msg.angle_min = 0.0f;
  msg.angle_max = 1.0f;
  msg.angle_increment = 0.1f;
  msg.time_increment = 0.0f;
  msg.scan_time = 0.1f;
  msg.range_min = 0.0f;
  msg.range_max = 30.0f;
  msg.ranges = ranges;
  return msg;
}

bool spin_until(
  rclcpp::Executor & exec, const std::function<bool()> & pred,
  std::chrono::milliseconds timeout)
{
  const auto start = std::chrono::steady_clock::now();
  while (!pred()) {
    exec.spin_some();
    if ((std::chrono::steady_clock::now() - start) > timeout) {
      return false;
    }
    std::this_thread::sleep_for(2ms);
  }
  return true;
}

}  // namespace

class ScanModeBridgeTest : public ::testing::Test
{
};

TEST_F(ScanModeBridgeTest, BridgesByDefault)
{
  auto bridge_options = rclcpp::NodeOptions().arguments({
    "--ros-args",
    "-r",
    "scan_raw:=test_scan",
    "-r",
    "scan_bridged:=test_scan_bridged",
    "-r",
    "trigger_mode:=test_trigger_mode",
  });

  auto bridge = std::make_shared<easynav_experiments::ScanModeBridge>(bridge_options);
  auto harness = std::make_shared<rclcpp::Node>("harness_bridges_by_default");

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(bridge);
  exec.add_node(harness);

  std::promise<sensor_msgs::msg::LaserScan> received_promise;
  auto received_future = received_promise.get_future();
  auto sub = harness->create_subscription<sensor_msgs::msg::LaserScan>(
      "test_scan_bridged", rclcpp::SensorDataQoS(),
    [&received_promise](sensor_msgs::msg::LaserScan::SharedPtr msg) {
      if (msg) {
        received_promise.set_value(*msg);
      }
      });

  auto pub = harness->create_publisher<sensor_msgs::msg::LaserScan>("test_scan",
    rclcpp::SensorDataQoS());

  ASSERT_TRUE(spin_until(exec, [&]() {return pub->get_subscription_count() > 0;}, 2s));

  const auto input = make_scan({1.0f, 2.0f, 3.0f});
  pub->publish(input);

  ASSERT_TRUE(spin_until(exec, [&]() {
      return received_future.wait_for(0ms) == std::future_status::ready;
                                                                                                        },
    2s));

  const auto out = received_future.get();
  ASSERT_EQ(out.ranges.size(), input.ranges.size());
  EXPECT_FLOAT_EQ(out.ranges[0], 1.0f);
  EXPECT_FLOAT_EQ(out.ranges[1], 2.0f);
  EXPECT_FLOAT_EQ(out.ranges[2], 3.0f);

  exec.remove_node(harness);
  exec.remove_node(bridge);
}

TEST_F(ScanModeBridgeTest, BlocksAfterTrigger)
{
  auto bridge_options = rclcpp::NodeOptions().arguments({
    "--ros-args",
    "-r",
    "scan_raw:=test_scan",
    "-r",
    "scan_bridged:=test_scan_bridged",
    "-r",
    "trigger_mode:=test_trigger_mode",
  });

  auto bridge = std::make_shared<easynav_experiments::ScanModeBridge>(bridge_options);
  auto harness = std::make_shared<rclcpp::Node>("harness_blocks_after_trigger");

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(bridge);
  exec.add_node(harness);

  auto client = harness->create_client<std_srvs::srv::Trigger>("test_trigger_mode");
  ASSERT_TRUE(client->wait_for_service(2s));

  auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future = client->async_send_request(req);
  ASSERT_EQ(exec.spin_until_future_complete(future, 2s), rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_TRUE(future.get()->success);

  std::promise<sensor_msgs::msg::LaserScan> received_promise;
  auto received_future = received_promise.get_future();
  auto sub = harness->create_subscription<sensor_msgs::msg::LaserScan>(
      "test_scan_bridged", rclcpp::SensorDataQoS(),
    [&received_promise](sensor_msgs::msg::LaserScan::SharedPtr msg) {
      if (msg) {
        received_promise.set_value(*msg);
      }
      });

  auto pub = harness->create_publisher<sensor_msgs::msg::LaserScan>("test_scan",
    rclcpp::SensorDataQoS());
  ASSERT_TRUE(spin_until(exec, [&]() {return pub->get_subscription_count() > 0;}, 2s));

  const auto input = make_scan({10.0f, 10.0f, 10.0f, 1.0f});
  pub->publish(input);

  ASSERT_TRUE(spin_until(exec, [&]() {
      return received_future.wait_for(0ms) == std::future_status::ready;
                                                                                                        },
    2s));

  const auto out = received_future.get();
  ASSERT_EQ(out.ranges.size(), input.ranges.size());
  for (const auto & r : out.ranges) {
    EXPECT_NEAR(r, 0.05f, 1e-6f);
  }

  exec.remove_node(harness);
  exec.remove_node(bridge);
}

TEST_F(ScanModeBridgeTest, ToggleBackToBridge)
{
  auto bridge_options = rclcpp::NodeOptions().arguments({
    "--ros-args",
    "-r",
    "scan_raw:=test_scan",
    "-r",
    "scan_bridged:=test_scan_bridged",
    "-r",
    "trigger_mode:=test_trigger_mode",
  });

  auto bridge = std::make_shared<easynav_experiments::ScanModeBridge>(bridge_options);
  auto harness = std::make_shared<rclcpp::Node>("harness_toggle_back_to_bridge");

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(bridge);
  exec.add_node(harness);

  auto client = harness->create_client<std_srvs::srv::Trigger>("test_trigger_mode");
  ASSERT_TRUE(client->wait_for_service(2s));

  // First call -> BLOCKED
  {
    auto future = client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    ASSERT_EQ(exec.spin_until_future_complete(future, 2s), rclcpp::FutureReturnCode::SUCCESS);
  }
  // Second call -> BRIDGE
  {
    auto future = client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    ASSERT_EQ(exec.spin_until_future_complete(future, 2s), rclcpp::FutureReturnCode::SUCCESS);
  }

  std::promise<sensor_msgs::msg::LaserScan> received_promise;
  auto received_future = received_promise.get_future();
  auto sub = harness->create_subscription<sensor_msgs::msg::LaserScan>(
      "test_scan_bridged", rclcpp::SensorDataQoS(),
    [&received_promise](sensor_msgs::msg::LaserScan::SharedPtr msg) {
      if (msg) {
        received_promise.set_value(*msg);
      }
      });

  auto pub = harness->create_publisher<sensor_msgs::msg::LaserScan>("test_scan",
    rclcpp::SensorDataQoS());
  ASSERT_TRUE(spin_until(exec, [&]() {return pub->get_subscription_count() > 0;}, 2s));

  const auto input = make_scan({0.2f, 0.3f});
  pub->publish(input);

  ASSERT_TRUE(spin_until(exec, [&]() {
      return received_future.wait_for(0ms) == std::future_status::ready;
                                                                                                        },
    2s));

  const auto out = received_future.get();
  ASSERT_EQ(out.ranges.size(), input.ranges.size());
  EXPECT_FLOAT_EQ(out.ranges[0], 0.2f);
  EXPECT_FLOAT_EQ(out.ranges[1], 0.3f);

  exec.remove_node(harness);
  exec.remove_node(bridge);
}

TEST_F(ScanModeBridgeTest, BlocksWithCustomDistanceParameter)
{
  auto bridge_options = rclcpp::NodeOptions().arguments({
    "--ros-args",
    "-p",
    "dist_blocked:=0.12",
    "-r",
    "scan_raw:=test_scan",
    "-r",
    "scan_bridged:=test_scan_bridged",
    "-r",
    "trigger_mode:=test_trigger_mode",
  });

  auto bridge = std::make_shared<easynav_experiments::ScanModeBridge>(bridge_options);
  auto harness = std::make_shared<rclcpp::Node>("harness_blocks_custom_distance");

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(bridge);
  exec.add_node(harness);

  auto client = harness->create_client<std_srvs::srv::Trigger>("test_trigger_mode");
  ASSERT_TRUE(client->wait_for_service(2s));

  // Switch to BLOCKED
  {
    auto future = client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    ASSERT_EQ(exec.spin_until_future_complete(future, 2s), rclcpp::FutureReturnCode::SUCCESS);
    EXPECT_TRUE(future.get()->success);
  }

  std::promise<sensor_msgs::msg::LaserScan> received_promise;
  auto received_future = received_promise.get_future();
  auto sub = harness->create_subscription<sensor_msgs::msg::LaserScan>(
      "test_scan_bridged", rclcpp::SensorDataQoS(),
    [&received_promise](sensor_msgs::msg::LaserScan::SharedPtr msg) {
      if (msg) {
        received_promise.set_value(*msg);
      }
      });

  auto pub = harness->create_publisher<sensor_msgs::msg::LaserScan>("test_scan",
    rclcpp::SensorDataQoS());
  ASSERT_TRUE(spin_until(exec, [&]() {return pub->get_subscription_count() > 0;}, 2s));

  pub->publish(make_scan({10.0f, 10.0f, 10.0f}));

  ASSERT_TRUE(spin_until(exec, [&]() {
      return received_future.wait_for(0ms) == std::future_status::ready;
                                                                                                        },
    2s));

  const auto out = received_future.get();
  for (const auto & r : out.ranges) {
    EXPECT_NEAR(r, 0.12f, 1e-6f);
  }

  exec.remove_node(harness);
  exec.remove_node(bridge);
}

TEST_F(ScanModeBridgeTest, BlocksDistanceClampedToRangeMax)
{
  auto bridge_options = rclcpp::NodeOptions().arguments({
    "--ros-args",
    "-p",
    "dist_blocked:=50.0",
    "-r",
    "scan_raw:=test_scan",
    "-r",
    "scan_bridged:=test_scan_bridged",
    "-r",
    "trigger_mode:=test_trigger_mode",
  });

  auto bridge = std::make_shared<easynav_experiments::ScanModeBridge>(bridge_options);
  auto harness = std::make_shared<rclcpp::Node>("harness_blocks_clamped_distance");

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(bridge);
  exec.add_node(harness);

  auto client = harness->create_client<std_srvs::srv::Trigger>("test_trigger_mode");
  ASSERT_TRUE(client->wait_for_service(2s));

  // Switch to BLOCKED
  {
    auto future = client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    ASSERT_EQ(exec.spin_until_future_complete(future, 2s), rclcpp::FutureReturnCode::SUCCESS);
  }

  std::promise<sensor_msgs::msg::LaserScan> received_promise;
  auto received_future = received_promise.get_future();
  auto sub = harness->create_subscription<sensor_msgs::msg::LaserScan>(
      "test_scan_bridged", rclcpp::SensorDataQoS(),
    [&received_promise](sensor_msgs::msg::LaserScan::SharedPtr msg) {
      if (msg) {
        received_promise.set_value(*msg);
      }
      });

  auto pub = harness->create_publisher<sensor_msgs::msg::LaserScan>("test_scan",
    rclcpp::SensorDataQoS());
  ASSERT_TRUE(spin_until(exec, [&]() {return pub->get_subscription_count() > 0;}, 2s));

  auto input = make_scan({1.0f, 2.0f});
  input.range_max = 0.3f;
  pub->publish(input);

  ASSERT_TRUE(spin_until(exec, [&]() {
      return received_future.wait_for(0ms) == std::future_status::ready;
                                                                                                        },
    2s));

  const auto out = received_future.get();
  for (const auto & r : out.ranges) {
    EXPECT_NEAR(r, 0.3f, 1e-6f);
  }

  exec.remove_node(harness);
  exec.remove_node(bridge);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
