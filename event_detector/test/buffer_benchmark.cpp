/** ============================================================================
MIT License

Copyright (c) 2025 Institute for Automotive Engineering (ika), RWTH Aachen University

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
============================================================================= */

#include <random>

#include <benchmark/benchmark.h>
#include <rosbag2_cpp/reader.hpp>

#include "event_detector/BufferManager.hpp"

sensor_msgs::msg::PointCloud2 readSamplePointcloud() {
  rclcpp::get_logger("rosbag2_storage").set_level(rclcpp::Logger::Level::Fatal);
  rosbag2_cpp::Reader reader;
  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serialization;
  reader.open("/docker-ros/ws/src/target/event_detector/test/sample_pointcloud");

  // openend sample file has exactly 1 message
  auto msg = reader.read_next();
  rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);
  sensor_msgs::msg::PointCloud2 deserialized_msg;
  serialization.deserialize_message(&serialized_msg, &deserialized_msg);

  return deserialized_msg;
}

static void insert_pointclouds(benchmark::State& state) {
  sensor_msgs::msg::PointCloud2 msg = readSamplePointcloud();

  std::vector<sensor_msgs::msg::PointCloud2::SharedPtr> messages;
  for (int i = 0; i < state.range(0); ++i) {
    msg.header.stamp = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    messages.push_back(std::make_shared<sensor_msgs::msg::PointCloud2>(msg));
  }

  for (auto _ : state) {
    event_detector::BufferManager manager;

    event_detector::ClientConnection conn1;
    conn1.id = "client";
    conn1.sensor_msgs__PointCloud2.push_back({"topic", 30.0, 100});
    manager.sensor_msgs__PointCloud2.update({conn1});

    for (auto& m : messages) {
      manager.sensor_msgs__PointCloud2.insert(m, m->header.stamp, "client", "topic");
    }
  }
}

static void insert_poses(benchmark::State& state) {
  std::vector<geometry_msgs::msg::PoseStamped::SharedPtr> messages;
  for (int i = 0; i < state.range(0); ++i) {
    messages.push_back(std::make_shared<geometry_msgs::msg::PoseStamped>());
    messages.back()->header.stamp = rclcpp::Clock(RCL_SYSTEM_TIME).now();
  }

  for (auto _ : state) {
    event_detector::BufferManager manager;

    event_detector::ClientConnection conn1;
    conn1.id = "client";
    conn1.geometry_msgs__PoseStamped.push_back({"topic", 30.0, 100});
    manager.geometry_msgs__PoseStamped.update({conn1});

    for (auto& m : messages) {
      manager.geometry_msgs__PoseStamped.insert(m, m->header.stamp, "client", "topic");
    }
  }
}

static void retrieve_poses_from_buffer(benchmark::State& state) {
  // 30 seconds of data with 100 Hz
  event_detector::BufferManager manager;
  event_detector::ClientConnection conn1;
  conn1.id = "client";
  conn1.geometry_msgs__PoseStamped.push_back({"topic", 30.0, 100});
  manager.geometry_msgs__PoseStamped.update({conn1});
  for (int i = 0; i < 3000; ++i) {
    auto msg = std::make_shared<geometry_msgs::msg::PoseStamped>();
    msg->header.stamp = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    manager.geometry_msgs__PoseStamped.insert(msg, msg->header.stamp, "client", "topic");
  }

  for (auto _ : state) {
    benchmark::DoNotOptimize(manager.geometry_msgs__PoseStamped.get(state.range(0), "", ""));
  }
}

static void retrieve_pointclouds_from_buffer(benchmark::State& state) {
  sensor_msgs::msg::PointCloud2 pcl = readSamplePointcloud();

  // 30 seconds of data with 10 Hz
  event_detector::BufferManager manager;
  event_detector::ClientConnection conn1;
  conn1.id = "client";
  conn1.sensor_msgs__PointCloud2.push_back({"topic", 30.0, 100});
  manager.sensor_msgs__PointCloud2.update({conn1});
  for (int i = 0; i < 300; ++i) {
    auto msg = std::make_shared<sensor_msgs::msg::PointCloud2>(pcl);
    msg->header.stamp = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    manager.sensor_msgs__PointCloud2.insert(msg, msg->header.stamp, "client", "topic");
  }

  for (auto _ : state) {
    benchmark::DoNotOptimize(manager.sensor_msgs__PointCloud2.get(state.range(0), "", ""));
  }
}

BENCHMARK(insert_poses)->RangeMultiplier(2)->Range(1, 1 << 10);
BENCHMARK(insert_pointclouds)->RangeMultiplier(2)->Range(1, 1 << 10);
BENCHMARK(retrieve_pointclouds_from_buffer)->RangeMultiplier(2)->Range(1, 1 << 10);
BENCHMARK(retrieve_poses_from_buffer)->RangeMultiplier(2)->Range(1, 1 << 10);

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);  // to use rclcpp::Clock
  ::benchmark::Initialize(&argc, argv);
  ::benchmark::RunSpecifiedBenchmarks();
}