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

#include <iostream>
#include <vector>

#include <gtest/gtest.h>

#include "event_detector/BufferManager.hpp"

TEST(event_detector, buffer_order_test) {
  event_detector::BufferManager manager;

  std::vector<std::shared_ptr<geometry_msgs::msg::PoseStamped>> messages;
  for (int i = 0; i < 30; ++i) {
    messages.push_back(std::make_shared<geometry_msgs::msg::PoseStamped>());
    messages.back()->header.stamp.nanosec = i;
  }
  std::random_shuffle(messages.begin(), messages.end());

  event_detector::ClientConnection conn1;
  conn1.id = "test_client_odd";
  conn1.geometry_msgs__PoseStamped.push_back({"test_topic_odd", 1000.0, 100});

  event_detector::ClientConnection conn2;
  conn2.id = "test_client_even";
  conn2.geometry_msgs__PoseStamped.push_back({"test_topic_even", 1000.0, 100});

  manager.geometry_msgs__PoseStamped.update({conn1, conn2});

  for (auto& m : messages) {
    manager.geometry_msgs__PoseStamped.insert(m, m->header.stamp, (m->header.stamp.nanosec % 2) ? "test_client_odd" : "test_client_even",
                        (m->header.stamp.nanosec % 2) ? "test_topic_odd" : "test_topic_even", false);
  }

  {
    // all clients
    auto result = manager.geometry_msgs__PoseStamped.get("", "");
    ASSERT_EQ(result.size(), 30u);
    for (unsigned int i = 0; i < 30u; ++i) {
      ASSERT_EQ(result[i].msg->header.stamp.nanosec, i);
    }
  }
  {
    // only even_client
    auto result = manager.geometry_msgs__PoseStamped.get("test_client_even", "");
    ASSERT_EQ(result.size(), 15u);
    for (unsigned int i = 0; i < 15u; ++i) {
      ASSERT_EQ(result[i].msg->header.stamp.nanosec, i * 2);
    }
  }
  {
    auto result = manager.geometry_msgs__PoseStamped.get(3, rclcpp::Time(0, 11, RCL_ROS_TIME), rclcpp::Time(0, 15, RCL_ROS_TIME),
                                   "test_client_odd", "");
    ASSERT_EQ(result.size(), 3u);
    ASSERT_EQ(result[0].msg->header.stamp.nanosec, 11u);
    ASSERT_EQ(result[1].msg->header.stamp.nanosec, 13u);
    ASSERT_EQ(result[2].msg->header.stamp.nanosec, 15u);
  }
  {
    auto result = manager.geometry_msgs__PoseStamped.get(2, rclcpp::Time(0, 11, RCL_ROS_TIME), rclcpp::Time(0, 15, RCL_ROS_TIME),
                                   "test_client_odd", "");
    ASSERT_EQ(result.size(), 2u);
    ASSERT_EQ(result[0].msg->header.stamp.nanosec, 13u);
    ASSERT_EQ(result[1].msg->header.stamp.nanosec, 15u);
  }
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);  // to use rclcpp::Clock
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}