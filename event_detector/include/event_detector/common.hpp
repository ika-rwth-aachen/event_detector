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

#pragma once

#include <cctype>
#include <string>
#include <tuple>
#include <type_traits>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>

#include "event_detector/datatypes.hpp"

namespace event_detector {

// constants
const std::string kTransformationTopic = "/tf";               ///< topic for dynamic transforms
const std::string kStaticTransformationTopic = "/tf_static";  ///< topic for static transforms

/**
 * @brief Struct wrapping an object with a timestamp.
 *
 * @tparam  T  object datatype
 */
template <typename T>
struct Stamped {
  rclcpp::Time stamp;            ///< timestamp
  std::shared_ptr<const T> msg;  ///< ROS message
};

// convenience data structure typedefs
template <typename T>
using string_map = std::unordered_map<std::string, T>;
template <typename T>
using string_map_2d = std::unordered_map<std::string, string_map<T>>;

/**
 * @brief Struct containing all data topics from a connected client
 */
struct ClientConnection {
  std::string id;          ///< client ID
  std::string name;        ///< client name
  std::string base_frame;  ///< name of base frame
  std::string tf_prefix;   ///< prefix of frames
#define DATATYPE(TYPE, VAR) \
  std::vector<std::tuple<std::string, double, int>> VAR;  ///< TYPE topics with buffer time and subscriber queue size
#include "event_detector/datatypes.macro"
#undef DATATYPE
};

// definitions for distinguishing between ROS messages with/without header
// based on SFINAE (https://fekir.info/post/detect-member-variables/)
using HasRosHeaderNo = std::false_type;
using HasRosHeaderYes = std::true_type;

template <typename T, typename = std_msgs::msg::Header>
struct HasRosHeader : HasRosHeaderNo {};

template <typename T>
struct HasRosHeader<T, decltype(T::header)> : HasRosHeaderYes {};

template <typename T>
bool hasRosHeader(const T&);

template <typename T>
HasRosHeader<T> getHasRosHeader(const T&);

}  // namespace event_detector

#include "event_detector/common.tpp"