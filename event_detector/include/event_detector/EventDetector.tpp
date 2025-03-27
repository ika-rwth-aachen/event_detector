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

namespace event_detector {

template <typename T>
void EventDetector::insertPassthrough(const std::shared_ptr<const T>& sample, const std::string& client_id,
                                      const std::string& topic, std::optional<rclcpp::Time> stamp_override) {
  std::shared_lock lock{reconfigure_mutex_};

  rclcpp::Time stamp = stamp_override.value_or(this->getSampleStamp(*sample));

#define DATATYPE(TYPE, VAR)                                       \
  if constexpr (std::is_same_v<T, TYPE>) {                        \
    buffer_manager_->VAR.insert(sample, stamp, client_id, topic); \
  }
#include "event_detector/datatypes.macro"
#undef DATATYPE
}

template <typename T>
rclcpp::Time EventDetector::getSampleStamp(const T& sample) {
  if constexpr (HasRosHeader<T>::value) {
    if (buffer_config_.use_msg_stamp) return sample.header.stamp;
  }
  return this->now();
}

}  // namespace event_detector
