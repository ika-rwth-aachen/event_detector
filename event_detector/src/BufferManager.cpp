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

#include <sstream>
#include <stdexcept>

#include "event_detector/BufferManager.hpp"

namespace event_detector {

BufferManager::BufferManager() {
#define DATATYPE(TYPE, VAR) VAR.setBufferManager(this);
#include "event_detector/datatypes.macro"
#undef DATATYPE
}

void BufferManager::lockBuffers(const bool is_write) {
  std::unique_lock<std::mutex> lock(buffer_lock_mutex_);
  if (is_write) {
    buffer_lock_condition_variable_.wait(lock, [&] { return buffer_lock_n_threads_ >= 0; });
    buffer_lock_n_threads_++;
    RCLCPP_DEBUG(rclcpp::get_logger("event_detector"), "lockBuffers(is_write=true): buffer_lock_n_threads_ = %d -> %d",
                 buffer_lock_n_threads_ - 1, buffer_lock_n_threads_);
  } else {
    buffer_lock_condition_variable_.wait(lock, [&] { return buffer_lock_n_threads_ <= 0; });
    buffer_lock_n_threads_--;
    RCLCPP_DEBUG(rclcpp::get_logger("event_detector"), "lockBuffers(is_write=false): buffer_lock_n_threads_ = %d -> %d",
                 buffer_lock_n_threads_ + 1, buffer_lock_n_threads_);
  }
  lock.unlock();
}

void BufferManager::unlockBuffers(const bool is_write) {
  std::unique_lock<std::mutex> lock(buffer_lock_mutex_);
  if (is_write) {
    buffer_lock_n_threads_--;
    RCLCPP_DEBUG(rclcpp::get_logger("event_detector"),
                 "unlockBuffers(is_write=true): buffer_lock_n_threads_ = %d -> %d", buffer_lock_n_threads_ + 1,
                 buffer_lock_n_threads_);
  } else {
    buffer_lock_n_threads_++;
    RCLCPP_DEBUG(rclcpp::get_logger("event_detector"),
                 "unlockBuffers(is_write=false): buffer_lock_n_threads_ = %d -> %d", buffer_lock_n_threads_ - 1,
                 buffer_lock_n_threads_);
  }
  lock.unlock();
  buffer_lock_condition_variable_.notify_all();
}

std::string BufferManager::getInfo() const {
  std::stringstream ss;
  ss << std::endl << "Buffer state:" << std::endl;
#define DATATYPE(TYPE, VAR)                              \
  ss << std::string(2, ' ') << #VAR << ":" << std::endl; \
  ss << VAR.getInfo();
#include "event_detector/datatypes.macro"
#undef DATATYPE
  return ss.str();
}

}  // namespace event_detector