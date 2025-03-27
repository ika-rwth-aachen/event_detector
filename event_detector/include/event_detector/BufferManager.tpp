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

#include <algorithm>
#include <functional>
#include <numeric>

namespace event_detector {

/**
 * @brief Returns the requested timestamp, if available.
 *
 * If `use_msg_stamp`, returns the ROS message stamp for messages with header.
 * Always returns the associated time-of-arrival for messages without header.
 *
 * @tparam  T  datatype
 *
 * @param   stamped_sample  stamped data sample
 * @param   use_msg_stamp   whether to return the message stamp
 *
 * @return  const rclcpp::Time&  timestamp
 */
template <typename T>
inline const rclcpp::Time getSampleStamp(const Stamped<T>& stamped_sample, const bool use_msg_stamp) {
  return getSampleStamp(stamped_sample, use_msg_stamp, getHasRosHeader(*(stamped_sample.msg)));
}

/**
 * @brief Returns the requested timestamp for ROS messages with header.
 *
 * @tparam  T  datatype
 *
 * @param   stamped_sample  stamped data sample
 * @param   use_msg_stamp   whether to return the message stamp
 *
 * @return  const rclcpp::Time&  timestamp
 */
template <typename T>
inline const rclcpp::Time getSampleStamp(const Stamped<T>& stamped_sample, const bool use_msg_stamp,
                                         const HasRosHeaderYes&) {
  if (use_msg_stamp) {
    rclcpp::Time time_stamp(stamped_sample.msg->header.stamp);
    return time_stamp;
  } else {
    return stamped_sample.stamp;
  }
}

/**
 * @brief Returns the associated time-of-arrival for ROS messages without header.
 *
 * @tparam  T  datatype
 *
 * @param   stamped_sample  stamped data sample
 * @param   use_msg_stamp   whether to return the message stamp (dummy)
 *
 * @return  const rclcpp::Time&  timestamp
 */
template <typename T>
inline const rclcpp::Time getSampleStamp(const Stamped<T>& stamped_sample, const bool use_msg_stamp,
                                         const HasRosHeaderNo&) {
  return stamped_sample.stamp;
}

template <typename T>
BufferManager::DataBuffers<T>::DataBuffers() {}

template <typename T>
void BufferManager::DataBuffers<T>::setBufferManager(BufferManager* manager) {
  std::unique_lock lock{reconfigure_mutex_};
  manager_ = manager;
}

template <typename T>
void BufferManager::DataBuffers<T>::update(const std::vector<ClientConnection>& clients) {
  std::unique_lock lock{reconfigure_mutex_};

#define DATATYPE(TYPE, VAR)                                                                        \
  if constexpr (std::is_same_v<T, TYPE>) {                                                         \
    string_map_2d<Buffer<std::shared_ptr<const T>>> new_buffers;                                   \
    string_map_2d<std::shared_ptr<std::shared_mutex>> new_mutexes;                                 \
                                                                                                   \
    /* Reserve capacity for clients */                                                             \
    new_buffers.reserve(clients.size());                                                           \
    new_mutexes.reserve(clients.size());                                                           \
    for (const auto& client : clients) {                                                           \
      for (const auto& [topic, duration, queue_size] : client.VAR) {                               \
        auto& client_buffers = buffer_by_client_topic_[client.id];                                 \
        auto buffer_it = client_buffers.find(topic);                                               \
        new_buffers[client.id][topic] = (buffer_it != client_buffers.end())                        \
                                            ? std::move(buffer_it->second)                         \
                                            : Buffer<std::shared_ptr<const T>>();                  \
        new_buffers[client.id][topic].setDifference(rclcpp::Duration::from_seconds(duration));     \
        auto& client_mutexes = mutex_by_client_topic_[client.id];                                  \
        auto mutex_it = client_mutexes.find(topic);                                                \
        new_mutexes[client.id][topic] = (mutex_it != client_mutexes.end())                         \
                                            ? std::move(mutex_it->second)                          \
                                            : std::make_shared<std::shared_mutex>();               \
      }                                                                                            \
    }                                                                                              \
    buffer_by_client_topic_ = std::move(new_buffers);                                              \
    mutex_by_client_topic_ = std::move(new_mutexes);                                               \
  }
#include "event_detector/datatypes.macro"
#undef DATATYPE
}

template <typename T>
bool BufferManager::DataBuffers<T>::timeIsRunningBackwards(const std::string& client_id, const std::string& topic,
                                                           const rclcpp::Time& stamp) {
  // initialize last_stamp_by_client_topic_
  if (last_stamp_by_client_topic_.find(client_id) == last_stamp_by_client_topic_.end() || last_stamp_by_client_topic_[client_id].find(topic) == last_stamp_by_client_topic_[client_id].end()) {
    last_stamp_by_client_topic_[client_id][topic] = rclcpp::Time(0, 0, stamp.get_clock_type());
  }

  // check
  auto& last_stamp = last_stamp_by_client_topic_[client_id][topic];
  bool is_running_backwards = (stamp < last_stamp);
  last_stamp_by_client_topic_[client_id][topic] = stamp;

  return is_running_backwards;
}

template <typename T>
void BufferManager::DataBuffers<T>::insert(const typename std::shared_ptr<const T>& sample, const rclcpp::Time& stamp,
                                           const std::string& client_id, const std::string& topic,
                                           const bool clear_if_time_is_running_backwards) {
  std::shared_lock lock{reconfigure_mutex_};

  if (clear_if_time_is_running_backwards && this->timeIsRunningBackwards(client_id, topic, stamp)) {
    RCLCPP_WARN(rclcpp::get_logger("event_detector"),
                "Time running backwards for topic '%s', clearing buffer of client '%s' for this topic", topic.c_str(),
                client_id.c_str());
    this->clearBuffer(client_id, topic);
  }

  RCLCPP_DEBUG(rclcpp::get_logger("event_detector"), "BufferManager::DataBuffers<T>::insert started for topic '%s'",
               topic.c_str());

  // insert
  auto& buffer_by_topic = buffer_by_client_topic_.at(client_id);
  auto& mutex_by_topic = mutex_by_client_topic_.at(client_id);
  if (buffer_by_topic.find(topic) == buffer_by_topic.end() || mutex_by_topic.find(topic) == mutex_by_topic.end()) {
    RCLCPP_ERROR(rclcpp::get_logger("event_detector"), "Buffer for topic '%s' has not yet been initialized",
                 topic.c_str());
    return;
  }

  {                                                                    /* thread-safe append to buffer */
    manager_->lockBuffers(true);                                       // lock reading from all buffers
    std::unique_lock<std::shared_mutex> lock(*mutex_by_topic[topic]);  // lock topic-specific buffer

    // determine position
    auto it = buffer_by_topic[topic].rbegin();
    while (it != buffer_by_topic[topic].rend() && stamp < it->first) {
      ++it;
    }

    // insert
    buffer_by_topic[topic].insert(it.base(), sample, stamp);

    manager_->unlockBuffers(true);  // unlock reading from all buffers
  }

  if constexpr (HasRosHeader<T>::value) {
    // store frame information, if not yet available
    auto& frame_by_topic = frame_by_client_topic_[client_id];
    if (frame_by_topic.find(topic) == frame_by_topic.end()) frame_by_topic[topic] = sample->header.frame_id;
  }

  RCLCPP_DEBUG(rclcpp::get_logger("event_detector"), "BufferManager::DataBuffers<T>::insert finished for topic '%s'",
               topic.c_str());
}

template <typename T>
std::vector<std::string> BufferManager::DataBuffers<T>::getClients() {
  std::shared_lock lock{reconfigure_mutex_};

  std::vector<std::string> client_ids;
  for (const auto& kv : buffer_by_client_topic_) client_ids.push_back(kv.first);

  return client_ids;
}

template <typename T>
std::vector<std::string> BufferManager::DataBuffers<T>::getFrames(const std::string& client_id) {
  std::shared_lock lock{reconfigure_mutex_};

  std::vector<std::string> frame_ids;
  if (frame_by_client_topic_.find(client_id) == frame_by_client_topic_.end()) return frame_ids;
  for (const auto& kv : frame_by_client_topic_[client_id]) frame_ids.push_back(kv.second);

  return frame_ids;
}

template <typename T>
std::vector<Stamped<T>> BufferManager::DataBuffers<T>::get(const std::string& client_id, const std::string& frame_id,
                                                           const std::string& topic) {
  return this->get(-1, client_id, frame_id, topic);
}

template <typename T>
std::vector<Stamped<T>> BufferManager::DataBuffers<T>::get(const int k, const std::string& client_id,
                                                           const std::string& frame_id, const std::string& topic) {
  rclcpp::Time zero(0, 0, RCL_ROS_TIME);
  return this->get(k, zero, zero, client_id, frame_id, topic);
}

template <typename T>
std::vector<Stamped<T>> BufferManager::DataBuffers<T>::get(const rclcpp::Time& start_time, const rclcpp::Time& end_time,
                                                           const std::string& client_id, const std::string& frame_id,
                                                           const std::string& topic) {
  return this->get(-1, start_time, end_time, client_id, frame_id, topic);
}

template <typename T>
std::vector<Stamped<T>> BufferManager::DataBuffers<T>::get(const int k, const rclcpp::Time& start_time,
                                                           const rclcpp::Time& end_time, const std::string& client_id,
                                                           const std::string& frame_id, const std::string& topic) {
  // merge samples by topic
  string_map<std::vector<Stamped<T>>> samples_by_topic =
      this->getWithTopic(k, start_time, end_time, client_id, frame_id, topic);
  if (samples_by_topic.size() == 1) {
    return samples_by_topic.begin()->second;
  }

  std::vector<Stamped<T>> samples;
  for (auto& kv : samples_by_topic) {
    for (auto& sample : kv.second) samples.push_back(sample);
  }

  // sort all samples by stamp
  auto comp = [](const Stamped<T>& s1, const Stamped<T>& s2) {
    return s1.stamp < s2.stamp;
  };
  std::sort(samples.begin(), samples.end(), comp);

  return samples;
}

template <typename T>
string_map<std::vector<Stamped<T>>> BufferManager::DataBuffers<T>::getWithTopic(
    const int k, const rclcpp::Time& start_time, const rclcpp::Time& end_time, const std::string& client_id,
    const std::string& frame_id, const std::string& topic) {
  RCLCPP_DEBUG(rclcpp::get_logger("event_detector"), "BufferManager::DataBuffers<T>::getWithTopic started");
  std::shared_lock lock{reconfigure_mutex_};

  string_map<std::vector<Stamped<T>>> samples_by_topic;

  // return data from all clients/frames if no ID is given
  bool from_all_clients = client_id.empty();
  bool from_all_frames = frame_id.empty();

  // find clients to iterate over
  auto client_it_1 = buffer_by_client_topic_.begin();
  auto client_it_2 = buffer_by_client_topic_.end();
  if (!from_all_clients) {
    client_it_1 = buffer_by_client_topic_.find(client_id);
    if (client_it_1 != client_it_2) client_it_2 = std::next(client_it_1);
  }

  // loop over clients
  for (auto& v_it = client_it_1; v_it != client_it_2; v_it++) {
    const std::string& v_id = v_it->first;

    // loop over topics
    for (const auto& kv : buffer_by_client_topic_[v_id]) {
      const std::string& current_topic = kv.first;
      if (topic != "" && current_topic != topic) continue;
      const auto& buffer = kv.second;
      const std::string& current_frame_id = frame_by_client_topic_[v_id][current_topic];

      // handle negative k
      const int n = (k >= 0) ? k : buffer.size();

      // check frame ID
      if (from_all_frames || current_frame_id == frame_id) {
        /* thread-safe get from buffer */
        // lock topic-specific buffer
        std::shared_lock<std::shared_mutex> lock(*mutex_by_client_topic_[v_id][current_topic]);

        // find last sample in time range
        auto rbegin_it = buffer.rbegin();
        while (rbegin_it != buffer.rend() && (end_time.nanoseconds() != 0) && rbegin_it->first > end_time) {
          ++rbegin_it;
        }

        // find first sample in time range
        auto rend_it = rbegin_it;
        while (rend_it != buffer.rend() && (rend_it->first >= start_time || (end_time.nanoseconds() == 0)) &&
               rend_it - rbegin_it < n) {
          ++rend_it;
        }

        // copy samples from first to last
        for (auto it = rend_it.base(); it != rbegin_it.base(); ++it) {
          samples_by_topic[current_topic].push_back(Stamped<T>{it->first, it->second});
        }
      }
    }
  }
  RCLCPP_DEBUG(rclcpp::get_logger("event_detector"), "BufferManager::DataBuffers<T>::getWithTopic finished");

  return samples_by_topic;
}

template <typename T>
std::string BufferManager::DataBuffers<T>::getInfo() const {
  std::shared_lock lock{reconfigure_mutex_};

  std::stringstream ss;
  for (const auto& [client, buffer_by_topic] : buffer_by_client_topic_) {
    ss << std::string(4, ' ') << client << ":" << std::endl;
    for (const auto& [topic, buffer] : buffer_by_topic) {
      ss << std::string(6, ' ') << topic << ": " << buffer.size() << " samples" << std::endl;
    }
  }
  return ss.str();
}

template <typename T>
void BufferManager::DataBuffers<T>::clearBuffer(const std::string& client_id, const std::string& topic) {
  auto& buffer_by_topic = buffer_by_client_topic_.at(client_id);
  auto& mutex_by_topic = mutex_by_client_topic_.at(client_id);
  if (buffer_by_topic.find(topic) == buffer_by_topic.end() || mutex_by_topic.find(topic) == mutex_by_topic.end()) {
    RCLCPP_ERROR(rclcpp::get_logger("event_detector"), "Buffer for topic '%s' has not yet been initialized",
                 topic.c_str());
    return;
  }

  // clear
  {                                                                    /* thread-safe append to buffer */
    manager_->lockBuffers(true);                                       // lock reading from all buffers
    std::unique_lock<std::shared_mutex> lock(*mutex_by_topic[topic]);  // lock topic-specific buffer

    buffer_by_client_topic_[client_id][topic].clear();
    last_stamp_by_client_topic_[client_id].erase(topic);

    manager_->unlockBuffers(true);  // unlock reading from all buffers
  }
}

}  // namespace event_detector