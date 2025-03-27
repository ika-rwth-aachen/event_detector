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

#include <condition_variable>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <vector>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <rclcpp/rclcpp.hpp>

#include "event_detector/DifferenceBasedBuffer.hpp"
#include "event_detector/common.hpp"

namespace event_detector {

template <typename T>
using Buffer = DifferenceBasedBuffer<T, rclcpp::Time, rclcpp::Duration>;

/**
 * @brief Component of the Event Detector responsible for buffering live data
 *
 * As one of the three components of the Event Detector, the BufferManager
 * buffers all incoming data streams for a certain amount of time. Time is
 * evaluated on a milli-second level. In order to access the buffer, its members
 * provide various getter functions.
 */
class BufferManager {
 public:
  /**
   * @brief Template class designed to hold all buffers for a specific data type
   *
   * An instance of this class manages the buffers for multiple clients /
   * frames for a specific data type. It provides multiple generic getter
   * functions.
   *
   * @tparam  T  ROS data type to buffer
   */
  template <typename T>
  class DataBuffers {
   public:
    /**
     * @brief Creates a new DataBuffers instance.
     */
    DataBuffers();

    /**
     * @brief Assigns the managing BufferManager.
     *
     * @param  manager  BufferManager
     */
    void setBufferManager(BufferManager* manager);

    /**
     * @brief Creates buffers and mutexes for topics mentioned in clients.
     * If a buffer and mutex already exists for a topic, the old buffer and
     * mutex will be retained and no new buffer and mutex will be created.
     * Existing mutexes and buffers for topics that are not mentioned in
     * clients will be deleted.
     *
     * @param clients client connections for which to set up buffers
     */
    void update(const std::vector<ClientConnection>& clients);

    /**
     * @brief Inserts a new ROS message into the buffer.
     *
     * May be used as a ROS subscription callback. Buffer has to be initialized
     * before first insertion. Supports ROS messages with or without header.
     *
     * @param   sample       new data message
     * @param   stamp        ROS time stamp associated with the message
     * @param   client_id    client ID
     * @param   topic        ROS topic
     * @param   clear_if_time_is_running_backwards  whether to clear the buffer if time is running backwards
     */
    void insert(const typename std::shared_ptr<const T>& sample, const rclcpp::Time& stamp,
                const std::string& client_id, const std::string& topic,
                const bool clear_if_time_is_running_backwards = true);

    /**
     * @brief Gets the clients IDs that have buffered data.
     *
     * @return  std::vector<std::string>  client IDs
     */
    std::vector<std::string> getClients();

    /**
     * @brief Gets the frame IDs for a specific client that has buffered data.
     *
     * @param   client_id  client ID
     *
     * @return  std::vector<std::string>  frame IDs
     */
    std::vector<std::string> getFrames(const std::string& client_id);

    /**
     * @brief Fetches all buffer samples.
     *
     * If no client or frame ID is given, data from all clients/frames is
     * returned.
     *
     * The returned data samples are stamped and sorted in ascending order.
     *
     * @param   client_id      client ID (optional)
     * @param   frame_id       frame ID (optional)
     * @param   topic          topic (optional)
     *
     * @return  std::vector<Stamped<T>>  data samples
     */
    std::vector<Stamped<T>> get(const std::string& client_id = "", const std::string& frame_id = "",
                                const std::string& topic = "");

    /**
     * @brief Fetches most recent k buffer samples.
     *
     * If `k` is negative, no limit on the number of samples is imposed.
     *
     * If no client or frame ID is given, data from all clients/frames is
     * returned.
     *
     * The returned data samples are stamped and sorted in ascending order.
     *
     * @param   k              how many recent samples to return
     * @param   client_id      client ID (optional)
     * @param   frame_id       frame ID (optional)
     * @param   topic          topic (optional)
     *
     * @return  std::vector<Stamped<T>>  data samples
     */
    std::vector<Stamped<T>> get(const int k, const std::string& client_id = "", const std::string& frame_id = "",
                                const std::string& topic = "");

    /**
     * @brief Fetches all buffer samples within specified timeframe.
     *
     * If `end_time` is 0, time filter is not used.
     *
     * If no client or frame ID is given, data from all clients/frames is
     * returned.
     *
     * The returned data samples are stamped and sorted in ascending order.
     *
     * @param   start_time     starting time filter
     * @param   end_time       ending time filter
     * @param   client_id      client ID (optional)
     * @param   frame_id       frame ID (optional)
     * @param   topic          topic (optional)
     *
     * @return  std::vector<Stamped<T>>  data samples
     */
    std::vector<Stamped<T>> get(const rclcpp::Time& start_time, const rclcpp::Time& end_time,
                                const std::string& client_id = "", const std::string& frame_id = "",
                                const std::string& topic = "");

    /**
     * @brief Fetches most recent k buffer samples within specified timeframe.
     *
     * If `end_time` is 0, time filter is not used. If `k` is negative, no limit
     * on the number of samples is imposed.
     *
     * If no client or frame ID is given, data from all clients/frames is
     * returned.
     *
     * The returned data samples are stamped and sorted in ascending order.
     *
     * @param   k              how many recent samples to return
     * @param   start_time     starting time filter
     * @param   end_time       ending time filter
     * @param   client_id      client ID (optional)
     * @param   frame_id       frame ID (optional)
     * @param   topic          topic (optional)
     *
     * @return  std::vector<Stamped<T>>  data samples
     */
    std::vector<Stamped<T>> get(const int k, const rclcpp::Time& start_time, const rclcpp::Time& end_time,
                                const std::string& client_id = "", const std::string& frame_id = "",
                                const std::string& topic = "");

    /**
     * @brief Same as corresponding `get`, but returns samples grouped by topic.
     *
     * @param   start_time     starting time filter
     * @param   end_time       ending time filter
     * @param   client_id      client ID (optional)
     * @param   frame_id       frame ID (optional)
     * @param   topic          topic (optional)
     *
     * @return  string_map<std::vector<Stamped<T>>>  data samples by topic
     */
    string_map<std::vector<Stamped<T>>> getWithTopic(const int k, const rclcpp::Time& start_time,
                                                     const rclcpp::Time& end_time, const std::string& client_id = "",
                                                     const std::string& frame_id = "", const std::string& topic = "");

    /**
     * @brief Returns a string representation of the buffers that are currently present
     *
     * @return std::string string representation of the buffers
    */
    std::string getInfo() const;

    /**
     * @brief Clears buffer for a specific client and topic.
     *
     * @param client_id client ID
     * @param topic topic
     */
    void clearBuffer(const std::string& client_id, const std::string& topic);

    /**
     * @brief Checks whether time is running backwards for a specific client and topic.
     *
     * @param client_id client ID
     * @param topic topic
     * @param stamp stamp to compare with the last known time stamp
     *
     * @return bool whether time is running backwards
     */
    bool timeIsRunningBackwards(const std::string& client_id, const std::string& topic, const rclcpp::Time& stamp);

   protected:
    /**
     * @brief managing BufferManager
     */
    BufferManager* manager_ = nullptr;

    /**
     * @brief container for data buffers, stored by client ID and topic
     */
    string_map_2d<Buffer<std::shared_ptr<const T>>> buffer_by_client_topic_;

    /**
     * @brief container to store frame ID by client ID and topic
     */
    string_map_2d<std::string> frame_by_client_topic_;

    /**
     * @brief container to store the last time stamp for each client and topic
     */
    string_map_2d<rclcpp::Time> last_stamp_by_client_topic_;

    /**
     * @brief container to store mutexes for locking buffer access
     */
    string_map_2d<std::shared_ptr<std::shared_mutex>> mutex_by_client_topic_;

    /**
     * mutex for reconfiguration
    */
    mutable std::shared_mutex reconfigure_mutex_;
  };

 public:
  /**
   * @brief Creates a new BufferManager.
   */
  BufferManager();

  /**
   * @brief Locks all buffers for either write or read mode.
   *
   * If all buffers are locked in write mode, then a call to this function
   * in read mode will block. Calls to this function in write mode will
   * continue. This allows to run multiple threads in parallel, but only if they
   * are of the same mode, i.e. writing/reading from buffers.
   * The same behavior applies the other way round.
   *
   * @param  is_write  write/read mode
   */
  void lockBuffers(const bool is_write);

  /**
   * @brief Unlocks all buffers for either write or read mode.
   *
   * Buffers stay locked in write or read mode, as long as there are other
   * threads within the same mode that have not yet unlocked the buffers.
   *
   * @param  is_write  write/read mode
   */
  void unlockBuffers(const bool is_write);

  /**
   * @brief Returns a string representation of all buffers that are currently present
   *
   * @return std::string string representation of the buffers
  */
  std::string getInfo() const;

 public:
  /**
   * @brief mutex for locking all buffer access
   */
  mutable std::mutex buffer_lock_mutex_;

  /**
   * @brief condition variable to lock write/read buffer access
   */
  std::condition_variable buffer_lock_condition_variable_;

  /**
   * @brief helper variable to count threads in write/read buffer access mode
   */
  int buffer_lock_n_threads_ = 0;

#define DATATYPE(TYPE, VAR)    \
  /** data buffers for TYPE */ \
  DataBuffers<TYPE> VAR;
#include "event_detector/datatypes.macro"
#undef DATATYPE
};

}  // namespace event_detector

#include "event_detector/BufferManager.tpp"