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

#include <map>
#include <memory>
#include <optional>
#include <string>

#include <tf2_ros/transform_listener.h>
#include <lifecycle_msgs/msg/transition.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/time_source.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/writer.hpp>

#include "event_detector/AnalysisManager.hpp"
#include "event_detector/BufferManager.hpp"

namespace event_detector {

class EventDetector;

class EventDetector : public rclcpp_lifecycle::LifecycleNode {
 public:
  /**
   * @brief Constructor getting its options e.g. from ComposableNodeContainer
   *
   * @param options NodeOptions
   */
  explicit EventDetector(rclcpp::NodeOptions options);

  /**
   * @brief Returns client connection details
   *
   * @return std::vector<ClientConnection> client connection details
   */
  std::vector<ClientConnection> getConnectedClients() const;

 private:
  /**
   * @brief Time source for the node
   */
    std::shared_ptr<rclcpp::TimeSource> time_source_;

 protected:
  /**
   * @brief State transition callback for 'configure' transition
   *
   * @param state state
   *
   * @return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn success, failure, error
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& state) override;

  /**
   * @brief State transition callback for 'activate' transition
   *
   * @param state state
   *
   * @return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn success, failure, error
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& state) override;

  /**
   * @brief State transition callback for 'deactivate' transition
   *
   * @param state state
   *
   * @return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn success, failure, error
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& state) override;

  /**
   * @brief State transition callback for 'cleanup' transition
   *
   * @param state state
   *
   * @return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn success, failure, error
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(
      const rclcpp_lifecycle::State& state) override;

  /**
   * @brief State transition callback for 'shutdown' transition
   *
   * @param state state
   *
   * @return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn success, failure, error
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(
      const rclcpp_lifecycle::State& state) override;

  /**
   * @brief Sets up subscriptions for connected client.
   *
   * Clients get assigned unique IDs.
   *
   *
   */
  void registerClients();

  /**
   * @brief Callback for `tf2_msgs::msg::TFMessage` that groups the transforms by clients and
   * passes them to the corresponding buffer insertion function.
   *
   * @param   transforms     message containing transforms
   * @param   topic          ROS topic which triggers this callback
   * @param   stamp_override will be used as buffer stamp if present
   */
  void insertTransformPassthrough(const std::shared_ptr<const tf2_msgs::msg::TFMessage>& transforms,
                                  const std::string& topic,
                                  std::optional<rclcpp::Time> stamp_override = std::optional<rclcpp::Time>());

 protected:
  /**
   * @brief Waits until the ROS 2 time has started
   */
  void waitForTimeToStart();

  /**
   * @brief Loads ROS parameters used in the node.
   */
  void loadParameters();
  /**
   * @brief Loads ROS parameters for EventDetector clients
  */
  void loadClientParameters();
  /**
   * @brief Sets up BufferManager, DatabaseInterface, subscribers and
   * registers the clients.
   *
   */
  void setup();
  /**
   * @brief initializes buffer from bag
  */
  void initializeBufferFromBag();
  /**
   * @brief Callback template for inserting data into the buffer.
   *
   * Determines timestamp and passes it to buffer's insertion function.
   *
   * @tparam T supported data type as defined in `datatypes.macro`
   *
   * @param   sample          data message
   * @param   client_id       client ID
   * @param   topic           ROS topic which triggers this callback
   * @param   stamp_override  will be used as buffer stamp if present
   */
  template <typename T>
  void insertPassthrough(const std::shared_ptr<const T>& sample, const std::string& client_id, const std::string& topic,
                         std::optional<rclcpp::Time> stamp_override = std::optional<rclcpp::Time>());
  /**
   * @brief Function for retrieving a time stamp to associate with a received
   * message depending on buffer parameters
   */
  template <typename T>
  rclcpp::Time getSampleStamp(const T& sample);

  /**
   * @brief Callback for validating parameter changes
  */
  rcl_interfaces::msg::SetParametersResult onParametersSetCallback(const std::vector<rclcpp::Parameter>& params);

  /**
   * @brief Callback for acting upon parameter changes
  */
  void postParameterSetCallback(rcl_interfaces::msg::ParameterEvent::UniquePtr event);

  /**
   * @brief Callback handle for onParameterSetCallback
  */
  OnSetParametersCallbackHandle::SharedPtr callback_handle_;

  /**
   * @brief Client for receiving parameter changes
  */
  rclcpp::SyncParametersClient::SharedPtr parameters_client_;

  /**
   * @brief Subscription to parameter events
  */
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr param_sub_;

  /**
   * @brief buffer configuration
   */
  struct {
    double default_time;                           ///< default buffer time
    int default_queue_size;                        ///< default subscriber queue size
    bool use_msg_stamp;                            ///< whether to use msg stamp for sorting if possible
    std::vector<std::string> initialization_bags;  ///<  filepaths used for initializing buffer at startup
  } buffer_config_;

  /**
   * @brief analysis configuration
   */
  struct {
    double default_period;  ///< default analysis interval
  } analysis_config_;

  /**
   * @brief BufferManager component
   */
  std::shared_ptr<BufferManager> buffer_manager_;

  /**
   * @brief AnalysisManager component
   */
  std::shared_ptr<AnalysisManager> analysis_manager_;

  /**
   * @brief client connection details
   */
  std::vector<ClientConnection> connected_clients_;

  /**
   * @brief mutex for connected clients vector
   */
  mutable std::shared_mutex reconfigure_mutex_;

  /**
   * @brief ROS subscriber for transforms
   */
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr transforms_subscriber_;

  /**
   * ROS subscriber for static transforms
   */
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr static_transforms_subscriber_;

  /**
   * @brief ROS subscribers mapped to their topics
   */
#define DATATYPE(TYPE, VAR) std::map<std::string, rclcpp::Subscription<TYPE>::ConstSharedPtr> VAR##_subscribers_;
#include "event_detector/datatypes.macro"
#undef DATATYPE
};

}  // namespace event_detector

#include "event_detector/EventDetector.tpp"
