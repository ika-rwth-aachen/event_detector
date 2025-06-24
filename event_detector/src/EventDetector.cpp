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

#include "event_detector/EventDetector.hpp"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <functional>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <thread>

namespace event_detector
{

EventDetector::EventDetector(rclcpp::NodeOptions options)
    : rclcpp_lifecycle::LifecycleNode(
          "event_detector",
          options.allow_undeclared_parameters(true).automatically_declare_parameters_from_overrides(true)),
      time_source_(std::make_shared<rclcpp::TimeSource>()) {
  RCLCPP_INFO(this->get_logger(), "Transitioned to state '%s' (%d), waiting to be configured (%d)",
              this->get_current_state().label().c_str(), this->get_current_state().id(),
              lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  // Load startup state parameter
  auto startup_state = lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED;
  this->get_parameter_or("startup_state", startup_state, startup_state);
  if (startup_state > lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) this->configure();
  if (startup_state > lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) this->activate();
}

std::vector<ClientConnection> EventDetector::getConnectedClients() const {
  std::shared_lock lock{reconfigure_mutex_};
  return connected_clients_;
}

void EventDetector::loadParameters() {
  // buffer parameters
  this->get_parameter_or("buffer.default_time", buffer_config_.default_time, 10.0);
  this->get_parameter_or("buffer.default_queue_size", buffer_config_.default_queue_size, 20);
  this->get_parameter_or("buffer.use_msg_stamp", buffer_config_.use_msg_stamp, true);
  this->get_parameter_or("buffer.initialization_bags", buffer_config_.initialization_bags, {});

  // analysis parameters
  this->get_parameter_or("analysis.default_period", analysis_config_.default_period, 1.0);

  this->loadClientParameters();
}

void EventDetector::loadClientParameters() {
  connected_clients_.clear();
  // client parameters
  std::vector<std::string> clients;
  this->get_parameter_or("clients", clients, {});
  for (auto& client : clients) {
    ClientConnection connection;
    connection.name = client;
    this->get_parameter_or("client_params." + client + ".base_frame", connection.base_frame, std::string{});
    this->get_parameter_or("client_params." + client + ".tf_prefix", connection.tf_prefix, std::string{});
    // parse data entries
    std::vector<std::string> data_types;
    this->get_parameter_or("client_params." + client + ".data_types", data_types, {});
    if (!data_types.empty()) {
      for (auto& data_type : data_types) {
        std::vector<std::string> topics;
        this->get_parameter_or("client_params." + client + ".data_type_params." + data_type + ".topics", topics,
                               std::vector<std::string>{});
        std::vector<double> buffer_times;
        this->get_parameter_or("client_params." + client + ".data_type_params." + data_type + ".buffer_times", buffer_times,
                               std::vector<double>{});
        std::vector<int64_t> queue_sizes;
        this->get_parameter_or("client_params." + client + ".data_type_params." + data_type + ".queue_sizes", queue_sizes,
                               std::vector<int64_t>{});
        for (unsigned int k = 0; k < topics.size(); k++) {
          std::string topic = topics[k];
          double buffer_time = buffer_config_.default_time;
          int queue_size = buffer_config_.default_queue_size;
          // check if length of buffer time array matches length of topics array
          if (k < buffer_times.size()) {
            buffer_time = buffer_times[k];
          } else if (!buffer_times.empty()) {
            RCLCPP_WARN(this->get_logger(),
                        "Number of buffer times (%ld) does not match number of topics (%ld) for "
                        "client '%s', using default buffer time %.1fs",
                        buffer_times.size(), topics.size(), client.c_str(), buffer_config_.default_time);
          }
          // check if length of queue size array matches length of topics array
          if (k < queue_sizes.size()) {
            queue_size = queue_sizes[k];
          } else if (!queue_sizes.empty()) {
            RCLCPP_WARN(this->get_logger(),
                        "Number of queue sizes (%ld) does not match number of topics (%ld)"
                        "for client '%s', using default queue size %d",
                        queue_sizes.size(), topics.size(), client.c_str(), buffer_config_.default_queue_size);
          }

#define DATATYPE(TYPE, VAR) \
  if (data_type == #VAR) connection.VAR.push_back({topic, buffer_time, queue_size});
#include "event_detector/datatypes.macro"
#undef DATATYPE
        }
      }
    }
    connection.tf2_msgs__TFMessage.push_back(
        {kTransformationTopic, buffer_config_.default_time, buffer_config_.default_queue_size});
    connection.tf2_msgs__TFMessage.push_back(
        {kStaticTransformationTopic, 0.0 /* no duration limit */, buffer_config_.default_queue_size});
    connected_clients_.push_back(connection);
  }
}

void EventDetector::waitForTimeToStart() {
  // Check if the time source has started
  if (this->now().seconds() == 0.0) {
    RCLCPP_WARN(this->get_logger(), "Clock time is 0, waiting until time has started ...");
    while (rclcpp::ok() && this->now().seconds() == 0.0) {
      rclcpp::sleep_for(std::chrono::milliseconds(10));
    }
  }

  // log the clock type
  auto clock_type = this->now().get_clock_type();
  switch (clock_type) {
    case RCL_ROS_TIME:
      bool use_sim_time;
      this->get_parameter_or("use_sim_time", use_sim_time, false);
      RCLCPP_INFO(this->get_logger(), "Clock type is RCL_ROS_TIME");
      break;
    case RCL_SYSTEM_TIME:
      RCLCPP_INFO(this->get_logger(), "Clock type is RCL_SYSTEM_TIME");
      break;
    case RCL_STEADY_TIME:
      RCLCPP_INFO(this->get_logger(), "Clock type is RCL_STEADY_TIME");
      break;
    default:
      RCLCPP_FATAL(this->get_logger(), "Clock type is unknown");
      this->shutdown();
  }
}

void EventDetector::setup() {
  // wait until time has started
  this->waitForTimeToStart();

  // set up BufferManager
  buffer_manager_ = std::make_shared<BufferManager>();

  // register connected clients
  this->registerClients();

  // set up AnalysisManager
  {
    std::shared_lock lock{reconfigure_mutex_};
    analysis_manager_ = std::make_shared<AnalysisManager>(this, buffer_manager_, analysis_config_.default_period);
  }

  // initialize buffer from bag files
  this->initializeBufferFromBag();

  // subscribe dynamic transforms
  std::function<void(const std::shared_ptr<const tf2_msgs::msg::TFMessage>&)> transform_callback =
      std::bind(&EventDetector::insertTransformPassthrough, this, std::placeholders::_1, kTransformationTopic,
                std::optional<rclcpp::Time>());
  transforms_subscriber_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
      kTransformationTopic, buffer_config_.default_queue_size, transform_callback);
  RCLCPP_INFO(this->get_logger(), "Subscribed to '%s' on topic '%s' (buffer time: %.1fs, queue size: %d)",
              rosidl_generator_traits::data_type<tf2_msgs::msg::TFMessage>(), kTransformationTopic.c_str(),
              buffer_config_.default_time, buffer_config_.default_queue_size);

  // subscribe to static transforms
  std::function<void(const std::shared_ptr<const tf2_msgs::msg::TFMessage>&)> static_transform_callback =
      std::bind(&EventDetector::insertTransformPassthrough, this, std::placeholders::_1, kStaticTransformationTopic,
                std::optional<rclcpp::Time>());
  rclcpp::QoS static_transform_qos{static_cast<size_t>(buffer_config_.default_queue_size)};
  static_transform_qos.transient_local();  // for "latched"-like topics such as /tf_static
  static_transforms_subscriber_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
      kStaticTransformationTopic, static_transform_qos, static_transform_callback);
  RCLCPP_INFO(this->get_logger(), "Subscribed to '%s' on topic '%s' (queue size: %d)",
              rosidl_generator_traits::data_type<tf2_msgs::msg::TFMessage>(), kStaticTransformationTopic.c_str(),
              buffer_config_.default_queue_size);

  // set up parameter change callbacks
  callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&EventDetector::onParametersSetCallback, this, std::placeholders::_1));
  parameters_client_ = std::make_shared<rclcpp::SyncParametersClient>(this);
  param_sub_ = std::move(parameters_client_->on_parameter_event(
      std::bind(&EventDetector::postParameterSetCallback, this, std::placeholders::_1)));
}

void EventDetector::registerClients() {
  for (auto& client : connected_clients_) {
    // generate id by hashing name
    size_t hash = std::hash<std::string>()(client.name);
    std::string id = std::to_string(hash);
    id = std::string((24 - id.length()), '0') + id;
    client.id = id;
  }

// update buffers
#define DATATYPE(TYPE, VAR) buffer_manager_->VAR.update(connected_clients_);
#include "event_detector/datatypes.macro"
#undef DATATYPE

  // print buffer info
  RCLCPP_DEBUG(this->get_logger(), buffer_manager_->getInfo().c_str());

// update subscriptions for each type
#define DATATYPE(TYPE, VAR)                                                                                   \
  {                                                                                                           \
    std::map<std::string, rclcpp::Subscription<TYPE>::ConstSharedPtr> new_subscribers_;                            \
    for (auto& client : connected_clients_) {                                                                 \
      for (const auto& [topic, buffer_time, queue_size] : client.VAR) {                                       \
        if (topic == kTransformationTopic || topic == kStaticTransformationTopic)                             \
          continue; /* transform are special case */                                                          \
        std::function<void(const std::shared_ptr<const TYPE>&)> buffer_callback =                             \
            std::bind(&EventDetector::insertPassthrough<TYPE>, this, std::placeholders::_1, client.id, topic, \
                      std::optional<rclcpp::Time>());                                                         \
        new_subscribers_[topic] = VAR##_subscribers_.find(topic) != VAR##_subscribers_.end()                  \
                                      ? VAR##_subscribers_[topic]                                             \
                                      : this->create_subscription<TYPE>(topic, queue_size, buffer_callback);  \
        RCLCPP_INFO(this->get_logger(),                                                                       \
                    "Subscribed to '%s' on topic '%s' (buffer time: %.1fs, "                                  \
                    "queue size: %d)",                                                                        \
                    rosidl_generator_traits::data_type<TYPE>(), topic.c_str(), buffer_time, queue_size);      \
      }                                                                                                       \
    }                                                                                                         \
                                                                                                              \
    std::swap(VAR##_subscribers_, new_subscribers_);                                                          \
  }
#include "event_detector/datatypes.macro"
#undef DATATYPE
  // dummy subscription to trigger removal of expired weak_ptrs from callback
  // group subscription_ptrs (see https://github.com/ros2/rclcpp/blob/a19ad2134b17d4f00dc600aed4d897bb7b32b77f/rclcpp/src/rclcpp/callback_group.cpp#L183)
  this->create_subscription<geometry_msgs::msg::PoseStamped>("dummy", 1,
                                                             [](geometry_msgs::msg::PoseStamped::SharedPtr) {});
}

void EventDetector::initializeBufferFromBag() {
  for (const std::string& filepath : buffer_config_.initialization_bags) {
    RCLCPP_INFO(this->get_logger(), "Reading messages from bag file '%s' into buffer ...", filepath.c_str());
    long msg_count_without_tf = 0;
    rosbag2_cpp::Reader reader;
    reader.open(filepath);
    while (reader.has_next()) {
      auto m = reader.read_next();
      if (m->topic_name == kStaticTransformationTopic || m->topic_name == kTransformationTopic) {
        std::shared_ptr<tf2_msgs::msg::TFMessage> deserialized_msg = std::make_shared<tf2_msgs::msg::TFMessage>();
        rclcpp::SerializedMessage extracted_serialized_msg(*m->serialized_data);
        rclcpp::Serialization<tf2_msgs::msg::TFMessage> serialization;
        serialization.deserialize_message(&extracted_serialized_msg, deserialized_msg.get());
        this->insertTransformPassthrough(deserialized_msg, m->topic_name, rclcpp::Time(m->recv_timestamp, RCL_ROS_TIME));
      } else {
        // find client that message belongs to and insert it into its buffer
        for (const ClientConnection& client : connected_clients_) {
          auto pred = [&m](const std::tuple<std::string, double, int>& elem) {
            return m->topic_name == std::get<0>(elem);
          };
#define DATATYPE(TYPE, VAR)                                                                                \
  if (std::find_if(client.VAR.begin(), client.VAR.end(), pred) != client.VAR.end()) {                      \
    std::shared_ptr<TYPE> deserialized_msg = std::make_shared<TYPE>();                                     \
    rclcpp::SerializedMessage extracted_serialized_msg(*m->serialized_data);                               \
    rclcpp::Serialization<TYPE> serialization;                                                             \
    serialization.deserialize_message(&extracted_serialized_msg, deserialized_msg.get());                  \
    this->insertPassthrough<TYPE>(deserialized_msg, client.id, m->topic_name,                              \
                                  std::optional<rclcpp::Time>(rclcpp::Time(m->recv_timestamp, RCL_ROS_TIME))); \
    msg_count_without_tf++;                                                                                \
    break;                                                                                                 \
  }
#include "event_detector/datatypes.macro"
#undef DATATYPE
        }
      }
    }
    RCLCPP_INFO(this->get_logger(), "Read %ld messages (excl. tf) from bag file '%s' into buffer", msg_count_without_tf,
                filepath.c_str());
  }
  if (!buffer_config_.initialization_bags.empty()) RCLCPP_INFO(this->get_logger(), buffer_manager_->getInfo().c_str());
}

void EventDetector::insertTransformPassthrough(const std::shared_ptr<const tf2_msgs::msg::TFMessage>& transforms,
                                               const std::string& topic, std::optional<rclcpp::Time> stamp_override) {
  std::shared_lock lock{reconfigure_mutex_};

  string_map<std::shared_ptr<tf2_msgs::msg::TFMessage>> client_transforms;

  // loop over transforms included in message
  for (const auto& tf : transforms->transforms) {
    // try to determine involved client
    std::string client_id;
    for (const auto& client : connected_clients_) {
      if (!client.tf_prefix.empty() && tf.child_frame_id.rfind(client.tf_prefix, 0) == 0) {  // startswith
        client_id = client.id;
        break;
      }
    }
    if (client_id.empty()) {
      RCLCPP_WARN_ONCE(this->get_logger(),
                       "Could not match transform from frame_id='%s' to child_frame_id='%s' "
                       "to any client's 'tf_prefix', skipping. This warning will only be shown once.",
                       tf.header.frame_id.c_str(), tf.child_frame_id.c_str());
      continue;
    }

    // collect transforms by client
    if (client_transforms.find(client_id) == client_transforms.end())
      client_transforms[client_id] = std::make_shared<tf2_msgs::msg::TFMessage>();
    client_transforms[client_id]->transforms.push_back(tf);
  }

  // pass transforms to client-specific buffers
  for (const auto& kv : client_transforms) {
    const std::string& id = kv.first;
    const auto& tfs = kv.second;
    buffer_manager_->tf2_msgs__TFMessage.insert(tfs, stamp_override.value_or(this->now()), id, topic);
  }
}

rcl_interfaces::msg::SetParametersResult EventDetector::onParametersSetCallback(
    const std::vector<rclcpp::Parameter>& parameters) {
  RCLCPP_DEBUG(this->get_logger(), "onParametersSetCallback");

  // rudimentary check to reject parameter changes not related to client parameters
  for (const rclcpp::Parameter& parameter : parameters) {
    if (parameter.get_name().find("client") == std::string::npos) {
      rcl_interfaces::msg::SetParametersResult result;

      result.successful = false;
      result.reason = "Dynamic reconfiguration of " + parameter.get_name() +
                      " is not supported. Only 'client' parameters can be reconfigured.";
      return result;
    }
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
}

void EventDetector::postParameterSetCallback(rcl_interfaces::msg::ParameterEvent::UniquePtr event) {
  RCLCPP_DEBUG(this->get_logger(), "postParametersSetCallback");

  if (event->node != this->get_node_base_interface()->get_fully_qualified_name()) return;

  std::unique_lock lock{reconfigure_mutex_};

  this->loadClientParameters();
  this->registerClients();
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn EventDetector::on_configure(
    const rclcpp_lifecycle::State& state) {
  RCLCPP_INFO(this->get_logger(), "Configuring (%d) from state '%s' (%d) to state 'inactive' (%d) ...",
              lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE, state.label().c_str(), state.id(),
              lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  {
    std::shared_lock lock{reconfigure_mutex_};

    // Attach the node interfaces to the TimeSource
    time_source_->attachNode(this->get_node_base_interface(), this->get_node_topics_interface(),
                             this->get_node_graph_interface(), this->get_node_services_interface(),
                             this->get_node_logging_interface(), this->get_node_clock_interface(),
                             this->get_node_parameters_interface());

    this->loadParameters();
    this->setup();
  }

  RCLCPP_INFO(this->get_logger(), "Transitioned to state 'inactive' (%d), waiting to be activated (%d)",
              lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn EventDetector::on_activate(
    const rclcpp_lifecycle::State& state) {
  RCLCPP_INFO(this->get_logger(), "Activating (%d) from state '%s' (%d) to state 'active' (%d) ...",
              lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE, state.label().c_str(), state.id(),
              lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  {
    std::shared_lock lock{reconfigure_mutex_};
    analysis_manager_->toggleAnalysis(true, true);
  }

  RCLCPP_INFO(this->get_logger(), "Transitioned to state 'active' (%d)",
              lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn EventDetector::on_deactivate(
    const rclcpp_lifecycle::State& state) {
  RCLCPP_INFO(this->get_logger(), "Deactivating (%d) from state '%s' (%d) to state 'inactive' (%d) ...",
              lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE, state.label().c_str(), state.id(),
              lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  {
    std::shared_lock lock{reconfigure_mutex_};
    analysis_manager_->toggleAnalysis(false, false);
  }

  RCLCPP_INFO(this->get_logger(), "Transitioned to state 'inactive' (%d)",
              lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn EventDetector::on_cleanup(
    const rclcpp_lifecycle::State& state) {
  RCLCPP_INFO(this->get_logger(), "Cleaning up (%d) from state '%s' (%d) to state 'unconfigured' (%d) ...",
              lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP, state.label().c_str(), state.id(),
              lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);

  {
    std::shared_lock lock{reconfigure_mutex_};

    // cleanup parameter change callbacks
    callback_handle_.reset();
    parameters_client_.reset();
    param_sub_.reset();

    // cleanup transform subscribers
    transforms_subscriber_.reset();
    static_transforms_subscriber_.reset();

    // cleanup analysis manager
    analysis_manager_.reset();

    // cleanup connected clients and their subscriptions
    connected_clients_.clear();
#define DATATYPE(TYPE, VAR) VAR##_subscribers_.clear();
#include "event_detector/datatypes.macro"
#undef DATATYPE

    // cleanup buffer manager
    buffer_manager_.reset();
  }

  RCLCPP_INFO(this->get_logger(), "Transitioned to state 'unconfigured' (%d)",
              lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn EventDetector::on_shutdown(
    const rclcpp_lifecycle::State& state) {
  RCLCPP_INFO(this->get_logger(), "Shutting down from state '%s' (%d)", state.label().c_str(), state.id());
  time_source_->detachNode();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

}  // namespace event_detector

RCLCPP_COMPONENTS_REGISTER_NODE(event_detector::EventDetector)
