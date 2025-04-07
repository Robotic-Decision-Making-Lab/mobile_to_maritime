// Copyright 2024, Evan Palmer
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include "message_transforms/message_transforms_node.hpp"

#include <ranges>

#include "rclcpp/rclcpp.hpp"

namespace m2m
{

MessageTransforms::MessageTransforms(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("message_transforms", options)
{
}

auto MessageTransforms::on_configure(const rclcpp_lifecycle::State & /*state*/) -> CallbackReturn
{
  param_listener_ = std::make_shared<message_transforms::ParamListener>(get_node_parameters_interface());
  params_ = param_listener_->get_params();

  // Lambda used to filter the endpoints to include only publishers from other nodes
  auto get_publisher_endpoints = [this](const std::string & topic) -> std::vector<rclcpp::TopicEndpointInfo> {
    std::vector<rclcpp::TopicEndpointInfo> info = get_publishers_info_by_topic(topic);
    auto is_publisher = [this](const rclcpp::TopicEndpointInfo & endpoint) {
      return endpoint.endpoint_type() == rclcpp::EndpointType::Publisher &&
             (endpoint.node_name() != get_name() || endpoint.node_namespace() != get_namespace());
    };
    auto filtered_endpoints = info | std::views::filter(is_publisher);
    return {filtered_endpoints.begin(), filtered_endpoints.end()};
  };

  for (const auto & topic : params_.incoming_topics) {
    // Wait for up to 5 seconds for publishers to be found on the topic
    // This gives us a chance to get the QoS profile of the publisher instead of immediately using the system defaults
    std::vector<rclcpp::TopicEndpointInfo> info;
    const std::chrono::time_point<std::chrono::system_clock> start = std::chrono::system_clock::now();
    do {
      info = get_publisher_endpoints(topic);
    } while (info.empty() && std::chrono::system_clock::now() - start < std::chrono::seconds(5));

    if (info.empty()) {
      RCLCPP_WARN(get_logger(), "No publishers found for topic %s", topic.c_str());
    }

    // If no publishers are found, use the system defaults. Otherwise, use the QoS profile of the last publisher
    auto qos = info.empty() ? rclcpp::SystemDefaultsQoS() : info.back().qos_profile();

    // The history policy can be set to unknown when the QoS profile is copied. See the following for more
    // information:
    // https://github.com/foxglove/ros-foxglove-bridge/blob/26d5fac4d11b34edc583ff63236910c94f4416e7/ros2_foxglove_bridge/src/ros2_foxglove_bridge.cpp#L657
    if (qos.history() == rclcpp::HistoryPolicy::Unknown) {
      qos.history(rclcpp::HistoryPolicy::SystemDefault);
    }

    const std::string message_type = params_.transforms.incoming_topics_map[topic].message_type;
    const std::string outgoing_topic = params_.transforms.incoming_topics_map[topic].outgoing_topic;

    RCLCPP_INFO(
      get_logger(),
      "Transforming messages of type %s from the topic %s and republishing to the topic %s",
      message_type.c_str(),
      topic.c_str(),
      outgoing_topic.c_str());

    const std::string frame_id = params_.transforms.incoming_topics_map[topic].frame_id;
    const std::string child_frame_id = params_.transforms.incoming_topics_map[topic].child_frame_id;

    if (frame_id.empty()) {
      RCLCPP_DEBUG(get_logger(), "No frame_id configured for topic %s", topic.c_str());
    }
    if (child_frame_id.empty()) {
      RCLCPP_DEBUG(get_logger(), "No child_frame_id configured for topic %s", topic.c_str());
    }

    // Find the relevant transform registration function and execute it
    // We just log a warning for missing frame_ids - while the output frames will be empty, it won't break things
    if (transform_map_.find(message_type) != transform_map_.end()) {
      transform_map_[message_type](topic, outgoing_topic, qos);
    } else if (transform_stamped_map_.find(message_type) != transform_stamped_map_.end()) {
      transform_stamped_map_[message_type](topic, outgoing_topic, frame_id, qos);
    } else if (transform_stamped_child_map_.find(message_type) != transform_stamped_child_map_.end()) {
      transform_stamped_child_map_[message_type](topic, outgoing_topic, frame_id, child_frame_id, qos);
    } else {
      RCLCPP_WARN(get_logger(), "No transform found for message type %s", message_type.c_str());
      return CallbackReturn::ERROR;
    }
  }

  return CallbackReturn::SUCCESS;
}

}  // namespace m2m

auto main(int argc, char * argv[]) -> int
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<m2m::MessageTransforms>();
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
