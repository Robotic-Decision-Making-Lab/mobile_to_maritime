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

#include "message_transforms/transforms.hpp"

#include "rclcpp/rclcpp.hpp"

namespace m2m::transforms
{

MessageTransforms::MessageTransforms(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("message_transforms", options)
{
}

auto MessageTransforms::on_configure(const rclcpp_lifecycle::State & /*state*/) -> CallbackReturn
{
  try {
    param_listener_ = std::make_shared<message_transforms::ParamListener>(get_node_parameters_interface());
    params_ = param_listener_->get_params();
  }
  catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to get MessageTransforms parameters: %s", e.what());
    return CallbackReturn::ERROR;
  }

  for (const auto & topic : params_.incoming_topics) {
    std::vector<rclcpp::TopicEndpointInfo> info = get_publishers_info_by_topic(topic);

    // Filter the endpoints to include only publishers from other nodes
    const auto endpoint_it = std::ranges::find_if(info, [this](const auto & endpoint) {
      return endpoint.endpoint_type() == rclcpp::EndpointType::Publisher &&
             (endpoint.node_name() != get_name() || endpoint.node_namespace() != get_namespace());
    });

    if (endpoint_it == info.end()) {
      RCLCPP_WARN(get_logger(), "No publishers found for topic %s", topic.c_str());
    }

    auto qos = endpoint_it == info.end() ? rclcpp::SystemDefaultsQoS() : endpoint_it->qos_profile();

    // The history policy can be set to unknown when the QoS profile is copied. See the following for more
    // information:
    // https://github.com/foxglove/ros-foxglove-bridge/blob/26d5fac4d11b34edc583ff63236910c94f4416e7/ros2_foxglove_bridge/src/ros2_foxglove_bridge.cpp#L657
    if (qos.history() == rclcpp::HistoryPolicy::Unknown) {
      qos.history(rclcpp::HistoryPolicy::SystemDefault);
    }

    const std::string message_type = params_.transforms.incoming_topics_map[topic].message_type;

    if (subscription_map_.find(message_type) == subscription_map_.end()) {
      RCLCPP_WARN(get_logger(), "Unsupported message type %s for topic %s", message_type.c_str(), topic.c_str());
    } else {
      subscription_map_[message_type](topic, params_.transforms.incoming_topics_map[topic].outgoing_topics, qos);
    }
  }

  return CallbackReturn::SUCCESS;
}

auto MessageTransforms::transform_message(const std::shared_ptr<geometry_msgs::msg::Pose> & message) -> void
{
  message->position.y *= -1;
  message->position.z *= -1;

  // TODO(evan): rotation the pose
}

auto MessageTransforms::transform_message(const std::shared_ptr<geometry_msgs::msg::Twist> & message) -> void
{
  message->linear.y *= -1;
  message->linear.z *= -1;

  message->angular.x *= -1;
  message->angular.y *= -1;
}

auto MessageTransforms::transform_message(const std::shared_ptr<geometry_msgs::msg::Wrench> & message) -> void {}

auto MessageTransforms::transform_message(const std::shared_ptr<geometry_msgs::msg::PoseStamped> & message) -> void {}

auto MessageTransforms::transform_message(const std::shared_ptr<geometry_msgs::msg::TwistStamped> & message) -> void {}

auto MessageTransforms::transform_message(const std::shared_ptr<geometry_msgs::msg::WrenchStamped> & message) -> void {}

auto MessageTransforms::transform_message(const std::shared_ptr<nav_msgs::msg::Odometry> & message) -> void {}

}  // namespace m2m::transforms

auto main(int argc, char * argv[]) -> int
{
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor executor;

  auto node = std::make_shared<m2m::transforms::MessageTransforms>();
  executor.add_node(node->get_node_base_interface());

  executor.spin();

  rclcpp::shutdown();

  return 0;
}
